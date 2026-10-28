// vimodetest: Wii VI resolution / refresh-rate test bench.
//
// Boots into the console's normal safe video mode (VIDEO_GetPreferredMode) and shows a list
// of 24 test modes. Selecting one programs the VI directly with computed timing (and the VI
// PLL when a faster pixel clock is needed), then shows test patterns. START (GC pad) or
// HOME / + (Wii Remote) returns to the list at the safe mode from anywhere.
//
// How the VI is driven (see docs/VI_PLL.md in this repo):
//   * Pixel clock: F = 54 MHz * Nfb / V  (Nfb = 0x1C4[15:8]+1, V = 0x1C8[23:20]), VICLK=1
//     passes F through, VICLK=0 halves it. Pixel rate on the bus P = F/2.
//   * Horizontal (all in pixel clocks): line total = 2*HLW, hsync width HSY from line start,
//     active video starts at HBE (from line start) and blanking restarts at HLW+HBS.
//   * Vertical (per field, in half-lines): 3*EQU + PRB + 2*ACV + PSB. Interlaced modes use an
//     odd half-line count; progressive/double-strike use an even one with NIN set in DCR.
//   * libogc is never told about the custom mode. It stays configured for the safe mode, the
//     custom registers are written straight to the hardware, and returning simply re-runs
//     VIDEO_Configure(safe) + VIDEO_Flush, which rewrites every register touched here.
//
// Rendering uses GX (so the normal RGB->YUV 4:2:2 copy path is exercised) in tiles of at
// most 640x528 EFB pixels. Field-rendered modes draw each field as its own half-height pass
// with the correct half-line offset, into an interleaved (double-field) framebuffer.
// "Double wide chroma" modes render 720 logical pixels and output each one twice, so every
// logical pixel owns a whole 4:2:2 chroma pair on the bus.

#include <gccore.h>
#include <fat.h>
#include <wiiuse/wpad.h>
#include <ogc/pad.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <malloc.h>
#include <ogc/machine/processor.h>
#include <ogc/lwp_watchdog.h>

#define VI    ((volatile u16 *)0xCC002000)
#define HW(o) (*(volatile u32 *)(0xCD800000 + (o)))

extern u8 console_font_8x16[];   // libogc's console font: 256 glyphs, 16 bytes each

// ======================================================================= clocks
typedef struct { u8 viclk, nfb, v; u32 khz; const char *desc; } Clock;
enum { C27, C54, C96, C108, C138, C148 };
static const Clock clocks[] = {
    { 0,  7, 7,  27000, "27 MHz (stock, VICLK=0)" },
    { 1,  7, 7,  54000, "54 MHz (stock, VICLK=1)" },
    { 1, 16, 9,  96000, "96 MHz (Nfb16 V9)" },
    { 1, 14, 7, 108000, "108 MHz (Nfb14 V7)" },
    { 1, 23, 9, 138000, "138 MHz (Nfb23 V9)" },
    { 1, 22, 8, 148500, "148.5 MHz (Nfb22 V8)" },
};

// ======================================================================= timings
// Horizontal, in pixel clocks: total, active, hsync width, back porch (active start = S+BP),
// and optionally the colour-burst window (0 = libogc's 71/105, clipped into the blanking).
typedef struct { u16 T, A, S, BP; u8 hcs, hce; } HTim;
// Vertical: interlaced?, half-lines per field (N), active lines per field, EQU (vsync length
// in half-lines), PSB of the odd field, and optionally the burst-blanking start/end of the odd
// field (0 = derived from EQU and N, which reproduces libogc's stock tables).
typedef struct { u8 il; u16 N, acv; u8 equ; u16 psb; u16 bs, be; } VTim;

static const HTim H_SD        = {  858,  720,  64,  58 };  // BT.601 / BT.1358
static const HTim H_SD2       = { 1716, 1440, 124, 120 };  // the above at twice the clock
static const HTim H_1280_960  = { 1526, 1280, 112, 104 };
// 1080i family: Swiss's 1080i60 horizontal (16-clock sync, picture 75..795 of 800), which
// matches SMPTE 274M's sync and back-porch durations; scaled for the wider variants.
static const HTim H_1080_720  = {  800,  720,  16,  59, 21,  55 };
static const HTim H_1080_1440 = { 1600, 1440,  32, 118, 42, 110 };
static const HTim H_1080_1920 = { 2046, 1920,  44,  74, 54, 117 };  // HLW=1023: 10-bit limit
static const HTim H_720_960   = { 1200,  960,  30, 160 };
static const HTim H_720_1280  = { 1650, 1280,  40, 220 };  // SMPTE 296M
// 720p at the stock 54 MHz clock (27 MHz pixels): 45 kHz x 750 lines leaves a 600-pixel
// line. 296M's sync/porch proportions scaled to 600 give ~466 active; 480 with slightly
// tighter porches (sync 16, back 72, front 32).
static const HTim H_720_480   = {  600,  480,  16,  72, 21,  55 };

static const VTim V_240P  = { 0,  526, 240,  6, 4 };  // libogc stock 240p
static const VTim V_480I  = { 1,  525, 240,  6, 3 };  // libogc stock 480i
static const VTim V_480P  = { 0, 1050, 480, 12, 6 };  // libogc stock 480p (BT.1358)
static const VTim V_960I  = { 1, 1049, 480, 12, 5 };  // Swiss 960i: prb 48/49, psb 5/4
static const VTim V_1080I = { 1, 1125, 540,  5, 1, 40, 1120 };  // Swiss 1080i60
static const VTim V_720P  = { 0, 1500, 720, 10, 2 };  // SMPTE 296M line count
static const VTim V_720I  = { 1,  751, 360,  6, 3 };  // 720 lines interlaced, 751/frame

typedef struct {
    const char *name, *aspect;
    u8 clk;
    const HTim *h;
    const VTim *v;
    u8 dbl;     // double wide chroma: 720 logical pixels, each output twice
    u8 field;   // field rendering (each field drawn as its own pass)
} Mode;

#define NMODES 25
static const Mode modes[NMODES] = {
    { "240p",                        "4:3",  C27,  &H_SD,        &V_240P,  0, 0 },
    { "240p (double wide chroma)",   "4:3",  C54,  &H_SD2,       &V_240P,  1, 0 },
    { "1440x240p",                   "4:3",  C54,  &H_SD2,       &V_240P,  0, 0 },
    { "240p 120Hz",                  "4:3",  C54,  &H_SD,        &V_240P,  0, 0 },
    { "480i",                        "4:3",  C27,  &H_SD,        &V_480I,  0, 0 },
    { "480i (double wide chroma)",   "4:3",  C54,  &H_SD2,       &V_480I,  1, 0 },
    { "1440x480i",                   "4:3",  C54,  &H_SD2,       &V_480I,  0, 0 },
    { "480i 120Hz",                  "4:3",  C54,  &H_SD,        &V_480I,  0, 0 },
    { "480p",                        "4:3",  C54,  &H_SD,        &V_480P,  0, 0 },
    { "480p (double wide chroma)",   "4:3",  C108, &H_SD2,       &V_480P,  1, 0 },
    { "1440x480p",                   "4:3",  C108, &H_SD2,       &V_480P,  0, 0 },
    { "480p 120Hz",                  "4:3",  C108, &H_SD,        &V_480P,  0, 0 },
    { "720x960i",                    "4:3",  C54,  &H_SD,        &V_960I,  0, 1 },
    { "720x960i (dbl wide chroma)",  "4:3",  C108, &H_SD2,       &V_960I,  1, 1 },
    { "1280x960i",                   "4:3",  C96,  &H_1280_960,  &V_960I,  0, 1 },
    { "720x960i 120Hz",              "4:3",  C108, &H_SD,        &V_960I,  0, 1 },
    { "720x1080i",                   "4:3",  C54,  &H_1080_720,  &V_1080I, 0, 1 },
    { "1440x1080i",                  "4:3",  C108, &H_1080_1440, &V_1080I, 0, 1 },
    { "1920x1080i",                  "16:9", C138, &H_1080_1920, &V_1080I, 0, 1 },
    { "720x1080i 120Hz",             "4:3",  C108, &H_1080_720,  &V_1080I, 0, 1 },
    { "960x720p",                    "4:3",  C108, &H_720_960,   &V_720P,  0, 0 },
    { "1280x720p",                   "16:9", C148, &H_720_1280,  &V_720P,  0, 0 },
    { "960x720i 120Hz",              "4:3",  C108, &H_720_960,   &V_720I,  0, 1 },
    { "1280x720i 120Hz",             "16:9", C148, &H_720_1280,  &V_720I,  0, 1 },
    { "480x720p (54 MHz)",           "16:9", C54,  &H_720_480,   &V_720P,  0, 0 },
};
static const char *group_name[6] = { "240p", "480i", "480p", "960i", "1080i", "720p / 720i" };

// Everything derived from a Mode.
typedef struct {
    const Mode *m;
    u16 A, frameH, logW;    // physical width, frame lines, logical width
    u8 sx;                  // physical pixels per logical pixel (1, or 2 for dbl)
    u8 il;
    u16 N;
    u16 HLW, HBE, HBS, HSY, hcs, hce;
    u16 prbO, prbE, psbO, psbE, bsO, bsE, beO, beE;
    u32 exp_line_hz, exp_field_mhz;   // expected rates
    u32 xfb_bytes;
} Calc;

static void calc_mode(const Mode *m, Calc *c)
{
    const HTim *h = m->h;
    const VTim *v = m->v;
    memset(c, 0, sizeof(*c));
    c->m = m;
    c->A = h->A;
    c->il = v->il;
    c->N = v->N;
    c->frameH = v->il ? 2 * v->acv : v->acv;
    c->sx = m->dbl ? 2 : 1;
    c->logW = h->A / c->sx;
    c->HLW = h->T / 2;
    c->HSY = h->S;
    c->HBE = h->S + h->BP;
    c->HBS = c->HBE + h->A - c->HLW;
    // colour-burst window (only used by the analog encoder), kept inside the blanking
    c->hcs = h->hcs ? h->hcs : 71;
    c->hce = h->hce ? h->hce : 105;
    if (c->hce >= c->HBE) c->hce = c->HBE - 1;
    if (c->hcs >= c->hce) c->hcs = c->hce - 1;
    c->psbO = v->psb;
    c->prbO = v->N - 3 * v->equ - 2 * v->acv - v->psb;
    c->prbE = c->prbO + v->il;
    c->psbE = c->psbO - v->il;
    c->bsO = v->bs ? v->bs : 2 * v->equ;
    c->bsE = c->bsO + v->il;
    c->beO = v->be ? v->be : v->N - v->equ + v->il;
    c->beE = c->beO - v->il;
    u32 p_hz = clocks[m->clk].khz * 500u;            // F/2 in Hz
    c->exp_line_hz = p_hz / h->T;
    c->exp_field_mhz = (u32)((u64)c->exp_line_hz * 2000ull / v->N);
    c->xfb_bytes = (u32)h->A * c->frameH * 2;
}

// ======================================================================= state
static GXRModeObj *safe;
static void *menu_xfb;
static void *xfb[2];
static u32 xfb_max = 1920 * 1080 * 2;
static u32 boot1c4, boot1c8;
static volatile u32 retraces;
static bool fat_ok;
static u8 gpfifo[512 * 1024] ATTRIBUTE_ALIGN(32);
static u8 checker_tex[64] ATTRIBUTE_ALIGN(32);
static GXTexObj checker_obj;

typedef struct { bool valid; u32 line_hz, field_mhz, maxline; } Meas;
static Meas last_meas[NMODES];

static void on_retrace(u32 c) { (void)c; retraces++; }

// Heartbeat: blink the disc-slot LED (HW_GPIOB_OUT bit 5) about twice a second from the main
// thread. If the picture goes black but the LED keeps blinking, the Wii is fine and the
// display chain is what stopped; if the LED freezes, the Wii itself hung.
#define GPIOB_OUT (*(volatile u32 *)0xCD0000C0)
static u64 hb_last;
static void heartbeat(void)
{
    u64 now = gettime();
    if (now - hb_last >= millisecs_to_ticks(250)) { GPIOB_OUT ^= 0x20; hb_last = now; }
}

static void tb_delay_ms(u32 ms)
{
    u64 to = gettime() + millisecs_to_ticks(ms);
    while (gettime() < to) heartbeat();
}
// Waits for the next VI interrupt, but never longer than ms (returns false on timeout).
static bool wait_retrace(u32 ms)
{
    u32 r = retraces; u64 to = gettime() + millisecs_to_ticks(ms);
    while (retraces == r && gettime() < to) heartbeat();
    return retraces != r;
}
static u32 dpv(void) { return VI[0x2C / 2] & 0x7FF; }

// ======================================================================= logging
// Appends (never truncates), one open/close per line, so the file survives a hang and a
// relaunch: the last line written before a lockup shows what the app was doing.
static void logf_(const char *fmt, ...)
{
    if (!fat_ok) return;
    static const char *const paths[] = { "usb:/vimodetest.txt", "sd:/vimodetest.txt" };
    char line[320];
    va_list ap; va_start(ap, fmt); vsnprintf(line, sizeof(line), fmt, ap); va_end(ap);
    for (int i = 0; i < 2; i++) {
        FILE *f = fopen(paths[i], "a");
        if (f) { fputs(line, f); fputc('\n', f); fclose(f); }
    }
}

// ======================================================================= VI programming
static void set_fbb(const Calc *c, void *fb)
{
    u32 t = MEM_VIRTUAL_TO_PHYSICAL(fb);
    u32 b = c->il ? t + (u32)c->A * 2 : t;   // interlaced: bottom field starts one line down
    u32 flag = 0;
    if (t >= 0x01000000 || b >= 0x01000000) { flag = 1; t >>= 5; b >>= 5; }
    VI[0x1C / 2] = (u16)((flag << 12) | ((t >> 16) & 0xFF));
    VI[0x1E / 2] = (u16)(t & 0xFFFF);
    VI[0x24 / 2] = (u16)((b >> 16) & 0xFF);
    VI[0x26 / 2] = (u16)(b & 0xFFFF);
}

static u32 pll_1c4(const Clock *k)
{
    if (k->nfb == 7 && k->v == 7) return boot1c4;
    return (boot1c4 & ~0x0000FF00u) | ((u32)(k->nfb - 1) << 8);    // byte2 never touched
}
static u32 pll_1c8(const Clock *k)
{
    if (k->nfb == 7 && k->v == 7) return boot1c8;
    return (boot1c8 & ~0x00F00000u) | ((u32)k->v << 20);
}

static void program_vi(const Calc *c, void *fb)
{
    const Clock *k = &clocks[c->m->clk];
    const VTim *v = c->m->v;
    u16 wpl = c->A / 16;
    u16 std = c->il ? wpl * 2 : wpl;
    u16 dcr = (VI[0x02 / 2] & 0x0300) | 0x0001 | (c->il ? 0 : 0x0004); // keep FMT, ENB, NIN
    u32 p4 = pll_1c4(k), p8 = pll_1c8(k);

    logf_("  regs: VTR=%04X DCR=%04X HTR0=%04X%04X HTR1=%04X%04X VTO=%04X%04X VTE=%04X%04X",
          (v->acv << 4) | v->equ, dcr, (c->hcs << 8) | c->hce, c->HLW,
          ((c->HBE >> 9) & 1) | (c->HBS << 1), ((c->HBE & 0x1FF) << 7) | c->HSY,
          c->psbO, c->prbO, c->psbE, c->prbE);
    logf_("  regs: PICCONF=%04X DI0=%04X/%04X VICLK=%u 0x1C4=%08X 0x1C8=%08X",
          (wpl << 8) | std, 0x1000 | (c->N / 2 + 1), ((c->N & 1) ? c->HLW : 0) + 1,
          k->viclk, (unsigned)p4, (unsigned)p8);

    // 1. Stop the VI (no scanout, no framebuffer reads) before touching its clock: the PLL
    //    briefly runs unlocked after CLKF changes, and the VI should not be fetching from
    //    memory on a glitching clock.
    u32 l; _CPU_ISR_Disable(l);
    VI[0x02 / 2] = VI[0x02 / 2] & ~0x0001;
    _CPU_ISR_Restore(l);
    tb_delay_ms(2);
    // 2. Reprogram the PLL and let it relock while the VI is idle.
    bool pll_change = (HW(0x1C4) & 0x7FFFFFFFu) != p4 || HW(0x1C8) != p8;
    _CPU_ISR_Disable(l);
    HW(0x1C4) = p4;
    HW(0x1C8) = p8;
    _CPU_ISR_Restore(l);
    if (pll_change) tb_delay_ms(50);
    // 3. Load the new timing, then enable (DCR last).
    _CPU_ISR_Disable(l);
    VI[0x6C / 2] = k->viclk;
    VI[0x00 / 2] = (u16)((v->acv << 4) | v->equ);
    VI[0x04 / 2] = (u16)((c->hcs << 8) | c->hce);
    VI[0x06 / 2] = c->HLW;
    VI[0x08 / 2] = (u16)(((c->HBE >> 9) & 1) | (c->HBS << 1));
    VI[0x0A / 2] = (u16)(((c->HBE & 0x1FF) << 7) | c->HSY);
    VI[0x0C / 2] = c->psbO;  VI[0x0E / 2] = c->prbO;
    VI[0x10 / 2] = c->psbE;  VI[0x12 / 2] = c->prbE;
    VI[0x14 / 2] = (u16)((c->beO << 5) | c->bsO);
    VI[0x16 / 2] = (u16)((c->beO << 5) | c->bsO);
    VI[0x18 / 2] = (u16)((c->beE << 5) | c->bsE);
    VI[0x1A / 2] = (u16)((c->beE << 5) | c->bsE);
    set_fbb(c, fb);
    VI[0x30 / 2] = (u16)(0x1000 | (c->N / 2 + 1));
    VI[0x32 / 2] = (u16)(((c->N & 1) ? c->HLW : 0) + 1);
    VI[0x48 / 2] = (u16)((wpl << 8) | std);
    VI[0x4A / 2] = 0x0100;      // horizontal scaler off: 1 fetched pixel per output pixel
    VI[0x02 / 2] = dcr;         // enable last
    _CPU_ISR_Restore(l);
}

static void restore_safe(void)
{
    // Same order as entering a mode: VI off, clock back to boot, settle, VI on, then let
    // libogc rewrite every timing register for the safe mode at the next retrace.
    u32 l; _CPU_ISR_Disable(l);
    u16 dcr = VI[0x02 / 2];
    VI[0x02 / 2] = dcr & ~0x0001;
    _CPU_ISR_Restore(l);
    tb_delay_ms(2);
    bool pll_change = (HW(0x1C4) & 0x7FFFFFFFu) != boot1c4 || HW(0x1C8) != boot1c8;
    _CPU_ISR_Disable(l);
    HW(0x1C4) = boot1c4;
    HW(0x1C8) = boot1c8;
    _CPU_ISR_Restore(l);
    if (pll_change) tb_delay_ms(50);
    _CPU_ISR_Disable(l);
    VI[0x02 / 2] = dcr | 0x0001;   // custom timing at the stock clock, just until the flush
    _CPU_ISR_Restore(l);
    VIDEO_Configure(safe);         // marks every timing register for rewrite
    VIDEO_SetNextFramebuffer(menu_xfb);
    VIDEO_SetBlack(false);
    VIDEO_Flush();
    int got = 0;
    for (int i = 0; i < 4; i++) got += wait_retrace(200);
    logf_("  returned to safe mode (%d/4 retraces seen)", got);
}

// ======================================================================= measurement
static bool wait_cross(u32 t, u32 ms, u32 *ml)
{
    u64 to = gettime() + millisecs_to_ticks(ms); u32 l;
    while ((l = dpv()) >= t) { if (l > *ml) *ml = l; if (gettime() > to) return false; }
    while ((l = dpv()) < t)  { if (l > *ml) *ml = l; if (gettime() > to) return false; }
    return true;
}

static Meas measure(void)
{
    Meas m; memset(&m, 0, sizeof(m));
    if (wait_retrace(300)) {
        u32 r1 = retraces; u64 t1 = gettime(); u64 win = t1 + millisecs_to_ticks(1000);
        while (gettime() < win) {}
        u32 n = retraces - r1;
        u64 us = ticks_to_microsecs(gettime() - t1);
        if (n && us) m.field_mhz = (u32)((u64)n * 1000000000ull / us);
    }
    u64 best = ~0ull; u32 ml = 0;
    for (int k = 0; k < 6; k++) {
        u64 sto = gettime() + millisecs_to_ticks(60); u32 l;
        while ((l = dpv()) > 10) { if (l > ml) ml = l; if (gettime() > sto) break; }
        bool a = wait_cross(40, 60, &ml);  u64 ta = gettime();
        bool b = wait_cross(200, 60, &ml); u64 tb = gettime();
        if (a && b && tb > ta && tb - ta < best) best = tb - ta;
    }
    // sample the counter for a while longer to catch the true maximum
    u64 to = gettime() + millisecs_to_ticks(120);
    while (gettime() < to) { u32 l = dpv(); if (l > ml) ml = l; }
    m.maxline = ml;
    if (best != ~0ull) {
        u64 ns = ticks_to_nanosecs(best);
        if (ns) m.line_hz = (u32)(160ull * 1000000000ull / ns);
    }
    m.valid = (m.line_hz != 0);
    return m;
}

// ======================================================================= GX drawing
static float cl, cr, ct, cb;   // current tile clip rectangle, logical coordinates
static int   cur_field;        // -1 = frame pass, 0/1 = field pass

static void gx_init(void)
{
    GX_Init(gpfifo, sizeof(gpfifo));
    GX_SetCopyClear((GXColor){ 0, 0, 0, 0xFF }, GX_MAX_Z24);
    GX_SetPixelFmt(GX_PF_RGB8_Z24, GX_ZC_LINEAR);
    GX_SetCullMode(GX_CULL_NONE);
    GX_SetZMode(GX_FALSE, GX_ALWAYS, GX_FALSE);
    GX_SetColorUpdate(GX_TRUE);
    GX_SetAlphaUpdate(GX_FALSE);
    GX_SetDispCopyGamma(GX_GM_1_0);
    GX_SetCopyFilter(GX_FALSE, safe->sample_pattern, GX_FALSE, safe->vfilter); // no AA, no V filter
    GX_SetFieldMode(GX_FALSE, GX_FALSE);
    GX_SetDispCopyYScale(1.0f);

    GX_ClearVtxDesc();
    GX_SetVtxDesc(GX_VA_POS, GX_DIRECT);
    GX_SetVtxDesc(GX_VA_CLR0, GX_DIRECT);
    GX_SetVtxDesc(GX_VA_TEX0, GX_DIRECT);
    GX_SetVtxAttrFmt(GX_VTXFMT0, GX_VA_POS, GX_POS_XY, GX_F32, 0);
    GX_SetVtxAttrFmt(GX_VTXFMT0, GX_VA_CLR0, GX_CLR_RGBA, GX_RGBA8, 0);
    GX_SetVtxAttrFmt(GX_VTXFMT0, GX_VA_TEX0, GX_TEX_ST, GX_F32, 0);
    GX_SetNumChans(1);
    GX_SetChanCtrl(GX_COLOR0A0, GX_DISABLE, GX_SRC_REG, GX_SRC_VTX, GX_LIGHTNULL, GX_DF_NONE, GX_AF_NONE);
    GX_SetNumTexGens(1);
    GX_SetTexCoordGen(GX_TEXCOORD0, GX_TG_MTX2x4, GX_TG_TEX0, GX_IDENTITY);
    GX_SetNumTevStages(1);
    GX_SetTevOrder(GX_TEVSTAGE0, GX_TEXCOORD0, GX_TEXMAP0, GX_COLOR0A0);
    GX_SetTevOp(GX_TEVSTAGE0, GX_PASSCLR);
    GX_SetBlendMode(GX_BM_NONE, GX_BL_ONE, GX_BL_ZERO, GX_LO_COPY);
    Mtx mv; guMtxIdentity(mv);
    GX_LoadPosMtxImm(mv, GX_PNMTX0);
    GX_SetCurrentMtx(GX_PNMTX0);

    // 4x4 RGBA8 1px checkerboard texture (one tile: 32 bytes AR, then 32 bytes GB)
    for (int i = 0; i < 16; i++) {
        u8 val = (((i & 3) + (i >> 2)) & 1) ? 0xFF : 0x00;
        checker_tex[i * 2] = 0xFF; checker_tex[i * 2 + 1] = val;
        checker_tex[32 + i * 2] = val; checker_tex[32 + i * 2 + 1] = val;
    }
    DCFlushRange(checker_tex, sizeof(checker_tex));
    GX_InitTexObj(&checker_obj, checker_tex, 4, 4, GX_TF_RGBA8, GX_REPEAT, GX_REPEAT, GX_FALSE);
    GX_InitTexObjFilterMode(&checker_obj, GX_NEAR, GX_NEAR);
    GX_InvalidateTexAll();
    GX_LoadTexObj(&checker_obj, GX_TEXMAP0);
}

static inline bool vis(float x0, float y0, float x1, float y1)
{
    return x1 > cl && x0 < cr && y1 > ct && y0 < cb;
}

static void rect(float x0, float y0, float x1, float y1, u8 r, u8 g, u8 b)
{
    if (!vis(x0, y0, x1, y1)) return;
    if (x0 < cl) x0 = cl;
    if (x1 > cr) x1 = cr;
    if (y0 < ct) y0 = ct;
    if (y1 > cb) y1 = cb;
    GX_Begin(GX_QUADS, GX_VTXFMT0, 4);
    GX_Position2f32(x0, y0); GX_Color4u8(r, g, b, 0xFF); GX_TexCoord2f32(0, 0);
    GX_Position2f32(x1, y0); GX_Color4u8(r, g, b, 0xFF); GX_TexCoord2f32(0, 0);
    GX_Position2f32(x1, y1); GX_Color4u8(r, g, b, 0xFF); GX_TexCoord2f32(0, 0);
    GX_Position2f32(x0, y1); GX_Color4u8(r, g, b, 0xFF); GX_TexCoord2f32(0, 0);
    GX_End();
}

// Full-clip checkerboard via the repeating 1px texture (offsets keep texcoords small).
static void checker_rect(float x0, float y0, float x1, float y1)
{
    if (!vis(x0, y0, x1, y1)) return;
    if (x0 < cl) x0 = cl;
    if (x1 > cr) x1 = cr;
    if (y0 < ct) y0 = ct;
    if (y1 > cb) y1 = cb;
    float ox = (float)((int)(x0 / 4) * 4), oy = (float)((int)(y0 / 4) * 4);
    GX_SetTevOp(GX_TEVSTAGE0, GX_REPLACE);
    GX_Begin(GX_QUADS, GX_VTXFMT0, 4);
    GX_Position2f32(x0, y0); GX_Color4u8(255, 255, 255, 255); GX_TexCoord2f32((x0 - ox) / 4, (y0 - oy) / 4);
    GX_Position2f32(x1, y0); GX_Color4u8(255, 255, 255, 255); GX_TexCoord2f32((x1 - ox) / 4, (y0 - oy) / 4);
    GX_Position2f32(x1, y1); GX_Color4u8(255, 255, 255, 255); GX_TexCoord2f32((x1 - ox) / 4, (y1 - oy) / 4);
    GX_Position2f32(x0, y1); GX_Color4u8(255, 255, 255, 255); GX_TexCoord2f32((x0 - ox) / 4, (y1 - oy) / 4);
    GX_End();
    GX_SetTevOp(GX_TEVSTAGE0, GX_PASSCLR);
}

// Text with libogc's 8x16 font; each font pixel is sx by sy logical pixels.
static void text(float x, float y, int sx, int sy, const char *s, u8 r, u8 g, u8 b)
{
    for (; *s; s++, x += 8 * sx) {
        if (!vis(x, y, x + 8 * sx, y + 16 * sy)) continue;
        const u8 *gl = &console_font_8x16[(u8)*s * 16];
        for (int row = 0; row < 16; row++) {
            u8 bits = gl[row];
            int col = 0;
            while (col < 8) {
                if (!(bits & (0x80 >> col))) { col++; continue; }
                int run = col;
                while (run < 8 && (bits & (0x80 >> run))) run++;
                rect(x + col * sx, y + row * sy, x + run * sx, y + (row + 1) * sy, r, g, b);
                col = run;
            }
        }
    }
}

// Iterate the integer logical rows overlapping the clip that this pass actually rasterises
// (field passes only hit rows of their own parity).
#define FOR_ROWS(y, y0, y1) \
    for (int y = (cur_field >= 0 ? (((int)(y0) & ~1) + cur_field) : (int)(y0)); y < (int)(y1); y += (cur_field >= 0 ? 2 : 1))

// ======================================================================= patterns
#define NPATS 12
static const char *pat_name[NPATS] = {
    "Color bars + gray ramp", "Grid / geometry", "Luma stripes 1/2/4px (H res)",
    "Line pairs 1/2/4 (V res)", "Checkerboard 1px", "Chroma: magenta/green 1px",
    "Chroma: red/gray isoluma 1px", "Chroma vs luma wedge", "White", "50% gray", "Black",
    "Motion / refresh test",
};

static const Calc *rc;      // calc of the mode being rendered
static int pat = 0;
static bool show_info = true;
static Meas cur_meas;
static u32 motion_t = 0;

static void pat_bars(void)
{
    static const u8 col[7][3] = { {191,191,191},{191,191,0},{0,191,191},{0,191,0},{191,0,191},{191,0,0},{0,0,191} };
    float W = rc->logW, H = rc->frameH, top = H * 2 / 3;
    for (int i = 0; i < 7; i++) rect(W * i / 7, 0, W * (i + 1) / 7, top, col[i][0], col[i][1], col[i][2]);
    int x0 = cl < 0 ? 0 : (int)cl, x1 = cr > W ? (int)W : (int)cr + 1;
    for (int x = x0; x < x1; x++) { u8 v = (u8)(x * 255 / (rc->logW - 1)); rect(x, top, x + 1, H, v, v, v); }
}

static void pat_grid(void)
{
    float W = rc->logW, H = rc->frameH;
    rect(0, 0, W, H, 0, 0, 0);
    int gx = rc->logW / 16, gy = rc->frameH / 12;
    for (int i = 1; i < 16; i++) rect(i * gx, 0, i * gx + 1, H, 128, 128, 128);
    for (int j = 1; j < 12; j++) rect(0, j * gy, W, j * gy + 1, 128, 128, 128);
    rect(W / 2 - 1, 0, W / 2 + 1, H, 255, 255, 255);
    rect(0, H / 2 - 1, W, H / 2 + 1, 255, 255, 255);
    rect(0, 0, W, 1, 255, 255, 255); rect(0, H - 1, W, H, 255, 255, 255);
    rect(0, 0, 1, H, 255, 255, 255); rect(W - 1, 0, W, H, 255, 255, 255);
    float s = gy;
    rect(1, 1, 1 + s, 1 + s, 255, 0, 0);         rect(W - 1 - s, 1, W - 1, 1 + s, 0, 255, 0);
    rect(1, H - 1 - s, 1 + s, H - 1, 0, 0, 255); rect(W - 1 - s, H - 1 - s, W - 1, H - 1, 255, 255, 0);
}

static void stripes_x(float y0, float y1, int (*colfn)(int x), const u8 (*pal)[3])
{
    int x0 = cl < 0 ? 0 : (int)cl, x1 = cr > rc->logW ? rc->logW : (int)cr + 1;
    for (int x = x0; x < x1; x++) { int k = colfn(x); rect(x, y0, x + 1, y1, pal[k][0], pal[k][1], pal[k][2]); }
}
static int zone3(int x) { int w = rc->logW; int p = x < w / 3 ? 1 : (x < 2 * w / 3 ? 2 : 4); return (x / p) & 1; }
static int alt1(int x) { return x & 1; }
static const u8 pal_bw[2][3] = { {0,0,0}, {255,255,255} };
static const u8 pal_mg[2][3] = { {0,191,0}, {191,0,191} };
static const u8 pal_rg[2][3] = { {66,66,66}, {220,0,0} };   // equal luma (Y ~ 66)

static void pat_lines(void)
{
    float W = rc->logW; int H = rc->frameH;
    int y0 = ct < 0 ? 0 : (int)ct, y1 = cb > H ? H : (int)cb + 1;
    FOR_ROWS(y, y0, y1) {
        int p = y < H / 3 ? 1 : (y < 2 * H / 3 ? 2 : 4);
        u8 v = ((y / p) & 1) ? 255 : 0;
        rect(0, y, W, y + 1, v, v, v);
    }
}

static void pat_wedge(void)
{
    static const int period[8] = { 1, 2, 3, 4, 6, 8, 12, 16 };
    float H = rc->frameH, half = H / 2;
    int x0 = cl < 0 ? 0 : (int)cl, x1 = cr > rc->logW ? rc->logW : (int)cr + 1;
    for (int x = x0; x < x1; x++) {
        int z = x * 8 / rc->logW; if (z > 7) z = 7;
        int on = (x / period[z]) & 1;
        if (on) rect(x, 0, x + 1, half, 200, 0, 0); else rect(x, 0, x + 1, half, 0, 160, 160);
        if (on) rect(x, half, x + 1, H, 255, 255, 255); else rect(x, half, x + 1, H, 0, 0, 0);
    }
}

static int txs(void) { return rc->logW >= 1200 ? 2 : 1; }
static int tys(void) { return rc->frameH >= 700 ? 2 : 1; }

static void pat_motion(void)
{
    float W = rc->logW, H = rc->frameH;
    rect(0, 0, W, H, 0, 0, 0);
    float bw = W / 40 < 2 ? 2 : W / 40, bh = H / 30 < 2 ? 2 : H / 30;
    u32 t = motion_t;
    float x = (float)((t * (u32)(W / 120 < 1 ? 1 : W / 120)) % (u32)(W - bw));
    float y = (float)((t * (u32)(H / 90 < 1 ? 1 : H / 90)) % (u32)(H - bh));
    rect(x, 0, x + bw, H, 255, 255, 255);
    rect(0, y, W, y + bh, 255, 64, 64);
    char buf[48]; snprintf(buf, sizeof(buf), "field %lu", (unsigned long)t);
    int sx = txs() * 2, sy = tys() * 2;
    text(W / 2 - 8 * sx * 5, H / 2 - 16 * sy, sx, sy, buf, 255, 255, 0);
}

static void draw_info(void)
{
    if (!show_info) return;
    const Calc *c = rc;
    int sx = txs(), sy = tys();
    char l1[64], l2[64], l3[64], l4[64];
    snprintf(l1, sizeof(l1), "%s  [%s]", c->m->name, c->m->aspect);
    snprintf(l2, sizeof(l2), "P%d/%d %s  (D-pad, A=info)", pat + 1, NPATS, pat_name[pat]);
    snprintf(l3, sizeof(l3), "%ux%u%c  clk %s", c->A, c->frameH, c->il ? 'i' : 'p', clocks[c->m->clk].desc);
    if (cur_meas.valid)
        snprintf(l4, sizeof(l4), "meas %lu.%02lu kHz  %lu.%03lu Hz  DPV %lu",
                 (unsigned long)(cur_meas.line_hz / 1000), (unsigned long)(cur_meas.line_hz % 1000 / 10),
                 (unsigned long)(cur_meas.field_mhz / 1000), (unsigned long)(cur_meas.field_mhz % 1000),
                 (unsigned long)cur_meas.maxline);
    else
        snprintf(l4, sizeof(l4), "expect %lu.%02lu kHz  %lu.%03lu Hz (measuring)",
                 (unsigned long)(c->exp_line_hz / 1000), (unsigned long)(c->exp_line_hz % 1000 / 10),
                 (unsigned long)(c->exp_field_mhz / 1000), (unsigned long)(c->exp_field_mhz % 1000));
    const char *ls[4] = { l1, l2, l3, l4 };
    int maxlen = 0; for (int i = 0; i < 4; i++) { int n = strlen(ls[i]); if (n > maxlen) maxlen = n; }
    float x = 8 * sx, y = 8 * sy;
    rect(x - 4 * sx, y - 4 * sy, x + maxlen * 8 * sx + 4 * sx, y + 4 * 16 * sy + 4 * sy, 0, 0, 0);
    for (int i = 0; i < 4; i++) text(x, y + i * 16 * sy, sx, sy, ls[i], 255, 255, 255);
}

static void draw_pattern(void)
{
    float W = rc->logW, H = rc->frameH;
    switch (pat) {
    case 0: pat_bars(); break;
    case 1: pat_grid(); break;
    case 2: stripes_x(0, H, zone3, pal_bw); break;
    case 3: pat_lines(); break;
    case 4: checker_rect(0, 0, W, H); break;
    case 5: stripes_x(0, H, alt1, pal_mg); break;
    case 6: stripes_x(0, H, alt1, pal_rg); break;
    case 7: pat_wedge(); break;
    case 8: rect(0, 0, W, H, 255, 255, 255); break;
    case 9: rect(0, 0, W, H, 128, 128, 128); break;
    case 10: rect(0, 0, W, H, 0, 0, 0); break;
    case 11: pat_motion(); break;
    }
    draw_info();
}

// Split [0,total) into n near-equal chunks, each a multiple of 'align' (last takes the rest).
static int split(int total, int maxc, int align, int *off, int *len)
{
    int n = (total + maxc - 1) / maxc;
    int base = (total + n - 1) / n;
    base = (base + align - 1) / align * align;
    int pos = 0, k = 0;
    while (pos < total) { off[k] = pos; len[k] = (total - pos < base) ? total - pos : base; pos += len[k]; k++; }
    return k;
}

// Render one pass (field = -1 whole frame, or 0/1 one field) into fb.
// GPU completion with a timeout: GX_DrawDone() waits forever if the GPU stalls, which
// would look exactly like a console lockup. This polls a draw-done flag instead.
static volatile bool gx_done;
static void on_drawdone(void) { gx_done = true; }
static u32 gpu_timeouts;
static bool gpu_wait(const char *what)
{
    u64 to = gettime() + millisecs_to_ticks(500);
    while (!gx_done && gettime() < to) heartbeat();
    if (gx_done) return true;
    gpu_timeouts++;
    logf_("  GPU timeout (%s), #%lu: aborting frame", what, (unsigned long)gpu_timeouts);
    GX_AbortFrame();
    return false;
}
static void gpu_finish(const char *what)
{
    gx_done = false;
    GX_SetDrawDone();
    gpu_wait(what);
}

static void render_pass(void *fb, int field)
{
    const Calc *c = rc;
    int xo[8], xl[8], yo[8], yl[8];
    int nx = split(c->A, 640, 16, xo, xl);
    int rows = (field >= 0) ? c->frameH / 2 : c->frameH;
    int ny = split(rows, 528, 2, yo, yl);
    u32 stride = (field >= 0) ? 2u * c->A : c->A;
    cur_field = field;
    for (int j = 0; j < ny; j++) {
        for (int i = 0; i < nx; i++) {
            int tw = xl[i], th = yl[j];
            GX_SetViewport(0, 0, tw, th, 0, 1);
            GX_SetScissor(0, 0, tw, th);
            cl = (float)xo[i] / c->sx;
            cr = (float)(xo[i] + tw) / c->sx;
            if (field >= 0) { ct = 2.0f * yo[j] + field - 0.5f; cb = ct + 2.0f * th; }
            else { ct = yo[j]; cb = yo[j] + th; }
            Mtx44 p;
            guOrtho(p, ct, cb, cl, cr, 0, 300);
            GX_LoadProjectionMtx(p, GX_ORTHOGRAPHIC);
            draw_pattern();
            GX_SetDispCopySrc(0, 0, tw, th);
            GX_SetDispCopyDst(stride, th);
            u32 row = (field >= 0) ? 2u * yo[j] + field : (u32)yo[j];
            GX_CopyDisp((u8 *)fb + (row * c->A + xo[i]) * 2, GX_TRUE);
        }
    }
    gpu_finish(field >= 0 ? "field pass" : "frame");
}

static void render_frame(void *fb)
{
    if (rc->m->field) { render_pass(fb, 0); render_pass(fb, 1); }
    else render_pass(fb, -1);
}

static void clear_fb(void *fb, u32 bytes)
{
    u32 *p = (u32 *)fb;                                      // uncached (K1) pointer
    for (u32 i = 0; i < bytes / 4; i++) p[i] = 0x10801080;   // YUV black
}

// ======================================================================= input
#define BTN_NEXT  1
#define BTN_PREV  2
#define BTN_A     4
#define BTN_BACK  8
#define BTN_UP   16
#define BTN_DOWN 32
#define BTN_LEFT 64
#define BTN_RIGHT 128
#define BTN_HOME 256

static u32 read_buttons(void)
{
    WPAD_ScanPads(); PAD_ScanPads();
    u32 w = WPAD_ButtonsDown(0); u32 g = PAD_ButtonsDown(0);
    u32 b = 0;
    if ((w & WPAD_BUTTON_UP) || (g & PAD_BUTTON_UP)) b |= BTN_UP;
    if ((w & WPAD_BUTTON_DOWN) || (g & PAD_BUTTON_DOWN)) b |= BTN_DOWN;
    if ((w & WPAD_BUTTON_LEFT) || (g & PAD_BUTTON_LEFT)) b |= BTN_LEFT;
    if ((w & WPAD_BUTTON_RIGHT) || (g & PAD_BUTTON_RIGHT)) b |= BTN_RIGHT;
    if ((w & WPAD_BUTTON_A) || (g & PAD_BUTTON_A)) b |= BTN_A;
    if ((g & PAD_BUTTON_START) || (w & WPAD_BUTTON_PLUS)) b |= BTN_BACK;
    if (w & WPAD_BUTTON_HOME) b |= BTN_BACK | BTN_HOME;
    if (b & (BTN_RIGHT | BTN_DOWN)) b |= BTN_NEXT;
    if (b & (BTN_LEFT | BTN_UP)) b |= BTN_PREV;
    return b;
}

// ======================================================================= test mode
static void run_mode(int idx)
{
    static Calc c;
    calc_mode(&modes[idx], &c);
    rc = &c;
    memset(&cur_meas, 0, sizeof(cur_meas));
    motion_t = 0;
    int front = 0, back = 1;

    logf_("");
    logf_("=== %s [%s]  %ux%u%c  %s  expect line %lu Hz field %lu mHz",
          c.m->name, c.m->aspect, c.A, c.frameH, c.il ? 'i' : 'p', clocks[c.m->clk].desc,
          (unsigned long)c.exp_line_hz, (unsigned long)c.exp_field_mhz);

    clear_fb(xfb[0], c.xfb_bytes);
    clear_fb(xfb[1], c.xfb_bytes);
    render_frame(xfb[front]);
    program_vi(&c, xfb[front]);
    for (int i = 0; i < 4; i++) wait_retrace(150);

    cur_meas = measure();
    last_meas[idx] = cur_meas;
    logf_("  measured: line %lu Hz  field %lu mHz  DPV max %lu",
          (unsigned long)cur_meas.line_hz, (unsigned long)cur_meas.field_mhz, (unsigned long)cur_meas.maxline);
    render_frame(xfb[back]);
    { u32 l; _CPU_ISR_Disable(l); set_fbb(&c, xfb[back]); _CPU_ISR_Restore(l); }
    front ^= 1; back ^= 1;

    while (1) {
        u32 b = read_buttons();
        if (b & BTN_BACK) { logf_("  back pressed"); break; }
        bool redraw = false;
        if (b & BTN_NEXT) { pat = (pat + 1) % NPATS; redraw = true; }
        if (b & BTN_PREV) { pat = (pat + NPATS - 1) % NPATS; redraw = true; }
        if (b & (BTN_NEXT | BTN_PREV)) logf_("  pattern %d", pat + 1);
        if (b & BTN_A)    { show_info = !show_info; redraw = true; }

        if (pat == 11) {
            // Motion: one new image per VI field. Field-rendered modes draw only the field
            // that comes next (its own rows of the interleaved buffer); the other rows of
            // the back buffer are never scanned during that field.
            motion_t++;
            if (c.m->field) {
                int cur = dpv() >= (u32)(c.N / 2 + 1) ? 1 : 0;   // field being scanned now
                render_pass(xfb[back], cur ^ 1);
            } else {
                render_pass(xfb[back], -1);
            }
            wait_retrace(100);
            u32 l; _CPU_ISR_Disable(l); set_fbb(&c, xfb[back]); _CPU_ISR_Restore(l);
            front ^= 1; back ^= 1;
        } else if (redraw) {
            // static pattern: draw the whole frame off-screen, then flip at a retrace
            render_frame(xfb[back]);
            wait_retrace(100);
            u32 l; _CPU_ISR_Disable(l); set_fbb(&c, xfb[back]); _CPU_ISR_Restore(l);
            front ^= 1; back ^= 1;
        } else {
            wait_retrace(100);
        }
    }
    restore_safe();
}

// ======================================================================= menu
static int sel = 0;

static void menu_pos(int i, int *row, int *col)
{
    if (i >= NMODES) { *row = 20; *col = 2; return; }
    if (i == 24) { *row = 19; *col = 41; return; }   // 5th entry under 720p / 720i
    int column = i / 12, j = i % 12, g = j / 4, k = j % 4;
    *row = 4 + g * 5 + 1 + k;
    *col = column ? 41 : 2;
}

static void menu_item(int i, bool hilite)
{
    int row, col; menu_pos(i, &row, &col);
    printf("\x1b[%d;%dH", row, col);
    if (hilite) printf("\x1b[30;42m");
    if (i >= NMODES) printf("%c Exit to loader", hilite ? '>' : ' ');
    else printf("%c %-28s", hilite ? '>' : ' ', modes[i].name);
    printf("\x1b[37;40m");
}

static void menu_details(void)
{
    printf("\x1b[22;2H\x1b[K");
    printf("\x1b[23;2H\x1b[K");
    if (sel >= NMODES) { printf("\x1b[22;2HReturn to the Homebrew Channel."); return; }
    Calc c; calc_mode(&modes[sel], &c);
    printf("\x1b[22;2H%ux%u%c %s  %s  line %lu.%02lu kHz  %lu.%02lu Hz", c.A, c.frameH, c.il ? 'i' : 'p',
           modes[sel].aspect, clocks[modes[sel].clk].desc,
           (unsigned long)(c.exp_line_hz / 1000), (unsigned long)(c.exp_line_hz % 1000 / 10),
           (unsigned long)(c.exp_field_mhz / 1000), (unsigned long)(c.exp_field_mhz % 1000 / 10));
    if (last_meas[sel].valid)
        printf("\x1b[23;2Hlast measured: line %lu.%02lu kHz  field %lu.%02lu Hz  DPV %lu",
               (unsigned long)(last_meas[sel].line_hz / 1000), (unsigned long)(last_meas[sel].line_hz % 1000 / 10),
               (unsigned long)(last_meas[sel].field_mhz / 1000), (unsigned long)(last_meas[sel].field_mhz % 1000 / 10),
               (unsigned long)last_meas[sel].maxline);
}

static void menu_draw(void)
{
    VIDEO_ClearFrameBuffer(safe, menu_xfb, COLOR_BLACK);
    CON_Init(menu_xfb, 20, 20, safe->fbWidth, safe->xfbHeight, safe->fbWidth * VI_DISPLAY_PIX_SZ);
    printf("\x1b[2J\x1b[37;40m");
    printf("\x1b[1;2HVI Mode Test  -  Wii-VI-Experiments");
    printf("\x1b[2;2HA: start test   D-pad: move   HOME: quit");
    printf("\x1b[25;2HIn a test: D-pad = pattern, A = info on/off,");
    printf("\x1b[26;2HSTART (GC) / HOME or + (Wii Remote) = back here.");
    for (int g = 0; g < 6; g++) {
        int row = 4 + (g % 3) * 5, col = g < 3 ? 2 : 41;
        printf("\x1b[%d;%dH\x1b[33m-- %s --\x1b[37m", row, col, group_name[g]);
    }
    for (int i = 0; i <= NMODES; i++) menu_item(i, i == sel);
    menu_details();
}

static void menu_move(int to)
{
    if (to < 0) to = 0;
    if (to > NMODES) to = NMODES;
    if (to == sel) return;
    menu_item(sel, false);
    sel = to;
    menu_item(sel, true);
    menu_details();
}

// ======================================================================= main
int main(void)
{
    VIDEO_Init();
    WPAD_Init();
    PAD_Init();
    VIDEO_SetPostRetraceCallback(on_retrace);

    boot1c4 = HW(0x1C4) & 0x7FFFFFFFu;   // bit 31 is a lock-status bit set by hardware
    boot1c8 = HW(0x1C8);

    // Big framebuffers first, so they land low in MEM1.
    xfb[0] = MEM_K0_TO_K1(memalign(32, xfb_max));
    xfb[1] = MEM_K0_TO_K1(memalign(32, xfb_max));
    menu_xfb = MEM_K0_TO_K1(memalign(32, 720 * 576 * 2));

    safe = VIDEO_GetPreferredMode(NULL);
    VIDEO_Configure(safe);
    VIDEO_ClearFrameBuffer(safe, menu_xfb, COLOR_BLACK);
    VIDEO_SetNextFramebuffer(menu_xfb);
    VIDEO_SetBlack(false);
    VIDEO_Flush();
    wait_retrace(200);
    if (safe->viTVMode & VI_NON_INTERLACE) wait_retrace(200);

    fat_ok = fatInitDefault();
    logf_("");
    logf_("################ session start: vimodetest build " __DATE__ " " __TIME__);
    logf_("boot 0x1C4=%08X 0x1C8=%08X  safe viTVMode=%u %ux%u  xfb0=%08X xfb1=%08X",
          (unsigned)boot1c4, (unsigned)boot1c8, safe->viTVMode, safe->fbWidth, safe->xfbHeight,
          (unsigned)MEM_VIRTUAL_TO_PHYSICAL(xfb[0]), (unsigned)MEM_VIRTUAL_TO_PHYSICAL(xfb[1]));

    gx_init();
    GX_SetDrawDoneCallback(on_drawdone);
    // clear the EFB once (copy-with-clear into a scratch area)
    GX_SetViewport(0, 0, 640, 528, 0, 1);
    GX_SetDispCopySrc(0, 0, 640, 528);
    GX_SetDispCopyDst(640, 528);
    GX_CopyDisp(xfb[1], GX_TRUE);
    gpu_finish("init");

    menu_draw();
    while (1) {
        wait_retrace(100);         // bounded: never hangs if VI interrupts stop
        u32 b = read_buttons();
        if (b & BTN_HOME) break;
        if (b & BTN_UP) menu_move(sel - 1);
        if (b & BTN_DOWN) menu_move(sel + 1);
        if (b & BTN_LEFT) menu_move(sel >= 12 && sel < 24 ? sel - 12 : sel);
        if (b & BTN_RIGHT) menu_move(sel < 12 ? sel + 12 : sel == NMODES ? 24 : sel);
        if (b & BTN_A) {
            if (sel >= NMODES) break;
            run_mode(sel);
            menu_draw();
        }
    }

    logf_("exit to loader");
    u32 l; _CPU_ISR_Disable(l); HW(0x1C4) = boot1c4; HW(0x1C8) = boot1c8; _CPU_ISR_Restore(l);
    VIDEO_Configure(safe);
    VIDEO_SetBlack(true);
    VIDEO_Flush();
    wait_retrace(200);
    GPIOB_OUT &= ~0x20;            // slot LED off
    return 0;
}
