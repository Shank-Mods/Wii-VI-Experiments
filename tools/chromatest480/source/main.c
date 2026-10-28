// chromatest480: 480p-only chroma-subsampling torture test with a brute-force "double
// horizontal" 480p mode that needs a reprogrammed VI clock.
//
// The Wii external framebuffer / digital video bus is YUV 4:2:2: every two horizontal
// pixels share one Cb/Cr sample, so fine colour detail is lost. The "brute force" idea:
// render each source pixel TWICE horizontally so that each source pixel occupies a whole
// 4:2:2 pair on the bus -- its colour is then carried intact and an FPGA (PixelFX, etc.)
// can pair the duplicates back into full 4:4:4.
//
// This build has exactly two modes (all 240p/480i modes removed):
//   0  480P STD        standard 480p, ~720 wide on the bus (stock 54 MHz-class clock)
//   1  480P X2 (1440)  480p internally, each pixel doubled to 1440 wide on the bus
//
// 480P X2 needs ~108 MHz of VI pixel clock -- double the stock 54 MHz. We set it with the
// VI PLL discovery from this repo:
//       F = 54 MHz * Nfb / V,   Nfb = (0x1C4>>8 & 0xFF)+1,   V = (0x1C8>>20 & 0xF)
//       108 MHz = 54 * 14/7  ->  0x1C4 = 0x00200DC0 (Nfb=14),  0x1C8 = 0xD870700E (V=7)
// Horizontal: set viWidth=1440 (libogc then frames a 1440-wide active window) and double
// HLW 429->858 so the line is long enough; 480p vertical timing is kept, giving 59.94 Hz.
// The PLL is restored to its boot value when leaving X2 and on exit; it also resets on reboot.
//
// Controls:  D-pad Left/Right = mode,  Up/Down = pattern,  HOME / START = exit.
// Requires <ahb_access/> (meta.xml) for the PLL writes.

#include <gccore.h>
#include <fat.h>
#include <wiiuse/wpad.h>
#include <ogc/pad.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <malloc.h>
#include <ogc/machine/processor.h>
#include <ogc/lwp_watchdog.h>

#define VI ((volatile u16 *)0xCC002000)
#define HW(off) (*(volatile u32 *)(0xCD800000 + (off)))

#define MAX_XFB_W 1440
#define MAX_XFB_H 480
#define EFB_MAX_TILE 480           // physical pixels per EFB tile (<=640 EFB limit)

// 108 MHz VI clock (Nfb=14, V=7). byte2 of 0x1C4 stays 0x20 (0x21 hard-hangs).
#define PLL_1C4_108 0x00200DC0u
#define PLL_1C8_108 0xD870700Eu

static void *xfb;
static GXRModeObj rmode;
static u8 gpfifo[256 * 1024] __attribute__((aligned(32)));

// ---- measurement + logging (so we capture the Wii's real timing even when GCVideo
//      cannot display it, e.g. the 108 MHz X2 mode may not reach the monitor) ----
static volatile u32 retraces;
static void on_retrace(u32 c) { (void)c; retraces++; }
static void tb_delay_ms(u32 ms) { u64 to = gettime() + millisecs_to_ticks(ms); while (gettime() < to) {} }
static u32 cur_line(void) { return VI[0x2C / 2] & 0x7FF; }

static void logf_(const char *fmt, ...)
{
    static const char *const paths[] = { "usb:/chroma480.txt", "sd:/chroma480.txt" };
    static bool first = true;
    char line[256]; va_list ap; va_start(ap, fmt); vsnprintf(line, sizeof(line), fmt, ap); va_end(ap);
    for (int i = 0; i < 2; i++) { FILE *f = fopen(paths[i], first ? "w" : "a");
        if (f) { fputs(line, f); fputc('\n', f); fflush(f); fclose(f); } }
    first = false;
}

static bool wait_cross(u32 t, u64 tk, u32 *ml)
{
    u64 to = gettime() + tk; u32 l;
    while ((l = cur_line()) >= t) { if (l > *ml) *ml = l; if (gettime() > to) return false; }
    while ((l = cur_line()) <  t) { if (l > *ml) *ml = l; if (gettime() > to) return false; }
    return true;
}

// Measure line rate / field rate / total lines against the CPU timebase and log them.
static void measure_log(const char *tag)
{
    u32 hlw = VI[0x06 / 2] & 0x3FF;
    // field rate (bounded)
    u32 r0 = retraces; u64 fto = gettime() + millisecs_to_ticks(400);
    while (retraces == r0 && gettime() < fto) {}
    u32 field_milli = 0; bool field_ok = false;
    if (retraces != r0) {
        u32 r1 = retraces; u64 t1 = gettime(); u64 win = gettime() + millisecs_to_ticks(1500);
        while ((retraces - r1) < 90 && gettime() < win) {}
        u64 t2 = gettime(); u32 g = retraces - r1;
        if (g >= 2) { u64 us = ticks_to_microsecs(t2 - t1);
            if (us) { field_ok = true; field_milli = (u32)((u64)g * 1000000000ull / us); } }
    }
    // line period (best of 6, bounded)
    u64 best = ~0ull; u32 maxline = 0;
    for (int k = 0; k < 6; k++) {
        u32 ml = maxline; u64 sto = gettime() + millisecs_to_ticks(40); u32 l;
        while ((l = cur_line()) > 10) { if (l > ml) ml = l; if (gettime() > sto) break; }
        bool a = wait_cross(40, millisecs_to_ticks(40), &ml); u64 ta = gettime();
        bool b = wait_cross(200, millisecs_to_ticks(40), &ml); u64 tb = gettime();
        maxline = ml; if (a && b && tb > ta && (tb - ta) < best) best = tb - ta;
    }
    if (best != ~0ull) {
        u32 line_ns = (u32)(ticks_to_nanosecs(best) / 160);
        u32 eff = line_ns ? (u32)(4ull * hlw * 1000000ull / line_ns) : 0; // = 2*(2*hlw/line)
        u32 lk = line_ns ? (u32)(1000000000ull / line_ns) : 0;
        logf_("[%s] line=%u ns (~%u.%02u kHz) hlw=%u eff~%u kHz field=%s%u.%03u Hz maxline=%u lines=%u 0x1C4=%08X 0x1C8=%08X",
              tag, line_ns, lk / 1000, (lk % 1000) / 10, hlw, eff,
              field_ok ? "" : "NO/", field_milli / 1000, field_milli % 1000,
              maxline, (maxline + 1) / 2, (unsigned)HW(0x1C4), (unsigned)HW(0x1C8));
    } else {
        logf_("[%s] NO LINE LOCK (beam counter stalled) maxline=%u field=%u.%03u Hz 0x1C4=%08X 0x1C8=%08X",
              tag, maxline, field_milli / 1000, field_milli % 1000, (unsigned)HW(0x1C4), (unsigned)HW(0x1C8));
    }
}

static int mode_idx = 0;
static int pat_idx = 0;
#define NUM_MODES 2
#define NUM_PATS 6

static bool have_component;
static u32 pll_boot_1c4, pll_boot_1c8;     // snapshot to restore
static bool cur_doubled;
static int  cur_logw;                      // logical source width (640 STD, 720 X2)
static const char *mode_label[NUM_MODES] = { "480P STD", "480P X2" };

// ------------------------------------------------------------------ tile layout
// Each mode is drawn as 1..3 EFB tiles (<=640 wide) copied side by side into the XFB.
typedef struct { int ntiles; int base[3]; int w[3]; int xfbW; bool doubled; int logw; } Layout;

static Layout layout_for(int idx)
{
    if (idx == 1) // 480P X2: 720 logical doubled -> 1440 physical, 3 tiles of 480
        return (Layout){ 3, {0, 480, 960}, {480, 480, 480}, 1440, true, 720 };
    // 480P STD: 640 physical in one tile (libogc scales 640->~720 on the bus)
    return (Layout){ 1, {0, 0, 0}, {640, 0, 0}, 640, false, 640 };
}

// ------------------------------------------------------------------ PLL / clock
static void pll_restore(void)
{
    u32 l; _CPU_ISR_Disable(l);
    HW(0x1C4) = pll_boot_1c4;
    HW(0x1C8) = pll_boot_1c8;
    _CPU_ISR_Restore(l);
}

// ------------------------------------------------------------------ video modes
static void apply_mode(int idx)
{
    Layout lay = layout_for(idx);
    cur_doubled = lay.doubled;
    cur_logw = lay.logw;

    VIDEO_SetBlack(true);

    if (idx == 0) {
        // 480P STD: stock progressive mode (component/digital). PLL back to boot (54).
        memcpy(&rmode, have_component ? &TVNtsc480Prog : &TVNtsc480IntDf, sizeof(rmode));
        pll_restore();
        VIDEO_Configure(&rmode);
    } else {
        // 480P X2 1440: start from 480p, widen to 1440, double HLW, clock to 108 MHz.
        memcpy(&rmode, &TVNtsc480Prog, sizeof(rmode));
        rmode.fbWidth   = MAX_XFB_W;   // 1440 -> PICCONF fetch/stride, scaler off
        rmode.viWidth   = MAX_XFB_W;   // libogc frames a 1440-wide active window
        rmode.xfbHeight = 480;
        rmode.viHeight  = 480;
        rmode.efbHeight = 480;
        VIDEO_Configure(&rmode);       // native 480p timing + 1440 active + scaler off
        u32 l; _CPU_ISR_Disable(l);
        VI[0x06 / 2] = 0x035A;         // HLW 429 -> 858 (full line 1716 units)
        VI[0x6C / 2] = 0x0001;         // VICLK: undivided (= PLL output)
        HW(0x1C4) = PLL_1C4_108;       // Nfb = 14
        HW(0x1C8) = PLL_1C8_108;       // V  = 7   -> 108 MHz
        _CPU_ISR_Restore(l);
    }

    // clear whole buffer to black (YUV 0x10801080)
    u32 *p = (u32 *)xfb;
    for (int i = 0; i < MAX_XFB_W * MAX_XFB_H / 2; i++) p[i] = 0x10801080;

    VIDEO_SetNextFramebuffer(xfb);
    VIDEO_Flush();
    VIDEO_WaitVSync();
    VIDEO_WaitVSync();
    VIDEO_SetBlack(false);
    VIDEO_Flush();

    // Capture the Wii's real timing for this mode (independent of whether GCVideo/the
    // monitor can display it). ~1-2 s pause on each mode switch.
    tb_delay_ms(200);
    measure_log(mode_label[idx]);
}

// ------------------------------------------------------------------ GX helpers
static void gx_init(void)
{
    GXColor bg = { 0, 0, 0, 0xff };
    GX_Init(gpfifo, sizeof(gpfifo));
    GX_SetCopyClear(bg, GX_MAX_Z24);
    GX_SetCullMode(GX_CULL_NONE);
    GX_SetDispCopyGamma(GX_GM_1_0);
    GX_SetColorUpdate(GX_TRUE);
    GX_SetAlphaUpdate(GX_TRUE);
    GX_SetZMode(GX_FALSE, GX_ALWAYS, GX_FALSE);
    GX_ClearVtxDesc();
    GX_SetVtxDesc(GX_VA_POS, GX_DIRECT);
    GX_SetVtxDesc(GX_VA_CLR0, GX_DIRECT);
    GX_SetVtxAttrFmt(GX_VTXFMT0, GX_VA_POS, GX_POS_XY, GX_F32, 0);
    GX_SetVtxAttrFmt(GX_VTXFMT0, GX_VA_CLR0, GX_CLR_RGBA, GX_RGBA8, 0);
    GX_SetNumChans(1);
    GX_SetNumTexGens(0);
    GX_SetTevOp(GX_TEVSTAGE0, GX_PASSCLR);
    GX_SetTevOrder(GX_TEVSTAGE0, GX_TEXCOORDNULL, GX_TEXMAP_NULL, GX_COLOR0A0);
    GX_SetBlendMode(GX_BM_NONE, GX_BL_ONE, GX_BL_ZERO, GX_LO_COPY);
}

static void gx_begin_tile(int tileW, int efbHeight)
{
    Mtx44 ortho; Mtx mv;
    GX_SetViewport(0, 0, tileW, efbHeight, 0, 1);
    GX_SetScissor(0, 0, tileW, efbHeight);
    guOrtho(ortho, 0, efbHeight, 0, tileW, 0, 300);
    GX_LoadProjectionMtx(ortho, GX_ORTHOGRAPHIC);
    guMtxIdentity(mv);
    GX_LoadPosMtxImm(mv, GX_PNMTX0);
}

static inline void bar(float x, float y, float w, float h, u8 r, u8 g, u8 b)
{
    GX_Begin(GX_QUADS, GX_VTXFMT0, 4);
    GX_Position2f32(x, y);         GX_Color4u8(r, g, b, 0xff);
    GX_Position2f32(x + w, y);     GX_Color4u8(r, g, b, 0xff);
    GX_Position2f32(x + w, y + h); GX_Color4u8(r, g, b, 0xff);
    GX_Position2f32(x, y + h);     GX_Color4u8(r, g, b, 0xff);
    GX_End();
}

// ------------------------------------------------------------------ tiny font
static const u8 glyph_digit[10][8] = {
    {0x3C,0x66,0x6E,0x76,0x66,0x66,0x3C,0},{0x18,0x38,0x18,0x18,0x18,0x18,0x7E,0},
    {0x3C,0x66,0x06,0x0C,0x30,0x60,0x7E,0},{0x3C,0x66,0x06,0x1C,0x06,0x66,0x3C,0},
    {0x0C,0x1C,0x3C,0x6C,0x7E,0x0C,0x0C,0},{0x7E,0x60,0x7C,0x06,0x06,0x66,0x3C,0},
    {0x1C,0x30,0x60,0x7C,0x66,0x66,0x3C,0},{0x7E,0x06,0x0C,0x18,0x30,0x30,0x30,0},
    {0x3C,0x66,0x66,0x3C,0x66,0x66,0x3C,0},{0x3C,0x66,0x66,0x3E,0x06,0x0C,0x38,0},
};
// P I S T D X
static const u8 glyph_letter[6][8] = {
    {0x7C,0x66,0x66,0x7C,0x60,0x60,0x60,0},{0x7E,0x18,0x18,0x18,0x18,0x18,0x7E,0},
    {0x3C,0x66,0x60,0x3C,0x06,0x66,0x3C,0},{0x7E,0x18,0x18,0x18,0x18,0x18,0x18,0},
    {0x78,0x6C,0x66,0x66,0x66,0x6C,0x78,0},{0x66,0x66,0x3C,0x18,0x3C,0x66,0x66,0},
};
static void draw_glyph(const u8 g[8], float x, float y, float s)
{
    for (int row = 0; row < 8; row++)
        for (int col = 0; col < 8; col++)
            if (g[row] & (0x80 >> col)) bar(x + col * s, y + row * s, s, s, 255, 255, 255);
}
static void draw_text(const char *str, float x, float y, float s)
{
    for (const char *c = str; *c; c++) {
        if (*c >= '0' && *c <= '9') draw_glyph(glyph_digit[*c - '0'], x, y, s);
        else switch (*c) {
            case 'P': draw_glyph(glyph_letter[0], x, y, s); break;
            case 'I': draw_glyph(glyph_letter[1], x, y, s); break;
            case 'S': draw_glyph(glyph_letter[2], x, y, s); break;
            case 'T': draw_glyph(glyph_letter[3], x, y, s); break;
            case 'D': draw_glyph(glyph_letter[4], x, y, s); break;
            case 'X': draw_glyph(glyph_letter[5], x, y, s); break;
        }
        x += 9 * s;
    }
}

// ------------------------------------------------------------------ patterns
// Physical pixel (tileBase+ex) maps to logical source pixel lx; in doubled mode lx halves,
// so a 1px logical feature becomes 2 physical px = one whole 4:2:2 pair on the bus.
static inline int logical_x(int tileBase, int ex) { return cur_doubled ? (tileBase + ex) / 2 : (tileBase + ex); }

static void pat_stripes(int tileBase, int tileW, int H, u8 ar,u8 ag,u8 ab, u8 br,u8 bg,u8 bb)
{
    for (int ex = 0; ex < tileW; ex++) {
        int lx = logical_x(tileBase, ex);
        if (lx & 1) bar(ex, 0, 1, H, ar, ag, ab);
        else        bar(ex, 0, 1, H, br, bg, bb);
    }
}
static void pat_wedge(int tileBase, int tileW, int H)
{
    static const int period[8] = { 1, 2, 3, 4, 6, 8, 12, 16 };
    int half = H / 2;
    for (int ex = 0; ex < tileW; ex++) {
        int lx = logical_x(tileBase, ex);
        int zone = (lx * 8) / cur_logw; if (zone > 7) zone = 7;
        int on = (lx / period[zone]) & 1;
        if (on) bar(ex, 0, 1, half, 200, 0, 0);     else bar(ex, 0, 1, half, 0, 160, 160);
        if (on) bar(ex, half, 1, H - half, 255,255,255); else bar(ex, half, 1, H - half, 0,0,0);
    }
}
static void pat_reference(int tileBase, int tileW, int H)
{
    static const u8 cols[8][3] = {
        {255,255,255},{255,255,0},{0,255,255},{0,255,0},{255,0,255},{255,0,0},{0,0,255},{0,0,0} };
    int half = H / 2;
    for (int ex = 0; ex < tileW; ex++) {
        int lx = logical_x(tileBase, ex);
        int b = (lx * 8) / cur_logw; if (b > 7) b = 7;
        bar(ex, 0, 1, half, cols[b][0], cols[b][1], cols[b][2]);
        u8 v = (u8)(lx * 255 / (cur_logw - 1));
        bar(ex, half, 1, H - half, v, v, v);
    }
}
static void pat_edge(int tileBase, int tileW, int H)
{
    int mid = cur_logw / 2;
    for (int ex = 0; ex < tileW; ex++) {
        int lx = logical_x(tileBase, ex);
        if (lx < mid) bar(ex, 0, 1, H, 200, 0, 0);
        else          bar(ex, 0, 1, H, 0, 160, 160);
        // thin white/black marker lines straddling the edge
        int d = lx - mid;
        if (d >= -8 && d <= 8 && (d % 2 == 0))
            bar(ex, H / 4, cur_doubled ? 2 : 1, H / 2, (d & 2) ? 0 : 255, (d & 2) ? 0 : 255, (d & 2) ? 0 : 255);
    }
}
static const char *pat_name[NUM_PATS] __attribute__((unused)) = {
    "MAGENTA/GREEN 1PX","RED/BLUE 1PX","ISOLUMA RED/GRAY 1PX",
    "CHROMA VS LUMA WEDGE","COLOR + GRAY REF","SHARP CHROMA EDGE" };

static void draw_pattern(int tileBase, int tileW, int H)
{
    switch (pat_idx) {
        case 0: pat_stripes(tileBase, tileW, H, 0,200,0, 200,0,200); break; // green/magenta
        case 1: pat_stripes(tileBase, tileW, H, 0,0,220, 220,0,0); break;   // blue/red
        case 2: pat_stripes(tileBase, tileW, H, 76,76,76, 220,0,0); break;  // isoluma gray/red
        case 3: pat_wedge(tileBase, tileW, H); break;
        case 4: pat_reference(tileBase, tileW, H); break;
        case 5: pat_edge(tileBase, tileW, H); break;
    }
}

static void draw_status(void)
{
    char line2[4] = { 'P', (char)('0' + pat_idx), 0, 0 };
    bar(6, 6, 170, 42, 0, 0, 0);
    draw_text(mode_label[mode_idx], 10, 9, 2.0f);
    draw_text(line2, 10, 28, 2.0f);
}

// ------------------------------------------------------------------ render
static void render(void)
{
    Layout lay = layout_for(mode_idx);
    int H = rmode.efbHeight;

    GX_SetDispCopyYScale((f32)rmode.xfbHeight / (f32)H);
    GX_SetDispCopyDst(lay.xfbW, rmode.xfbHeight);
    GX_SetCopyFilter(rmode.aa, rmode.sample_pattern, GX_FALSE, rmode.vfilter);
    GX_SetFieldMode(rmode.field_rendering, (rmode.viHeight == 2 * rmode.xfbHeight) ? GX_ENABLE : GX_DISABLE);
    GX_SetPixelFmt(GX_PF_RGB8_Z24, GX_ZC_LINEAR);

    for (int t = 0; t < lay.ntiles; t++) {
        int tileBase = lay.base[t];
        int tileW = lay.w[t];
        gx_begin_tile(tileW, H);
        GX_SetCopyClear((GXColor){ 0, 0, 0, 0xff }, GX_MAX_Z24);
        draw_pattern(tileBase, tileW, H);
        if (t == 0) draw_status();
        GX_SetDispCopySrc(0, 0, tileW, H);
        GX_CopyDisp((u8 *)xfb + tileBase * 2, GX_TRUE);
    }
    GX_DrawDone();

    VIDEO_SetNextFramebuffer(xfb);
    VIDEO_Flush();
    VIDEO_WaitVSync();
}

int main(void)
{
    VIDEO_Init();
    WPAD_Init();
    PAD_Init();
    VIDEO_SetPostRetraceCallback(on_retrace);
    have_component = VIDEO_HaveComponentCable();

    pll_boot_1c4 = HW(0x1C4);
    pll_boot_1c8 = HW(0x1C8);

    bool fat = fatInitDefault();
    logf_("chromatest480 build " __DATE__ " " __TIME__ "  fat=%d component=%d boot 0x1C4=%08X 0x1C8=%08X",
          fat, have_component, (unsigned)pll_boot_1c4, (unsigned)pll_boot_1c8);

    xfb = MEM_K0_TO_K1(memalign(32, MAX_XFB_W * MAX_XFB_H * 2));
    apply_mode(mode_idx);
    gx_init();

    bool need_reapply = false;
    while (1) {
        WPAD_ScanPads(); PAD_ScanPads();
        u32 wd = WPAD_ButtonsDown(0), pd = PAD_ButtonsDown(0);
        if ((wd & WPAD_BUTTON_HOME) || (pd & PAD_BUTTON_START)) break;

        int old_mode = mode_idx;
        if ((wd & WPAD_BUTTON_RIGHT) || (pd & PAD_BUTTON_RIGHT)) mode_idx = (mode_idx + 1) % NUM_MODES;
        if ((wd & WPAD_BUTTON_LEFT)  || (pd & PAD_BUTTON_LEFT))  mode_idx = (mode_idx + NUM_MODES - 1) % NUM_MODES;
        if ((wd & WPAD_BUTTON_UP)    || (pd & PAD_BUTTON_UP))    pat_idx = (pat_idx + 1) % NUM_PATS;
        if ((wd & WPAD_BUTTON_DOWN)  || (pd & PAD_BUTTON_DOWN))  pat_idx = (pat_idx + NUM_PATS - 1) % NUM_PATS;
        if (mode_idx != old_mode) need_reapply = true;

        if (need_reapply) { apply_mode(mode_idx); need_reapply = false; }
        render();
    }

    // restore clock and a safe display before returning to the loader
    logf_("exit; restoring PLL to boot");
    pll_restore();
    GXRModeObj *safe = VIDEO_GetPreferredMode(NULL);
    VIDEO_Configure(safe);
    VIDEO_SetNextFramebuffer(xfb);
    VIDEO_SetBlack(true);
    VIDEO_Flush();
    VIDEO_WaitVSync();
    return 0;
}
