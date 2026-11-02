// vilinetest: can the Wii VI do more than 1023 lines per field?
//
// Part 1, readback: with the VI stopped, write all-ones then all-zeros to each VI register
//   and read back, giving the set of bits that actually exist in each register. Bits above
//   the documented fields (e.g. VTR bits 14-15 above the 10-bit active-line count, HLW bits
//   10-15) either stick or they don't.
//
// Part 2, counter tests: run real rasters with those extra bits set and measure what the VI
//   does. The beam counter (DPV, read as a full 16 bits) gives the line rate and the length
//   of each frame cycle against the CPU timebase, so a bit that is wired shows up as a
//   longer frame (or line) and one that isn't gives the same numbers as without it.
//
// Part 3, field merge: progressive mode scans two "fields" per frame cycle, each with its
//   own vsync (EQU) and blanking. If the vsync and blanking between them can be removed,
//   two 540-line fields become one 1080-line frame without needing a wider register. The
//   CPU follows the beam counter and switches EQU to 0 for the second field and back for
//   the first, with the per-field blanking (odd: before, even: after) set so the two
//   active regions touch. Measured by frame length, and shown on screen as an ordinary
//   480p made of two merged 240-line fields (top half from one framebuffer address, bottom
//   half from the other), which any 480p display can show.
//
// Everything here is timing only: bad values give a bad picture, never a hang (the PLL is
// changed for one test only, to 270 MHz, which viclkmax verified).

#include <gccore.h>
#include <fat.h>
#include <wiiuse/wpad.h>
#include <ogc/pad.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <malloc.h>
#include <math.h>
#include <ogc/machine/processor.h>
#include <ogc/lwp_watchdog.h>

#define VI        ((volatile u16 *)0xCC002000)
#define HW(o)     (*(volatile u32 *)(0xCD800000 + (o)))
#define GPIOB_OUT (*(volatile u32 *)0xCD0000C0)

static GXRModeObj *safe;
static void *menu_xfb;
static u32 *fb480, *fb1080;
static u32 boot1c4, boot1c8;
static volatile u32 retraces;
static bool fat_ok;

// ======================================================================= sign of life
static u32 hb_ms = 250;
static u64 hb_last;
static void heartbeat(void)
{
    u64 now = gettime();
    if (now - hb_last >= millisecs_to_ticks(hb_ms)) { GPIOB_OUT ^= 0x20; hb_last = now; }
}
static void delay_ms(u32 ms)
{
    u64 to = gettime() + millisecs_to_ticks(ms);
    while (gettime() < to) heartbeat();
}
static void on_retrace(u32 c) { (void)c; retraces++; }
static bool wait_retrace(u32 ms)
{
    u32 r = retraces; u64 to = gettime() + millisecs_to_ticks(ms);
    while (retraces == r && gettime() < to) heartbeat();
    return retraces != r;
}

// ======================================================================= log
static void logf_(const char *fmt, ...)
{
    if (!fat_ok) return;
    static const char *const paths[] = { "usb:/vilinetest.txt", "sd:/vilinetest.txt" };
    char line[400];
    va_list ap; va_start(ap, fmt); vsnprintf(line, sizeof(line), fmt, ap); va_end(ap);
    for (int i = 0; i < 2; i++) {
        FILE *f = fopen(paths[i], "a");
        if (f) { fputs(line, f); fputc('\n', f); fclose(f); }
    }
}

// ======================================================================= screen
static void scr_line(int row, const char *fmt, ...)
{
    char buf[160];
    va_list ap; va_start(ap, fmt); vsnprintf(buf, sizeof(buf), fmt, ap); va_end(ap);
    printf("\x1b[%d;2H\x1b[K%s\x1b[37m", row, buf);
}

// ======================================================================= VI register images
static const u8 img_regs[] = {
    0x00, 0x04, 0x06, 0x08, 0x0A, 0x0C, 0x0E, 0x10, 0x12, 0x14, 0x16, 0x18, 0x1A,
    0x1C, 0x1E, 0x20, 0x22, 0x24, 0x26, 0x28, 0x2A,
    0x30, 0x32, 0x34, 0x36, 0x38, 0x3A, 0x3C, 0x3E,
    0x48, 0x4A, 0x4C, 0x4E, 0x50, 0x52, 0x54, 0x56, 0x58, 0x5A, 0x5C, 0x5E, 0x60, 0x62,
    0x64, 0x66, 0x68, 0x6A, 0x6C, 0x70 };
#define NIMG (sizeof(img_regs))
typedef struct { u16 v[NIMG]; u16 dcr; } VIImage;
static VIImage safe_img;

static u16 img_mask(u8 off)
{
    if (off == 0x30 || off == 0x34 || off == 0x38 || off == 0x3C) return 0x7FFF;
    return 0xFFFF;
}
static void img_read(VIImage *im)
{
    for (u32 i = 0; i < NIMG; i++) im->v[i] = VI[img_regs[i] / 2] & img_mask(img_regs[i]);
    im->dcr = VI[0x02 / 2];
}
static void img_set(VIImage *im, u8 off, u16 v)
{
    for (u32 i = 0; i < NIMG; i++) if (img_regs[i] == off) { im->v[i] = v; return; }
}

static void vi_stop(void)
{
    u32 l; _CPU_ISR_Disable(l);
    VI[0x02 / 2] = VI[0x02 / 2] & ~0x0001;
    _CPU_ISR_Restore(l);
    delay_ms(2);
}

// VI off -> reset -> PLL -> relock -> full register image -> VI on.
static void apply_image(const VIImage *im, u32 p4, u32 p8)
{
    vi_stop();
    u32 l; _CPU_ISR_Disable(l);
    VI[0x02 / 2] = 0x0002;
    _CPU_ISR_Restore(l);
    delay_ms(1);
    _CPU_ISR_Disable(l);
    VI[0x02 / 2] = 0x0000;
    bool change = (HW(0x1C4) & 0x7FFFFFFFu) != p4 || HW(0x1C8) != p8;
    HW(0x1C4) = p4;
    HW(0x1C8) = p8;
    _CPU_ISR_Restore(l);
    if (change) delay_ms(50);
    _CPU_ISR_Disable(l);
    for (u32 i = 0; i < NIMG; i++) VI[img_regs[i] / 2] = im->v[i];
    VI[0x02 / 2] = im->dcr;
    _CPU_ISR_Restore(l);
}

static bool restore_safe(void)
{
    int got = 0;
    for (int attempt = 0; attempt < 2 && !got; attempt++) {
        apply_image(&safe_img, boot1c4, boot1c8);
        VIDEO_Configure(safe);
        VIDEO_SetNextFramebuffer(menu_xfb);
        VIDEO_SetBlack(false);
        VIDEO_Flush();
        for (int i = 0; i < 3; i++) got += wait_retrace(200);
        if (!got) logf_("  WARNING: no retrace after returning to safe mode (attempt %d)", attempt + 1);
    }
    return got > 0;
}

// ======================================================================= part 1: readback
typedef struct { u8 off; u16 doc; const char *name; } RegDoc;
// Documented implemented bits (libogc / YAGCD field layouts); 0 = no documented layout.
static const RegDoc regdocs[] = {
    { 0x00, 0x3FFF, "VTR   EQU 0-3, ACV 4-13" },
    { 0x04, 0x7F7F, "HTR0h HCE 0-6, HCS 8-14" },
    { 0x06, 0x03FF, "HTR0l HLW 0-9" },
    { 0x08, 0x07FF, "HTR1h HBE b9, HBS 1-10" },
    { 0x0A, 0xFFFF, "HTR1l HSY 0-6, HBE 7-15" },
    { 0x0C, 0x03FF, "VTO   PSB odd" },
    { 0x0E, 0x03FF, "VTO   PRB odd" },
    { 0x10, 0x03FF, "VTE   PSB even" },
    { 0x12, 0x03FF, "VTE   PRB even" },
    { 0x14, 0xFFFF, "BBEI" }, { 0x16, 0xFFFF, "BBEI" }, { 0x18, 0xFFFF, "BBOI" }, { 0x1A, 0xFFFF, "BBOI" },
    { 0x1C, 0, "TFBL hi" }, { 0x1E, 0, "TFBL lo" }, { 0x20, 0, "TFBR hi" }, { 0x22, 0, "TFBR lo" },
    { 0x24, 0, "BFBL hi" }, { 0x26, 0, "BFBL lo" }, { 0x28, 0, "BFBR hi" }, { 0x2A, 0, "BFBR lo" },
    { 0x30, 0, "DI0 hi" }, { 0x32, 0, "DI0 lo" }, { 0x34, 0, "DI1 hi" }, { 0x36, 0, "DI1 lo" },
    { 0x38, 0, "DI2 hi" }, { 0x3A, 0, "DI2 lo" }, { 0x3C, 0, "DI3 hi" }, { 0x3E, 0, "DI3 lo" },
    { 0x40, 0, "DL0 hi" }, { 0x42, 0, "DL0 lo" }, { 0x44, 0, "DL1 hi" }, { 0x46, 0, "DL1 lo" },
    { 0x48, 0, "PICCONF" }, { 0x4A, 0, "HSR" },
    { 0x4C, 0, "FCT0" }, { 0x4E, 0, "FCT0" }, { 0x50, 0, "FCT1" }, { 0x52, 0, "FCT1" },
    { 0x54, 0, "FCT2" }, { 0x56, 0, "FCT2" }, { 0x58, 0, "FCT3" }, { 0x5A, 0, "FCT3" },
    { 0x5C, 0, "FCT4" }, { 0x5E, 0, "FCT4" }, { 0x60, 0, "FCT5" }, { 0x62, 0, "FCT5" },
    { 0x64, 0, "FCT6" }, { 0x66, 0, "FCT6" }, { 0x68, 0, "?" }, { 0x6A, 0, "?" },
    { 0x6C, 0, "VICLK" }, { 0x70, 0, "HBE656/FBW?" }, { 0x72, 0, "?" }, { 0x74, 0, "?" }, { 0x76, 0, "?" },
};
#define NDOC (sizeof(regdocs) / sizeof(regdocs[0]))
static u16 bits1[NDOC], bits0[NDOC];

static int extra_bits_found;
static u16 vtr_bits, hlw_bits, prb_bits;

static void part1_readback(void)
{
    logf_("");
    logf_("=== Part 1: register bit readback (VI stopped; write FFFF then 0000, read each)");
    vi_stop();
    u32 l; _CPU_ISR_Disable(l);
    for (u32 i = 0; i < NDOC; i++) {
        volatile u16 *r = &VI[regdocs[i].off / 2];
        u16 save = *r;
        *r = 0xFFFF; bits1[i] = *r;
        *r = 0x0000; bits0[i] = *r;
        *r = save;
    }
    _CPU_ISR_Restore(l);
    restore_safe();
    extra_bits_found = 0;
    for (u32 i = 0; i < NDOC; i++) {
        const RegDoc *d = &regdocs[i];
        u16 extra = d->doc ? (bits1[i] & ~d->doc) : 0;
        if (extra) extra_bits_found++;
        logf_("  %02X %-26s wrote FFFF read %04X, wrote 0000 read %04X%s", d->off, d->name, bits1[i], bits0[i],
              extra ? "   <-- bits beyond the documented field" : "");
        if (d->off == 0x00) vtr_bits = bits1[i];
        if (d->off == 0x06) hlw_bits = bits1[i];
        if (d->off == 0x0E) prb_bits = bits1[i];
    }
}

// ======================================================================= part 2/3: rasters
typedef struct {
    const char *id, *name;
    bool fast;                 // 270 MHz (Nfb 20, V 4) instead of the stock 54 MHz
    bool hd;                   // 1920-wide / 2046-clock line and the 1080 framebuffer
    u16 hlw;                   // raw HTR0 low (0x06), may carry extra bits
    u16 vtr1, vtr2;            // VTR for field 1 / field 2 (vtr2 = 0: same as vtr1)
    u16 prbo, psbo, prbe, psbe;
    u16 bfbb_line;             // second field fetches from this framebuffer line (0 = same)
    u8  kind;                  // 0: compare frame-cycle lines, 1: compare line length
    u32 hyp[3]; const char *hypn[3];
} Test;

#define V(acv, equ) ((u16)(((acv) << 4) | (equ)))
static const Test tests[] = {
    { "B0", "control: 480p raster", 0, 0, 429, V(480, 6), 0, 66, 6, 66, 6, 0, 0,
      { 1050 }, { "as programmed" } },
    { "B1", "1000 active lines: beam counter > 2047?", 0, 0, 429, V(1000, 6), 0, 66, 6, 66, 6, 0, 0,
      { 2090, 2048 }, { "as programmed, counter > 11 bits", "11-bit counter overflow" } },
    { "B2", "1023 active lines (documented max)", 0, 0, 429, V(1023, 6), 0, 66, 6, 66, 6, 0, 0,
      { 2136, 2048 }, { "as programmed, counter > 11 bits", "11-bit counter overflow" } },
    { "B3", "VTR bit 14 set (ACV bit 10?)", 0, 0, 429, (u16)(V(100, 6) | 0x4000), 0, 66, 6, 66, 6, 0, 0,
      { 2338, 290, 2048 }, { "bit 14 IS active-line bit 10", "bit 14 ignored", "counter overflow" } },
    { "B4", "VTR bit 15 set (ACV bit 11?)", 0, 0, 429, (u16)(V(100, 6) | 0x8000), 0, 66, 6, 66, 6, 0, 0,
      { 4386, 290, 2048 }, { "bit 15 IS active-line bit 11", "bit 15 ignored", "counter overflow" } },
    { "B5", "PRB bit 10 set (pre-blank 1090?)", 0, 0, 429, V(480, 6), 0, 66 | 0x400, 6, 66 | 0x400, 6, 0, 0,
      { 2074, 1050 }, { "PRB has an 11th bit", "PRB bit 10 ignored" } },
    { "B6", "PSB bit 10 set (post-blank 1030?)", 0, 0, 429, V(480, 6), 0, 66, 6 | 0x400, 66, 6 | 0x400, 0, 0,
      { 2074, 1050 }, { "PSB has an 11th bit", "PSB bit 10 ignored" } },
    { "B7", "HLW bit 10 set (2906-clock line?)", 0, 0, 429 | 0x400, V(480, 6), 0, 66, 6, 66, 6, 0, 1,
      { 2906, 858 }, { "HLW has an 11th bit", "HLW bit 10 ignored" } },
    { "C0", "progressive uses odd AND even blanking?", 0, 0, 429, V(480, 6), 0, 40, 0, 100, 0, 0, 0,
      { 1048, 1018, 1078 }, { "alternates odd/even", "odd values both fields", "even values both fields" } },
    { "C1", "EQU = 0 (no vsync) accepted?", 0, 0, 429, V(480, 0), 0, 84, 6, 84, 6, 0, 0,
      { 1050 }, { "EQU 0 accepted" } },
    { "C2", "MERGE: 480p from two 240-line fields", 0, 0, 429, V(240, 6), V(240, 0), 60, 0, 0, 12, 240, 0,
      { 525, 534 }, { "MERGED: field-2 vsync removed", "EQU switch not applied" } },
    { "C2c", "control: same, no EQU switch", 0, 0, 429, V(240, 6), 0, 60, 0, 0, 12, 240, 0,
      { 534 }, { "unmerged as expected" } },
    { "C3", "MERGE: 1080p from two 540-line fields", 1, 1, 1023, V(540, 6), V(540, 0), 40, 0, 0, 32, 540, 0,
      { 1125, 1134 }, { "MERGED: 1080 continuous lines", "EQU switch not applied" } },
    { "C3c", "control: same, no EQU switch", 1, 1, 1023, V(540, 6), 0, 40, 0, 0, 32, 540, 0,
      { 1134 }, { "unmerged as expected" } },
};
#define NTESTS (sizeof(tests) / sizeof(tests[0]))

static u32 pll_fast_1c4(void) { return (boot1c4 & ~0x0000FF00u) | ((u32)(20 - 1) << 8); }   // Nfb 20
static u32 pll_fast_1c8(void) { return (boot1c8 & ~0x00F00000u) | (4u << 20); }              // V 4 -> 270 MHz

static void fbb_regs(VIImage *im, u8 hi, u8 lo, const void *p)
{
    u32 t = MEM_VIRTUAL_TO_PHYSICAL(p), flag = 0;
    if (t >= 0x01000000) { flag = 1; t >>= 5; }
    img_set(im, hi, (u16)((flag << 12) | ((t >> 16) & 0xFF)));
    img_set(im, lo, (u16)(t & 0xFFFF));
}

static void build_image(const Test *t, VIImage *im)
{
    *im = safe_img;
    u16 A = t->hd ? 1920 : 720, hsy = t->hd ? 44 : 64, hbe = t->hd ? 104 : 122;
    u16 hbs = (u16)(hbe + A - (t->hlw & 0x3FF));
    u32 *fb = t->hd ? fb1080 : fb480;
    img_set(im, 0x6C, 1);
    img_set(im, 0x00, t->vtr1);
    img_set(im, 0x04, (u16)((40 << 8) | 60));
    img_set(im, 0x06, t->hlw);
    img_set(im, 0x08, (u16)(((hbe >> 9) & 1) | (hbs << 1)));
    img_set(im, 0x0A, (u16)(((hbe & 0x1FF) << 7) | hsy));
    img_set(im, 0x0C, t->psbo); img_set(im, 0x0E, t->prbo);
    img_set(im, 0x10, t->psbe); img_set(im, 0x12, t->prbe);
    fbb_regs(im, 0x1C, 0x1E, fb);
    fbb_regs(im, 0x24, 0x26, (u8 *)fb + (u32)t->bfbb_line * A * 2);
    for (u8 o = 0x30; o <= 0x3E; o += 2) img_set(im, o, 0);      // no display interrupts
    img_set(im, 0x48, (u16)(((A / 16) << 8) | (A / 16)));
    img_set(im, 0x4A, 0x0100);
    im->dcr = (safe_img.dcr & 0x0300) | 0x0004 | 0x0001;        // progressive, enabled
}

// Field bookkeeping for the merge (in lines; DPV counts lines across both fields).
static void merge_points(const Test *t, u32 *w1, u32 *w2)
{
    u32 eq1 = t->vtr1 & 0xF, acv = (t->vtr1 >> 4) & 0x3FF;
    u32 eq2 = t->vtr2 & 0xF;
    u32 f1 = (3 * eq1 + t->prbo + 2 * acv + t->psbo) / 2;
    u32 f2 = (3 * eq2 + t->prbe + 2 * acv + t->psbe) / 2;
    *w1 = f1 / 2;            // middle of field 1's active area: switch to field-2 EQU
    *w2 = f1 + f2 / 2;       // middle of field 2's active area: switch back
}

typedef struct {
    bool moving; u32 lines, resets, top, rmin, rmax, orbits, writes;
    u64 ns, cyc_ns; double line_hz, cyc_lines;
} Meas;

// Poll the beam counter for `ms`. Line rate from counter increments; frame-cycle length
// from the time between counter resets. With merge, also run the EQU switch state machine.
static int mstate;   // EQU switch state, kept across calls so a held view never skips a field

static Meas run_measure(const Test *t, u32 ms, bool merge)
{
    Meas m; memset(&m, 0, sizeof(m));
    m.rmin = 0xFFFF;
    u32 w1 = 0, w2 = 0;
    if (merge) merge_points(t, &w1, &w2);
    u16 p = VI[0x2C / 2], raw = p;
    u64 to = gettime() + millisecs_to_ticks(60);
    while ((raw = VI[0x2C / 2]) == p) if (gettime() > to) return m;
    u64 t0 = gettime(), tl = t0, end = t0 + millisecs_to_ticks(ms), first_reset = 0, last_reset = 0;
    u32 prev = raw;
    m.top = raw;
    while (1) {
        u64 now = gettime();
        if (now >= end) break;
        if (now - tl > millisecs_to_ticks(40)) break;
        raw = VI[0x2C / 2];
        u32 d = raw;
        if (d != prev) {
            m.orbits |= raw;
            if (d > prev) m.lines += d - prev;
            else {
                m.lines += 1;
                if (!first_reset) first_reset = now; else m.resets++;
                last_reset = now;
                if (prev > m.top) m.top = prev;
                if (d < m.rmin) m.rmin = d;
                if (d > m.rmax) m.rmax = d;
            }
            if (d > m.top) m.top = d;
            prev = d; tl = now;
            if (merge) {
                u32 dl = d & 0x7FF;
                if (mstate == 0 && dl >= w1 && dl < w2) { VI[0x00 / 2] = t->vtr2; mstate = 1; m.writes++; }
                else if (mstate == 1 && (dl >= w2 || dl < w1)) { VI[0x00 / 2] = t->vtr1; mstate = 0; m.writes++; }
            }
        }
        heartbeat();
    }
    m.ns = ticks_to_nanosecs(tl - t0);
    m.moving = (gettime() - tl) < millisecs_to_ticks(40) && m.lines > 10;
    if (m.ns) m.line_hz = (double)m.lines * 1e9 / (double)m.ns;
    if (m.resets) {
        m.cyc_ns = ticks_to_nanosecs(last_reset - first_reset) / m.resets;
        m.cyc_lines = m.line_hz * (double)m.cyc_ns / 1e9;
    }
    return m;
}

static char verdicts[NTESTS][64];
static bool verdict_ok[NTESTS];

static void run_test(int i)
{
    const Test *t = &tests[i];
    VIImage im;
    build_image(t, &im);
    bool merge = t->vtr2 != 0;
    double pix = t->fast ? 135e6 : 27e6;                         // pixel rate = F / 2

    logf_("");
    logf_("%s  %s  (%s, HLW reg %04X, VTR %04X%s%04X, PRB/PSB odd %u/%u even %u/%u)", t->id, t->name,
          t->fast ? "270 MHz" : "54 MHz", t->hlw, t->vtr1, merge ? " / field 2 " : "", merge ? t->vtr2 : 0,
          t->prbo, t->psbo, t->prbe, t->psbe);
    apply_image(&im, t->fast ? pll_fast_1c4() : boot1c4, t->fast ? pll_fast_1c8() : boot1c8);
    hb_ms = 62;
    mstate = 0;
    delay_ms(100);
    Meas m = run_measure(t, 600, merge);
    restore_safe();
    hb_ms = 250;

    double line_clk = m.line_hz > 0 ? pix / m.line_hz : 0;
    double val = t->kind ? line_clk : m.cyc_lines;
    logf_("  %s: line %.1f Hz (%.1f clocks), %lu counter resets, cycle %.2f lines (%.3f Hz), counter top %u, "
          "resets to %u..%u, bits seen %04X",
          m.moving ? "moving" : "STOPPED", m.line_hz, line_clk, (unsigned long)m.resets, m.cyc_lines,
          m.cyc_ns ? 1e9 / m.cyc_ns : 0.0, (unsigned)m.top, m.rmin == 0xFFFF ? 0 : (unsigned)m.rmin,
          (unsigned)m.rmax, (unsigned)m.orbits);
    if (merge) logf_("  EQU switches written: %lu", (unsigned long)m.writes);

    int best = -1; double bestd = 1e9;
    for (int h = 0; h < 3 && t->hypn[h]; h++) {
        double tol = t->hyp[h] * 0.004 + 1.5;
        double dd = fabs(val - t->hyp[h]);
        if (dd <= tol && dd < bestd) { best = h; bestd = dd; }
    }
    if (!m.moving) snprintf(verdicts[i], sizeof(verdicts[i]), "VI stopped");
    else if (best >= 0) snprintf(verdicts[i], sizeof(verdicts[i]), "%s", t->hypn[best]);
    else snprintf(verdicts[i], sizeof(verdicts[i]), "unexpected: %.1f %s", val, t->kind ? "clocks/line" : "lines");
    verdict_ok[i] = best >= 0;
    logf_("  %s = %.2f %s -> %s", t->kind ? "line length" : "frame cycle", val, t->kind ? "clocks" : "lines", verdicts[i]);
}

// ======================================================================= test pictures
static u32 yuv2(u8 y, u8 cb, u8 cr) { return ((u32)y << 24) | ((u32)cb << 16) | ((u32)y << 8) | cr; }

static void draw_fb(u32 *fb, int W, int H, int rows_alloc)
{
    int w2 = W / 2;
    for (int y = 0; y < rows_alloc; y++)
        for (int x = 0; x < w2; x++) {
            u32 c;
            if (y >= H) c = yuv2(60, 128, 128);
            else {
                bool top = y < H / 2;
                c = top ? yuv2(45, 170, 110) : yuv2(50, 110, 175);          // blue top, red bottom
                int px = x * 2;
                int diag = (int)((long)y * W / H);
                if (abs(px - diag) < 6) c = yuv2(235, 128, 128);              // diagonal across the seam
                if (y % (H / 12) < 2) c = yuv2(180, 128, 128);                // ruler lines
                if (y < 3 || y >= H - 3 || px < 6 || px >= W - 6) c = yuv2(235, 128, 128);
                if (abs(y - H / 2) < 3 && px > W / 3 && px < 2 * W / 3) c = yuv2(162, 44, 142); // yellow seam mark
                if (px >= W - 60 && px < W - 20) c = top ? yuv2(41, 240, 110) : yuv2(81, 90, 240); // side tab
            }
            fb[y * w2 + x] = c;
        }
}

// ======================================================================= input
#define B_A    1
#define B_STOP 2
#define B_HOME 4
#define B_UP   8
#define B_DOWN 16
static u32 read_buttons(void)
{
    WPAD_ScanPads(); PAD_ScanPads();
    u32 w = WPAD_ButtonsDown(0), g = PAD_ButtonsDown(0), b = 0;
    if ((w & WPAD_BUTTON_A) || (g & PAD_BUTTON_A)) b |= B_A;
    if ((w & WPAD_BUTTON_B) || (g & PAD_BUTTON_B) || (g & PAD_BUTTON_START) || (w & WPAD_BUTTON_PLUS)) b |= B_STOP;
    if (w & WPAD_BUTTON_HOME) b |= B_HOME | B_STOP;
    if ((w & WPAD_BUTTON_UP) || (g & PAD_BUTTON_UP)) b |= B_UP;
    if ((w & WPAD_BUTTON_DOWN) || (g & PAD_BUTTON_DOWN)) b |= B_DOWN;
    return b;
}

// Hold a raster on screen (with the EQU switching running) until a button is pressed.
static void view(int ti, bool merge)
{
    const Test *t = &tests[ti];
    VIImage im;
    build_image(t, &im);
    logf_("");
    logf_("VIEW %s %s", t->id, merge ? "(merging)" : "(not merging)");
    apply_image(&im, boot1c4, boot1c8);
    hb_ms = 62;
    mstate = 0;
    u32 frames = 0;
    while (1) {
        Meas m = run_measure(t, 150, merge && t->vtr2);
        frames++;
        if (frames == 4) logf_("  cycle %.2f lines, line %.1f Hz, EQU writes %lu", m.cyc_lines, m.line_hz, (unsigned long)m.writes);
        if (read_buttons() & (B_STOP | B_A)) break;
    }
    restore_safe();
    hb_ms = 250;
}

// ======================================================================= main
static int sel;
#define NITEMS 4
static const char *items[NITEMS] = {
    "Run all tests (about 15 s, screen blanks)",
    "View: 480p built from two merged 240-line fields",
    "View: same two fields, NOT merged (control)",
    "Exit",
};

static void draw_menu(bool have_results)
{
    printf("\x1b[2J\x1b[37;40m");
    scr_line(1, "VI Line Test  -  Wii-VI-Experiments");
    scr_line(2, "Can the VI exceed 1023 lines per field? Slot LED: slow = here,");
    scr_line(3, "fast = test raster running, stopped = hung.");
    for (int i = 0; i < NITEMS; i++)
        scr_line(5 + i, "%s%c %s", i == sel ? "\x1b[30;42m" : "", i == sel ? '>' : ' ', items[i]);
    if (have_results) {
        scr_line(10, "\x1b[33mBits: VTR %04X  HLW %04X  PRB %04X, %d regs with extra bits",
                 vtr_bits, hlw_bits, prb_bits, extra_bits_found);
        for (u32 i = 0; i < NTESTS; i++)
            scr_line(11 + i, "%-3s %-38.38s %s%.32s", tests[i].id, tests[i].name,
                     verdict_ok[i] ? "\x1b[32m" : "\x1b[31m", verdicts[i]);
    } else {
        scr_line(10, "Run the tests first; results show here and go to vilinetest.txt.");
        scr_line(12, "The views hold a 480p-rate picture (31.5 kHz, 60 Hz): if the merge");
        scr_line(13, "works, a 480p display shows one normal picture, blue top half and");
        scr_line(14, "red bottom half, with the white diagonal continuous across the seam.");
    }
    scr_line(26, "D-pad: choose   A: select   B/START/HOME: leave a view%s", fat_ok ? "" : "   \x1b[31m(no USB/SD!)");
}

int main(void)
{
    VIDEO_Init();
    WPAD_Init();
    PAD_Init();
    VIDEO_SetPostRetraceCallback(on_retrace);

    boot1c4 = HW(0x1C4) & 0x7FFFFFFFu;
    boot1c8 = HW(0x1C8);

    // 720-wide buffer tall enough for the largest raster (B4: 2148 active lines)
    fb480 = MEM_K0_TO_K1(memalign(32, 720 * 2200 * 2));
    fb1080 = MEM_K0_TO_K1(memalign(32, 1920 * 1080 * 2));
    menu_xfb = MEM_K0_TO_K1(memalign(32, 720 * 576 * 2));

    safe = VIDEO_GetPreferredMode(NULL);
    VIDEO_Configure(safe);
    VIDEO_ClearFrameBuffer(safe, menu_xfb, COLOR_BLACK);
    VIDEO_SetNextFramebuffer(menu_xfb);
    VIDEO_SetBlack(false);
    VIDEO_Flush();
    for (int i = 0; i < 4; i++) wait_retrace(200);
    img_read(&safe_img);
    CON_Init(menu_xfb, 20, 20, safe->fbWidth, safe->xfbHeight, safe->fbWidth * VI_DISPLAY_PIX_SZ);

    draw_fb(fb480, 720, 480, 2200);
    draw_fb(fb1080, 1920, 1080, 1080);

    fat_ok = fatInitDefault();
    logf_("");
    logf_("################ vilinetest session start, build " __DATE__ " " __TIME__);
    logf_("boot 0x1C4=%08X 0x1C8=%08X  safe viTVMode=%u", (unsigned)boot1c4, (unsigned)boot1c8, safe->viTVMode);

    bool have = false;
    draw_menu(have);
    while (1) {
        u32 b = read_buttons();
        if (b & B_HOME) break;
        if (b & B_UP)   { sel = (sel + NITEMS - 1) % NITEMS; draw_menu(have); }
        if (b & B_DOWN) { sel = (sel + 1) % NITEMS; draw_menu(have); }
        if (b & B_A) {
            if (sel == 3) break;
            if (sel == 0) {
                scr_line(10, "\x1b[36mPart 1: reading register bits...");
                part1_readback();
                for (u32 i = 0; i < NTESTS; i++) {
                    scr_line(10, "\x1b[36mRunning %s: %s", tests[i].id, tests[i].name);
                    wait_retrace(100);
                    run_test(i);
                }
                have = true;
                logf_("");
                logf_("SUMMARY: VTR bits %04X, HLW bits %04X, PRB bits %04X, %d registers with bits beyond the documented field",
                      vtr_bits, hlw_bits, prb_bits, extra_bits_found);
                for (u32 i = 0; i < NTESTS; i++) logf_("  %-3s %-42s %s", tests[i].id, tests[i].name, verdicts[i]);
            } else {
                // C2 / C2c layout: index of C2 in the table
                int c2 = 0;
                for (u32 i = 0; i < NTESTS; i++) if (!strcmp(tests[i].id, "C2")) c2 = i;
                view(c2, sel == 1);
            }
            draw_menu(have);
        }
        wait_retrace(100);
    }

    logf_("exit to loader");
    u32 l; _CPU_ISR_Disable(l); HW(0x1C4) = boot1c4; HW(0x1C8) = boot1c8; _CPU_ISR_Restore(l);
    VIDEO_Configure(safe);
    VIDEO_SetBlack(true);
    VIDEO_Flush();
    wait_retrace(200);
    GPIOB_OUT &= ~0x20;
    return 0;
}
