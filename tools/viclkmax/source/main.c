// viclkmax: find the highest pixel clock the Wii's Video Interface (VI) will run at.
//
// The app climbs the VI PLL step by step. At every step it:
//   1. writes the step to a state file and the log BEFORE touching the clock (so if the
//      console hangs, the next launch knows exactly which clock did it),
//   2. stops and resets the VI, programs the PLL (multiplier Nfb, divider V), waits for it
//      to relock, and starts a fixed 1920x960 progressive raster (2046-clock lines),
//   3. measures the clock the VI is really running at by counting scan lines on the VI's
//      own beam counter against the CPU timebase (works with no display attached),
//   4. switches straight back to the console's normal safe mode and shows the result.
//
// A step passes when the measured clock matches the programmed one within 0.3 %, twice,
// with the beam counter wrapping at a consistent line. The sweep climbs in ~6 % steps until
// a step fails, then fills in every reachable clock between the last pass and that failure,
// then holds the best clock for a 10 s soak. Results persist in viclkmax_state.txt: after a
// hang, power-cycle and launch again, and the step that hung is counted as a failure.
//
// Clock law (docs/VI_PLL.md): F = 54 MHz * Nfb / V, Nfb = 0x1C4[15:8] + 1, V = 0x1C8[23:20].
// The PLL's VCO runs at 108 MHz * Nfb. Nfb stays within 7..25 (VCO 756..2700 MHz, the range
// already verified to lock); higher clocks come from smaller V. 0x1C4 byte 2 is never
// written. With VICLK = 1 the VI's pixel rate is F / 2.
//
// Sign of life: the disc-slot LED blinks slowly (2/s) at the menu and fast (8/s) while a
// test clock is running. If it stops blinking, the console hung.

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
static u32 *test_xfb;
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
static u32 dpv(void) { return VI[0x2C / 2] & 0x7FF; }

// ======================================================================= files
static const char *const log_paths[]   = { "usb:/viclkmax.txt", "sd:/viclkmax.txt" };
static const char *const state_paths[] = { "usb:/viclkmax_state.txt", "sd:/viclkmax_state.txt" };

static void append_line(const char *const *paths, const char *line)
{
    if (!fat_ok) return;
    for (int i = 0; i < 2; i++) {
        FILE *f = fopen(paths[i], "a");       // one open/close per line: survives a hang
        if (f) { fputs(line, f); fputc('\n', f); fclose(f); }
    }
}
static void logf_(const char *fmt, ...)
{
    char line[320];
    va_list ap; va_start(ap, fmt); vsnprintf(line, sizeof(line), fmt, ap); va_end(ap);
    append_line(log_paths, line);
}
static void statef(const char *fmt, ...)
{
    char line[64];
    va_list ap; va_start(ap, fmt); vsnprintf(line, sizeof(line), fmt, ap); va_end(ap);
    append_line(state_paths, line);
}

// ======================================================================= results
#define NFB_MIN 7
#define NFB_MAX 25
#define V_MIN   1
#define V_MAX   15
typedef struct { char code; u32 khz; } Res;      // code: 0 untested, P pass, F fail, H hang
static Res  res[NFB_MAX + 1][V_MAX + 1];
static char soak[NFB_MAX + 1][V_MAX + 1];        // 0 untested, S pass, X fail/hang
static char hang_note[96];

static void clear_results(void)
{
    memset(res, 0, sizeof(res));
    memset(soak, 0, sizeof(soak));
    hang_note[0] = 0;
}

static double f_of(int n, int v) { return 54.0 * n / v; }

static void load_state(void)
{
    clear_results();
    if (!fat_ok) return;
    FILE *f = NULL;
    for (int i = 0; i < 2 && !f; i++) f = fopen(state_paths[i], "r");
    if (!f) return;
    static u8 pend[NFB_MAX + 1][V_MAX + 1];      // 1 = test started, 2 = soak started
    memset(pend, 0, sizeof(pend));
    char line[80];
    while (fgets(line, sizeof(line), f)) {
        char c; int n, v; unsigned long k = 0;
        if (sscanf(line, "%c %d %d", &c, &n, &v) != 3) continue;
        if (n < NFB_MIN || n > NFB_MAX || v < V_MIN || v > V_MAX) continue;
        switch (c) {
        case 'T': pend[n][v] = 1; break;
        case 'U': pend[n][v] = 2; break;
        case 'R': {
            char code = 0;
            if (sscanf(line, "R %d %d %c %lu", &n, &v, &code, &k) >= 3) {
                res[n][v].code = code; res[n][v].khz = (u32)k;
            }
            pend[n][v] = 0;
            break; }
        case 'S': case 'X': soak[n][v] = c; pend[n][v] = 0; break;
        }
    }
    fclose(f);
    // A step that was started but never finished hung the console.
    for (int n = NFB_MIN; n <= NFB_MAX; n++)
        for (int v = V_MIN; v <= V_MAX; v++) {
            if (pend[n][v] == 1) {
                res[n][v].code = 'H'; res[n][v].khz = 0;
                statef("R %d %d H 0", n, v);
                snprintf(hang_note, sizeof(hang_note), "Last run hung at %.3f MHz (Nfb %d, V %d): counted as a failure.",
                         f_of(n, v), n, v);
                logf_("previous run hung while testing Nfb=%d V=%d (%.3f MHz); recorded as a failure", n, v, f_of(n, v));
            } else if (pend[n][v] == 2) {
                soak[n][v] = 'X';
                statef("X %d %d", n, v);
                snprintf(hang_note, sizeof(hang_note), "Last run hung during the soak at %.3f MHz (Nfb %d, V %d).",
                         f_of(n, v), n, v);
                logf_("previous run hung during the soak at Nfb=%d V=%d (%.3f MHz); recorded as a failure", n, v, f_of(n, v));
            }
        }
}

// ======================================================================= clock points
// Every reachable clock, F = 54 * Nfb / V, grouped by frequency. Each frequency lists its
// factorisations in order of preference: V within the verified 4..10 first, then lowest VCO.
typedef struct { double mhz; u8 nc; u8 n[16]; u8 v[16]; } Pt;
static Pt pts[NFB_MAX * V_MAX];
static int npts, base_pt;

static int pref(int n, int v) { return ((v >= 4 && v <= 10) ? 0 : 1000) + n; }

static void build_points(void)
{
    npts = 0;
    for (int n = NFB_MIN; n <= NFB_MAX; n++)
        for (int v = V_MIN; v <= V_MAX; v++) {
            double f = f_of(n, v);
            if (f < 54.0 - 1e-9) continue;
            int i;
            for (i = 0; i < npts; i++) if (fabs(pts[i].mhz - f) < 1e-6) break;
            if (i == npts) { pts[npts].mhz = f; pts[npts].nc = 0; npts++; }
            Pt *p = &pts[i];
            if (p->nc < 16) { p->n[p->nc] = n; p->v[p->nc] = v; p->nc++; }
        }
    // sort points by frequency, combos by preference (simple insertion sorts)
    for (int i = 1; i < npts; i++) {
        Pt t = pts[i]; int j = i - 1;
        while (j >= 0 && pts[j].mhz > t.mhz) { pts[j + 1] = pts[j]; j--; }
        pts[j + 1] = t;
    }
    for (int i = 0; i < npts; i++) {
        Pt *p = &pts[i];
        for (int a = 1; a < p->nc; a++) {
            u8 tn = p->n[a], tv = p->v[a]; int b = a - 1;
            while (b >= 0 && pref(p->n[b], p->v[b]) > pref(tn, tv)) { p->n[b + 1] = p->n[b]; p->v[b + 1] = p->v[b]; b--; }
            p->n[b + 1] = tn; p->v[b + 1] = tv;
        }
    }
    base_pt = 0;
    for (int i = 0; i < npts; i++) if (fabs(pts[i].mhz - 54.0) < 1e-6) base_pt = i;
    // the baseline must be the stock setting itself
    Pt *b = &pts[base_pt];
    for (int a = 0; a < b->nc; a++) if (b->n[a] == 7 && b->v[a] == 7) {
        b->n[a] = b->n[0]; b->v[a] = b->v[0]; b->n[0] = 7; b->v[0] = 7;
    }
}

static bool combo_pass(int n, int v) { return res[n][v].code == 'P' && soak[n][v] != 'X'; }

// 1 = this clock works, 0 = it doesn't, -1 = a combo needs testing (returned in *tn, *tv).
// A clock fails only after its first two factorisations fail, so a single bad divider
// setting isn't mistaken for the VI's limit. After a hang no second attempt is made.
static int eval_pt(int i, int *tn, int *tv)
{
    const Pt *p = &pts[i];
    for (int a = 0; a < p->nc && a < 2; a++) {
        int n = p->n[a], v = p->v[a];
        if (combo_pass(n, v)) return 1;
        if (res[n][v].code == 0) { *tn = n; *tv = v; return -1; }
        if (res[n][v].code == 'H' || soak[n][v] == 'X') return 0;
    }
    return 0;
}
static bool pt_pass_combo(int i, int *n, int *v)
{
    for (int a = 0; a < pts[i].nc; a++)
        if (combo_pass(pts[i].n[a], pts[i].v[a])) { *n = pts[i].n[a]; *v = pts[i].v[a]; return true; }
    return false;
}

enum { NEXT_TEST, NEXT_SOAK, NEXT_DONE };
static int best_pt, fail_pt;

// Decide the next step purely from the results so far (so a resumed run carries on).
static int next_step(int *tn, int *tv)
{
    best_pt = -1; fail_pt = -1;
    int r = eval_pt(base_pt, tn, tv);
    if (r < 0) return NEXT_TEST;
    if (r == 0) return NEXT_DONE;            // even the stock clock failed: measurement problem
    int cur = base_pt;
    // coarse climb, ~6 % per step
    while (cur < npts - 1) {
        int j = cur + 1;
        while (j < npts - 1 && pts[j].mhz < pts[cur].mhz * 1.06 - 1e-9) j++;
        r = eval_pt(j, tn, tv);
        if (r < 0) return NEXT_TEST;
        if (r == 1) cur = j; else { fail_pt = j; break; }
    }
    // fill in every clock between the last pass and the first failure
    if (fail_pt >= 0)
        for (int j = cur + 1; j < fail_pt; j++) {
            r = eval_pt(j, tn, tv);
            if (r < 0) return NEXT_TEST;
            if (r == 1) cur = j; else { fail_pt = j; break; }
        }
    best_pt = cur;
    int n, v;
    if (!pt_pass_combo(cur, &n, &v)) return NEXT_DONE;
    if (soak[n][v] == 0) { *tn = n; *tv = v; return NEXT_SOAK; }
    return NEXT_DONE;
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
static VIImage safe_img, test_img;

static u16 img_mask(u8 off)
{
    if (off == 0x30 || off == 0x34 || off == 0x38 || off == 0x3C) return 0x7FFF;  // IRQ status bit
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

// Test raster: 1920x960 progressive, 2046-clock lines (the longest the 10-bit half-line
// register allows), 1000 lines per frame. At the stock 54 MHz that is 13.2 kHz / 13.2 Hz;
// every MHz of F adds 244 Hz of line rate. Large on purpose: the VI fetches 1920 pixels
// per line from memory, so the memory path is loaded too.
#define T_LINE  2046
#define T_HLW   1023
#define T_A     1920
#define T_HSY   44
#define T_HBE   (44 + 60)
#define T_ACV   960
#define T_EQU   6
#define T_PSB   4
#define T_N     2000                       // half-lines per field (two per progressive line)
#define T_PRB   (T_N - 3 * T_EQU - 2 * T_ACV - T_PSB)

static void build_test_image(void)
{
    VIImage *im = &test_img;
    *im = safe_img;
    u16 wpl = T_A / 16;
    img_set(im, 0x6C, 1);                                        // VICLK: F straight through
    img_set(im, 0x00, (u16)((T_ACV << 4) | T_EQU));
    img_set(im, 0x04, (u16)((40 << 8) | 90));                    // burst window (unused)
    img_set(im, 0x06, T_HLW);
    img_set(im, 0x08, (u16)(((T_HBE >> 9) & 1) | ((T_HBE + T_A - T_HLW) << 1)));
    img_set(im, 0x0A, (u16)(((T_HBE & 0x1FF) << 7) | T_HSY));
    img_set(im, 0x0C, T_PSB); img_set(im, 0x0E, T_PRB);
    img_set(im, 0x10, T_PSB); img_set(im, 0x12, T_PRB);
    u16 bb = (u16)(((T_N - T_EQU) << 5) | (2 * T_EQU));
    img_set(im, 0x14, bb); img_set(im, 0x16, bb); img_set(im, 0x18, bb); img_set(im, 0x1A, bb);
    u32 t = MEM_VIRTUAL_TO_PHYSICAL(test_xfb), flag = 0;
    if (t >= 0x01000000) { flag = 1; t >>= 5; }
    img_set(im, 0x1C, (u16)((flag << 12) | ((t >> 16) & 0xFF)));
    img_set(im, 0x1E, (u16)(t & 0xFFFF));
    img_set(im, 0x24, (u16)((t >> 16) & 0xFF));
    img_set(im, 0x26, (u16)(t & 0xFFFF));
    img_set(im, 0x30, (u16)(0x1000 | (T_N / 2 + 1)));
    img_set(im, 0x32, 1);
    img_set(im, 0x34, 0x1001); img_set(im, 0x36, 0x0001);
    img_set(im, 0x48, (u16)((wpl << 8) | wpl));
    img_set(im, 0x4A, 0x0100);                                   // scaler off
    im->dcr = (safe_img.dcr & 0x0300) | 0x0004 | 0x0001;         // FMT, progressive, enable
}

// VI off -> reset -> PLL -> relock -> full register image -> VI on.
static void apply_image(const VIImage *im, u32 p4, u32 p8)
{
    u32 l; _CPU_ISR_Disable(l);
    VI[0x02 / 2] = VI[0x02 / 2] & ~0x0001;
    _CPU_ISR_Restore(l);
    delay_ms(2);
    _CPU_ISR_Disable(l);
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

// ======================================================================= measurement
// Count VI scan lines on the beam counter for a fixed time. The CPU polls far faster than
// one line (>= 3 us even at the top of the range), so every line step is seen.
typedef struct { bool moving; u32 lines, top; u64 ns, max_gap_ns; double line_hz, f_mhz; } Meas;

static Meas measure(u32 ms)
{
    Meas m; memset(&m, 0, sizeof(m));
    u32 p = dpv(), d;
    u64 to = gettime() + millisecs_to_ticks(40);
    while ((d = dpv()) == p) if (gettime() > to) return m;      // counter not moving
    u64 t0 = gettime(), tl = t0, end = t0 + millisecs_to_ticks(ms);
    u32 prev = d;
    m.top = d;
    while (1) {
        u64 now = gettime();
        if (now >= end) break;
        if (now - tl > millisecs_to_ticks(20)) break;            // stalled
        d = dpv();
        if (d != prev) {
            m.lines += (d > prev) ? d - prev : 1;                // wrap = one line step
            u64 gap = ticks_to_nanosecs(now - tl);
            if (gap > m.max_gap_ns) m.max_gap_ns = gap;
            prev = d; tl = now;
            if (d > m.top) m.top = d;
        }
        heartbeat();
    }
    m.ns = ticks_to_nanosecs(tl - t0);
    m.moving = (gettime() - tl) < millisecs_to_ticks(20) && m.lines > 10;
    if (m.ns) {
        m.line_hz = (double)m.lines * 1e9 / (double)m.ns;
        m.f_mhz = m.line_hz * 2.0 * T_LINE / 1e6;                // pixel rate = F/2
    }
    return m;
}

// ======================================================================= screen
#define HIST 14
static char hist[HIST][80];
static int nhist;

static void scr_line(int row, const char *fmt, ...)
{
    char buf[160];
    va_list ap; va_start(ap, fmt); vsnprintf(buf, sizeof(buf), fmt, ap); va_end(ap);
    printf("\x1b[%d;2H\x1b[K%s\x1b[37m", row, buf);
}
static void hist_add(const char *fmt, ...)
{
    if (nhist == HIST) { memmove(hist[0], hist[1], sizeof(hist[0]) * (HIST - 1)); nhist--; }
    va_list ap; va_start(ap, fmt); vsnprintf(hist[nhist], sizeof(hist[0]), fmt, ap); va_end(ap);
    nhist++;
}
static void scr_best(void)
{
    double best = 0; int bn = 0, bv = 0;
    for (int n = NFB_MIN; n <= NFB_MAX; n++)
        for (int v = V_MIN; v <= V_MAX; v++)
            if (combo_pass(n, v) && f_of(n, v) > best) { best = f_of(n, v); bn = n; bv = v; }
    if (best > 0)
        scr_line(6, "Best so far: \x1b[32m%.3f MHz\x1b[37m (Nfb %d V %d), pixel %.3f MHz%s",
                 best, bn, bv, best / 2, soak[bn][bv] == 'S' ? ", soak OK" : "");
    else
        scr_line(6, "Best so far: none yet");
}
static void scr_table(void)
{
    scr_line(8, "\x1b[33m  F (MHz)  Nfb  V    VCO   measured   result");
    for (int i = 0; i < HIST; i++) scr_line(9 + i, "%s", i < nhist ? hist[i] : "");
}
static void scr_frame(void)
{
    printf("\x1b[2J\x1b[37;40m");
    scr_line(1, "VI Clock Max  -  Wii-VI-Experiments");
    scr_line(2, "Climbs the VI pixel clock until the VI stops working. Slot LED:");
    scr_line(3, "slow blink = here, fast blink = test clock running, stopped = hung.");
    scr_best();
    scr_table();
}

// ======================================================================= one step
static u32 pll_1c4(int n, int v)
{
    if (n == 7 && v == 7) return boot1c4;
    return (boot1c4 & ~0x0000FF00u) | ((u32)(n - 1) << 8);       // byte 2 never touched
}
static u32 pll_1c8(int n, int v)
{
    if (n == 7 && v == 7) return boot1c8;
    return (boot1c8 & ~0x00F00000u) | ((u32)v << 20);
}

static void run_step(int n, int v, bool is_soak)
{
    double fx = f_of(n, v);
    u32 p4 = pll_1c4(n, v), p8 = pll_1c8(n, v);
    int nmeas = is_soak ? 10 : 2;

    scr_line(5, "\x1b[36m%s %.3f MHz (Nfb %d V %d), picture off ~%d s",
             is_soak ? "Soak (10 s) at" : "Testing", fx, n, v, is_soak ? 11 : 2);
    wait_retrace(100); wait_retrace(100);       // let the status line reach the screen

    // write-ahead: if the console hangs from here on, the next launch knows why
    statef("%c %d %d", is_soak ? 'U' : 'T', n, v);
    logf_("");
    logf_("%s Nfb=%d V=%d  F=%.3f MHz  VCO=%u MHz  0x1C4=%08X 0x1C8=%08X%s",
          is_soak ? "SOAK" : "TRY ", n, v, fx, 108u * n, (unsigned)p4, (unsigned)p8,
          (v < 4 || v > 10) ? "  (V outside verified 4..10)" : "");

    hb_ms = 62;
    apply_image(&test_img, p4, p8);
    delay_ms(100);
    u32 lock = HW(0x1C4) >> 31;
    u32 r0 = retraces; u64 t0 = gettime();

    Meas ms[10];
    for (int i = 0; i < nmeas; i++) {
        ms[i] = measure(250);
        if (i < nmeas - 1) delay_ms(is_soak ? 750 : 500);
    }
    u32 rr = retraces - r0;
    u64 el_us = ticks_to_microsecs(gettime() - t0);

    bool back = restore_safe();
    hb_ms = 250;

    // evaluate
    bool ok = true;
    double fmin = 1e9, fmax = 0;
    u32 top0 = ms[0].top;
    for (int i = 0; i < nmeas; i++) {
        const Meas *m = &ms[i];
        logf_("  meas %d: %s lines=%lu in %llu ns  line %.1f Hz  F=%.4f MHz (%+.3f %%)  DPV top %lu  max gap %llu ns",
              i + 1, m->moving ? "moving " : "STOPPED", (unsigned long)m->lines, m->ns, m->line_hz, m->f_mhz,
              fx > 0 ? (m->f_mhz / fx - 1) * 100 : 0, (unsigned long)m->top, m->max_gap_ns);
        if (!m->moving) { ok = false; continue; }
        if (m->f_mhz < fmin) fmin = m->f_mhz;
        if (m->f_mhz > fmax) fmax = m->f_mhz;
        if (fabs(m->f_mhz / fx - 1) > 0.003) ok = false;
        if (m->top != top0) ok = false;
    }
    double fr = el_us ? rr * 1e6 / el_us : 0;
    logf_("  PLL lock bit %lu, %lu retrace IRQs in %llu us (%.2f Hz; expected %.2f Hz), back to safe mode: %s",
          (unsigned long)lock, (unsigned long)rr, el_us, fr, fx * 1e6 / 2 / T_LINE / (T_N / 2), back ? "yes" : "NO");

    const char *why = ok ? "OK" : "FAIL";
    char reason[40] = "";
    if (!ok) {
        bool any_stop = false; for (int i = 0; i < nmeas; i++) if (!ms[i].moving) any_stop = true;
        if (any_stop) snprintf(reason, sizeof(reason), "VI stopped");
        else if (fmax > 0 && fabs(fmax / fx - 1) > 0.003) snprintf(reason, sizeof(reason), "wrong clock");
        else snprintf(reason, sizeof(reason), "unstable");
    }
    double fshow = fmax > 0 ? (fmin + fmax) / 2 : 0;
    logf_("  RESULT %s%s%s  measured %.4f MHz", why, ok ? "" : ": ", reason, fshow);

    if (is_soak) {
        soak[n][v] = ok ? 'S' : 'X';
        statef("%c %d %d", ok ? 'S' : 'X', n, v);
    } else {
        res[n][v].code = ok ? 'P' : 'F';
        res[n][v].khz = (u32)(fshow * 1000 + 0.5);
        statef("R %d %d %c %lu", n, v, ok ? 'P' : 'F', (unsigned long)res[n][v].khz);
    }
    hist_add("%9.3f  %3d %2d  %5u  %9.3f   %s%s%s%s", fx, n, v, 108u * n, fshow,
             ok ? "\x1b[32m" : "\x1b[31m", is_soak ? (ok ? "soak OK" : "soak FAIL") : why,
             ok ? "" : " ", reason);
    scr_best();
    scr_table();
}

// ======================================================================= summary
typedef struct { const char *name; u16 T, lines, acv; bool il; u16 rate; } Target;
static const Target targets[] = {
    { "1280x720p60",               1650,  750,  720, false, 60 },
    { "1920x1080i60 (2046 line)",  2046, 1125,  540, true,  30 },
    { "1280x960p60",               1800, 1000,  960, false, 60 },
    { "1920x1020p60 (tallest p)",  2046, 1060, 1020, false, 60 },
    { "1920x2040i60 (tallest i)",  2046, 2100, 1020, true,  30 },
    { "1920x1080i120 (2046 line)", 2046, 1125,  540, true,  60 },
    { "1280x720p120",              1650,  750,  720, false, 120 },
    { "1920x1020p120",             2046, 1060, 1020, false, 120 },
    { "1920x1080i60 (2200 line)",  2200, 1125,  540, true,  30 },
    { "1920x1080p60",              2200, 1125, 1080, false, 60 },
};

static void show_summary(int state)
{
    double best = best_pt >= 0 ? pts[best_pt].mhz : 0;
    int bn = 0, bv = 0;
    if (best_pt >= 0) pt_pass_combo(best_pt, &bn, &bv);
    printf("\x1b[2J\x1b[37;40m");
    scr_line(1, "VI Clock Max  -  result");
    if (state == NEXT_DONE && best_pt < 0) {
        scr_line(3, "\x1b[31mThe stock 54 MHz baseline did not measure correctly.\x1b[37m");
        scr_line(4, "See viclkmax.txt; the measurement needs fixing before a sweep means anything.");
        logf_("SUMMARY: baseline failed");
        return;
    }
    scr_line(3, "Best VI clock: \x1b[32m%.3f MHz\x1b[37m (Nfb %d V %d), pixel rate %.3f MHz",
             best, bn, bv, best / 2);
    if (fail_pt >= 0)
        scr_line(4, "First failing clock: %.3f MHz. Soak at the best clock: %s", pts[fail_pt].mhz,
                 soak[bn][bv] == 'S' ? "passed" : "not run");
    else
        scr_line(4, "No failure up to %.3f MHz, the top of the reachable range.", pts[npts - 1].mhz);
    logf_("");
    logf_("SUMMARY: best %.3f MHz (Nfb %d V %d, pixel rate %.3f MHz), first fail %.3f MHz, soak %s",
          best, bn, bv, best / 2, fail_pt >= 0 ? pts[fail_pt].mhz : 0.0, soak[bn][bv] == 'S' ? "passed" : "not run");

    scr_line(6, "\x1b[33mWhat this clock can drive (timing only, display not tested):\x1b[37m");
    scr_line(7, "\x1b[33m  mode                        needs F   registers  clock");
    int row = 8;
    for (u32 i = 0; i < sizeof(targets) / sizeof(targets[0]); i++) {
        const Target *t = &targets[i];
        double need = 2.0 * t->T * t->lines * t->rate / 1e6;      // F = 2 x pixel rate
        bool regs_ok = t->T <= 2046 && t->acv <= 1023;
        bool clk_ok = need <= best + 1e-6;
        scr_line(row++, "  %-26s %7.1f    %s   %s", t->name, need,
                 regs_ok ? "\x1b[32m  fits  \x1b[37m" : "\x1b[31mtoo big \x1b[37m",
                 clk_ok ? "\x1b[32mOK\x1b[37m" : "\x1b[31mtoo fast\x1b[37m");
        logf_("  %-26s needs F %.1f MHz  registers %s  clock %s", t->name, need,
              regs_ok ? "fit" : "too big", clk_ok ? "OK" : "too fast");
    }
    scr_line(row + 1, "Register limits: line <= 2046 clocks, <= 1023 active lines per field.");
    scr_line(row + 2, "Whether the VI's memory fetch keeps up needs a display that can show it.");
}

// ======================================================================= input
#define B_A    1
#define B_B    2
#define B_HOME 4
#define B_STOP 8
static u32 read_buttons(void)
{
    WPAD_ScanPads(); PAD_ScanPads();
    u32 w = WPAD_ButtonsDown(0), g = PAD_ButtonsDown(0), b = 0;
    if ((w & WPAD_BUTTON_A) || (g & PAD_BUTTON_A)) b |= B_A;
    if ((w & WPAD_BUTTON_B) || (g & PAD_BUTTON_B)) b |= B_B | B_STOP;
    if (w & WPAD_BUTTON_HOME) b |= B_HOME | B_STOP;
    if ((g & PAD_BUTTON_START) || (w & WPAD_BUTTON_PLUS)) b |= B_STOP;
    return b;
}

static int count_results(void)
{
    int c = 0;
    for (int n = NFB_MIN; n <= NFB_MAX; n++) for (int v = V_MIN; v <= V_MAX; v++) if (res[n][v].code) c++;
    return c;
}

static void idle_screen(void)
{
    int tn, tv;
    int st = next_step(&tn, &tv);
    scr_frame();
    int c = count_results();
    if (hang_note[0]) scr_line(5, "\x1b[31m%s\x1b[37m", hang_note);
    else if (c) scr_line(5, "%d clock settings already tested (saved in viclkmax_state.txt).", c);
    else scr_line(5, "No results yet.");
    if (st == NEXT_DONE) scr_line(24, "Sweep complete. A: show result   B: start over   HOME: exit");
    else if (c) scr_line(24, "A: continue the sweep   B: start over   HOME: exit");
    else scr_line(24, "A: start the sweep   HOME: exit");
    scr_line(25, "During the sweep: B, START, + or HOME stops after the current step.");
    scr_line(26, "Log: viclkmax.txt on the USB drive (or SD).%s", fat_ok ? "" : " \x1b[31m(no USB/SD found!)\x1b[37m");
}

// ======================================================================= main
int main(void)
{
    VIDEO_Init();
    WPAD_Init();
    PAD_Init();
    VIDEO_SetPostRetraceCallback(on_retrace);

    boot1c4 = HW(0x1C4) & 0x7FFFFFFFu;
    boot1c8 = HW(0x1C8);

    test_xfb = MEM_K0_TO_K1(memalign(32, T_A * T_ACV * 2));
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

    // 75 % colour bars in the test framebuffer (visible only on a display that can sync)
    static const u8 bars[8][3] = { {180,128,128}, {162,44,142}, {131,156,44}, {112,72,58},
                                   {84,184,198}, {65,100,212}, {35,212,114}, {16,128,128} };
    for (int y = 0; y < T_ACV; y++)
        for (int x = 0; x < T_A / 2; x++) {
            const u8 *c = bars[(x * 2 * 8) / T_A];
            bool edge = y < 4 || y >= T_ACV - 4 || x < 2 || x >= T_A / 2 - 2;
            u8 Y = edge ? 235 : c[0], cb = edge ? 128 : c[1], cr = edge ? 128 : c[2];
            test_xfb[y * (T_A / 2) + x] = ((u32)Y << 24) | ((u32)cb << 16) | ((u32)Y << 8) | cr;
        }

    fat_ok = fatInitDefault();
    logf_("");
    logf_("################ viclkmax session start, build " __DATE__ " " __TIME__);
    logf_("boot 0x1C4=%08X 0x1C8=%08X  safe viTVMode=%u  test raster %dx%dp, line %d clocks, %d lines",
          (unsigned)boot1c4, (unsigned)boot1c8, safe->viTVMode, T_A, T_ACV, T_LINE, T_N / 2);

    build_points();
    build_test_image();
    load_state();

    bool quit = false;
    while (!quit) {
        idle_screen();
        u32 b = 0;
        while (!(b = read_buttons())) wait_retrace(100);
        if (b & B_HOME) break;
        if (b & B_B) {
            for (int i = 0; i < 2; i++) remove(state_paths[i]);
            clear_results(); nhist = 0;
            logf_("state cleared: starting a fresh sweep");
            continue;
        }
        if (!(b & B_A)) continue;

        scr_frame();
        while (1) {
            int tn = 0, tv = 0;
            int st = next_step(&tn, &tv);
            if (st == NEXT_DONE) {
                show_summary(st);
                scr_line(26, "A / B / HOME: back");
                while (!read_buttons()) wait_retrace(100);
                break;
            }
            run_step(tn, tv, st == NEXT_SOAK);
            if (read_buttons() & B_STOP) { logf_("stopped by user"); break; }
        }
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
