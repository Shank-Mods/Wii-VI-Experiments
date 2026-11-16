// Custom VI modes for aveprobe: the timing engine of tools/vimodetest (same clocks, timings,
// register images and safe-mode restore), without its GX renderer. The probe draws straight
// into a YUY2 framebuffer of the mode's size.
//
//   * Pixel clock: F = 54 MHz * Nfb / V (Nfb = 0x1C4[15:8]+1, V = 0x1C8[23:20]), VICLK=1
//     passes F through. Pixel rate on the bus P = F/2. See docs/VI_PLL.md.
//   * Every switch writes a complete VI register image after a VI reset; leaving puts back
//     the image libogc programmed (and the boot PLL), then re-asserts libogc's mode.

#include <gccore.h>
#include <string.h>
#include <ogc/machine/processor.h>
#include <ogc/lwp_watchdog.h>

#include "vimode.h"

#define VI ((volatile u16 *)0xCC002000)
#define HW(o) (*(volatile u32 *)(0xCD800000 + (o)))

typedef struct { u8 viclk, nfb, v; u32 khz; } Clock;
enum { C54, C108, C148 };
static const Clock clocks[] = {
    {1, 7, 7, 54000},    // stock
    {1, 14, 7, 108000},  // Nfb14 V7
    {1, 22, 8, 148500},  // Nfb22 V8
};

typedef struct { u16 T, A, S, BP; u8 hcs, hce; } HTim;
typedef struct { u8 il; u16 N, acv; u8 equ; u16 psb; u16 bs, be; } VTim;

static const HTim H_SD = {858, 720, 64, 58};          // BT.601 / BT.1358
static const HTim H_SD2 = {1716, 1440, 124, 120};     // the above at twice the clock
static const HTim H_720_960 = {1200, 960, 30, 160};
static const HTim H_720_1280 = {1650, 1280, 40, 220}; // SMPTE 296M
static const HTim H_720_480 = {600, 480, 16, 72, 21, 55};
static const VTim V_480P = {0, 1050, 480, 12, 6};     // libogc stock 480p (BT.1358)
static const VTim V_720P = {0, 1500, 720, 10, 2};     // SMPTE 296M line count

typedef struct {
    const char *name, *desc;
    u8 clk;
    const HTim *h;
    const VTim *v;
} Mode;

static const Mode modes[] = {
    {"480p", "720x480p at 54 MHz (stock timing, programmed directly)", C54, &H_SD, &V_480P},
    {"1440x480p", "1440x480p at 108 MHz: standard 480p timing, twice the bus clock", C108, &H_SD2, &V_480P},
    {"480p120", "720x480p at 108 MHz: 480p lines, 63 kHz / 120 Hz", C108, &H_SD, &V_480P},
    {"480x720p", "480x720p at 54 MHz: 720p timing (45 kHz, 750 lines), stock clock", C54, &H_720_480, &V_720P},
    {"960x720p", "960x720p at 108 MHz: 720p timing", C108, &H_720_960, &V_720P},
    {"1280x720p", "1280x720p at 148.5 MHz: SMPTE 296M", C148, &H_720_1280, &V_720P},
};
#define NMODES (sizeof(modes) / sizeof(modes[0]))

static const u8 img_regs[] = {0x00, 0x04, 0x06, 0x08, 0x0A, 0x0C, 0x0E, 0x10, 0x12, 0x14, 0x16, 0x18, 0x1A, 0x1C,
                              0x1E, 0x20, 0x22, 0x24, 0x26, 0x28, 0x2A, 0x30, 0x32, 0x34, 0x36, 0x38, 0x3A, 0x3C,
                              0x3E, 0x48, 0x4A, 0x4C, 0x4E, 0x50, 0x52, 0x54, 0x56, 0x58, 0x5A, 0x5C, 0x5E, 0x60,
                              0x62, 0x64, 0x66, 0x68, 0x6A, 0x6C, 0x70, 0x72, 0x74};
#define NIMG (sizeof(img_regs))
typedef struct { u16 v[NIMG]; u16 dcr; } VIImage;

static const Mode *cur;
static VIImage safe_img, mode_img;
static u32 boot1c4, boot1c8;
static bool active;

static void delay_ms(u32 ms) {
    const u64 to = gettime() + millisecs_to_ticks(ms);
    while (gettime() < to)
        ;
}

static u16 img_mask(u8 off) {
    // display-interrupt registers: bit 15 is the interrupt status (write 0 = clear)
    return (off == 0x30 || off == 0x34 || off == 0x38 || off == 0x3C) ? 0x7FFF : 0xFFFF;
}
static void img_read(VIImage *im) {
    for (u32 i = 0; i < NIMG; i++)
        im->v[i] = VI[img_regs[i] / 2] & img_mask(img_regs[i]);
    im->dcr = VI[0x02 / 2];
}
static void img_set(VIImage *im, u8 off, u16 v) {
    for (u32 i = 0; i < NIMG; i++)
        if (img_regs[i] == off)
            im->v[i] = v;
}

static u32 pll_1c4(const Clock *k) {
    if (k->nfb == 7 && k->v == 7)
        return boot1c4;
    return (boot1c4 & ~0x0000FF00u) | ((u32)(k->nfb - 1) << 8);  // byte 2 never touched
}
static u32 pll_1c8(const Clock *k) {
    if (k->nfb == 7 && k->v == 7)
        return boot1c8;
    return (boot1c8 & ~0x00F00000u) | ((u32)k->v << 20);
}

static void build_image(const Mode *m, void *fb, VIImage *im) {
    const HTim *h = m->h;
    const VTim *v = m->v;
    const u16 HLW = h->T / 2, HSY = h->S, HBE = h->S + h->BP, HBS = HBE + h->A - HLW;
    u8 hcs = h->hcs ? h->hcs : (u8)((71u * h->T + 429) / 858);
    u8 hce = h->hce ? h->hce : (u8)((105u * h->T + 429) / 858);
    if (hce >= HBE)
        hce = HBE - 1;
    if (hcs >= hce)
        hcs = hce - 1;
    const u16 psbO = v->psb, prbO = v->N - 3 * v->equ - 2 * v->acv - v->psb;
    const u16 prbE = prbO + v->il, psbE = psbO - v->il;
    const u16 bsO = v->bs ? v->bs : 2 * v->equ, bsE = bsO + v->il;
    const u16 beO = v->be ? v->be : v->N - v->equ + v->il, beE = beO - v->il;
    *im = safe_img;
    const u16 wpl = h->A / 16, std = v->il ? wpl * 2 : wpl;
    img_set(im, 0x6C, clocks[m->clk].viclk);
    img_set(im, 0x00, (u16)((v->acv << 4) | v->equ));
    img_set(im, 0x04, (u16)((hcs << 8) | hce));
    img_set(im, 0x06, HLW);
    img_set(im, 0x08, (u16)(((HBE >> 9) & 1) | (HBS << 1)));
    img_set(im, 0x0A, (u16)(((HBE & 0x1FF) << 7) | HSY));
    img_set(im, 0x0C, psbO);
    img_set(im, 0x0E, prbO);
    img_set(im, 0x10, psbE);
    img_set(im, 0x12, prbE);
    img_set(im, 0x14, (u16)((beO << 5) | bsO));
    img_set(im, 0x16, (u16)((beO << 5) | bsO));
    img_set(im, 0x18, (u16)((beE << 5) | bsE));
    img_set(im, 0x1A, (u16)((beE << 5) | bsE));
    u32 t = MEM_VIRTUAL_TO_PHYSICAL(fb);
    u32 b = v->il ? t + (u32)h->A * 2 : t;
    u32 flag = 0;
    if (t >= 0x01000000 || b >= 0x01000000) {
        flag = 1;
        t >>= 5;
        b >>= 5;
    }
    img_set(im, 0x1C, (u16)((flag << 12) | ((t >> 16) & 0xFF)));
    img_set(im, 0x1E, (u16)(t & 0xFFFF));
    img_set(im, 0x24, (u16)((b >> 16) & 0xFF));
    img_set(im, 0x26, (u16)(b & 0xFFFF));
    img_set(im, 0x30, (u16)(0x1000 | (v->N / 2 + 1)));
    img_set(im, 0x32, (u16)(((v->N & 1) ? HLW : 0) + 1));
    img_set(im, 0x34, 0x1001);
    img_set(im, 0x36, 0x0001);
    img_set(im, 0x48, (u16)((wpl << 8) | std));
    img_set(im, 0x4A, 0x0100);  // horizontal scaler off
    im->dcr = (safe_img.dcr & 0x0300) | 0x0001 | (v->il ? 0 : 0x0004);  // FMT, ENB, NIN
}

// VI off -> reset -> clock -> settle -> full register image -> VI on
static void apply_image(const VIImage *im, u32 p4, u32 p8) {
    u32 l;
    _CPU_ISR_Disable(l);
    VI[0x02 / 2] = VI[0x02 / 2] & ~0x0001;
    _CPU_ISR_Restore(l);
    delay_ms(2);
    _CPU_ISR_Disable(l);
    VI[0x02 / 2] = 0x0002;
    _CPU_ISR_Restore(l);
    delay_ms(1);
    _CPU_ISR_Disable(l);
    VI[0x02 / 2] = 0x0000;
    const bool pll_change = (HW(0x1C4) & 0x7FFFFFFFu) != p4 || HW(0x1C8) != p8;
    HW(0x1C4) = p4;
    HW(0x1C8) = p8;
    _CPU_ISR_Restore(l);
    if (pll_change)
        delay_ms(50);
    _CPU_ISR_Disable(l);
    for (u32 i = 0; i < NIMG; i++)
        VI[img_regs[i] / 2] = im->v[i];
    VI[0x02 / 2] = im->dcr;
    _CPU_ISR_Restore(l);
}

bool vim_select(const char *name) {
    for (u32 i = 0; i < NMODES; i++)
        if (strcmp(modes[i].name, name) == 0) {
            cur = &modes[i];
            return true;
        }
    return false;
}
const char *vim_names(void) { return "480p, 1440x480p, 480p120, 480x720p, 960x720p, 1280x720p"; }
bool vim_selected(void) { return cur != NULL; }
const char *vim_desc(void) { return cur ? cur->desc : ""; }
u16 vim_width(void) { return cur ? cur->h->A : 0; }
u16 vim_height(void) { return cur ? (cur->v->il ? 2 * cur->v->acv : cur->v->acv) : 0; }
u32 vim_clock_khz(void) { return cur ? clocks[cur->clk].khz : 0; }

// Call once libogc's own (safe) mode is up: remembers it, then switches to the custom mode
void vim_enter(void *fb) {
    boot1c4 = HW(0x1C4) & 0x7FFFFFFFu;  // bit 31 is a lock-status bit set by hardware
    boot1c8 = HW(0x1C8);
    img_read(&safe_img);
    build_image(cur, fb, &mode_img);
    apply_image(&mode_img, pll_1c4(&clocks[cur->clk]), pll_1c8(&clocks[cur->clk]));
    active = true;
}

// Back to libogc's mode and the boot PLL (also the Homebrew Channel's, after exit)
void vim_leave(GXRModeObj *rmode, void *safe_fb) {
    if (!active)
        return;
    apply_image(&safe_img, boot1c4, boot1c8);
    VIDEO_Configure(rmode);
    VIDEO_SetNextFramebuffer(safe_fb);
    VIDEO_SetBlack(FALSE);
    VIDEO_Flush();
    delay_ms(50);  // not VIDEO_WaitVSync: never wait on an interrupt that might not come
    active = false;
}
