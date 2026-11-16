// aveprobe: reads and writes the Wii's AV encoder (AVE-RVL) registers over I2C, one step at a
// time, with a test pattern and a machine-readable step code on screen, so a capture can tie
// each picture to the register state that produced it.
//
// The AVE sits at I2C address 0xE0 on Hollywood's GPIO lines (the bus libogc's video.c bit-bangs;
// the I2C primitives below follow it). Only part of its register map is known
// (wiibrew.org/wiki/Hardware/AV_Encoder); libogc writes ~15 registers at start-up. Everything
// here is volatile: the AVE is reprogrammed by every VIDEO_Init and reset by a power cycle.
//
// Arguments (wtb.txt on SD, or from the Wii test bench as cfg=name=value):
//   dump=1           try reading registers back (0xE1), 00..FF, and log what answers
//   hold=MS          how long each step stays on screen (default 3000)
//   w=RR:VV[+RR:VV]  one step: write VV (hex; 2 hex digits per byte, so 71:8e8e is 2 bytes)
//                    to register RR, several writes per step joined with '+'. Repeatable;
//                    steps run in the order given.
//   sweep=RR:A-B     one step per value A..B (hex) written to register RR
//   keep=1           don't restore libogc's encoder settings between steps
// Step 0 is always the baseline (libogc's settings). After the last step the baseline is
// restored and the tool exits to the Homebrew Channel.
//
// Screen: 8 colour bars, a 16-step grey ramp, and the step code: a white guard block, then
// 12 bits (most significant first, white = 1), then a white guard block, along the bottom.
//
// Log (ave.txt, sent at the end): one line per step with the time it went up (ms since start)
// and each write's ACK, plus the read-back dump.

#include <gccore.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fat.h>
#include <ogc/machine/processor.h>
#include <ogc/lwp_watchdog.h>

#include "wtb.h"

#define MAX_STEPS 1100
#define MAX_WRITES 8

// ---------------------------------------------------------------------------------------------
// Log: kept in memory, sent as ave.txt; each line also goes to the bench as a LOG line

static char logbuf[256 * 1024];
static size_t loglen;
static u64 t_start;

static unsigned ms_now(void) { return (unsigned)ticks_to_millisecs(gettime() - t_start); }

static void say(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
static void say(const char *fmt, ...) {
    char t[512];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(t, sizeof(t), fmt, ap);
    va_end(ap);
    if (loglen + strlen(t) + 2 < sizeof(logbuf))
        loglen += (size_t)sprintf(logbuf + loglen, "%s\n", t);
    wtb_log("%s", t);
}

// ---------------------------------------------------------------------------------------------
// I2C on Hollywood's GPIOs, as libogc's video.c does it (GPIO B out 0xC0, dir 0xC4, in 0xC8:
// bit 14 SCL, bit 15 SDA). i2c_ident: 1 when a released SDA reads high (libogc's i2cIdentFlag).

static vu32 *const gpio = (vu32 *)0xCD800000;
static u32 i2c_ident = 1;

static void delay_us(unsigned us) {
    const u64 end = gettime() + microsecs_to_ticks(us);
    while (gettime() < end)
        ;
}
static void i2c_dir(u32 sda_out) { gpio[49] = (gpio[49] & ~0x8000u) | 0x4000u | (sda_out << 15); }
static void i2c_scl(u32 v) { gpio[48] = (gpio[48] & ~0x4000u) | (v << 14); }
static void i2c_sda(bool high) { gpio[48] = (gpio[48] & ~0x8000u) | ((u32)(high ? i2c_ident : i2c_ident ^ 1) << 15); }
static bool i2c_sda_high(void) { return ((gpio[50] >> 15) & 1) == i2c_ident; }

static void i2c_check(void) {
    i2c_dir(0);
    delay_us(4);
    i2c_ident = ((gpio[50] >> 15) & 1) ? 1 : 0;
}

static void i2c_start(void) {
    i2c_dir(1);
    i2c_sda(true);
    i2c_scl(1);
    delay_us(4);
    i2c_sda(false);
    delay_us(2);
    i2c_scl(0);
}

static void i2c_stop(void) {
    i2c_dir(1);
    i2c_sda(false);
    delay_us(2);
    i2c_scl(1);
    delay_us(2);
    i2c_sda(true);
    delay_us(4);
}

// Sends a byte, returns true if the device pulled SDA low on the ninth clock (ACK)
static bool i2c_tx(u8 b) {
    i2c_dir(1);
    for (int i = 0; i < 8; i++) {
        i2c_sda(b & 0x80);
        delay_us(2);
        i2c_scl(1);
        delay_us(2);
        i2c_scl(0);
        b <<= 1;
    }
    i2c_dir(0);
    delay_us(2);
    i2c_scl(1);
    delay_us(2);
    const bool ack = !i2c_sda_high();
    i2c_scl(0);
    i2c_dir(1);
    return ack;
}

static u8 i2c_rx(bool ack) {
    u8 b = 0;
    i2c_dir(0);
    for (int i = 0; i < 8; i++) {
        delay_us(2);
        i2c_scl(1);
        delay_us(2);
        b = (u8)((b << 1) | (i2c_sda_high() ? 1 : 0));
        i2c_scl(0);
    }
    i2c_dir(1);
    i2c_sda(!ack);
    delay_us(2);
    i2c_scl(1);
    delay_us(2);
    i2c_scl(0);
    return b;
}

#define AVE_ADDR 0xE0

// Writes len bytes from reg on. Returns the number of bytes ACKed (address and register count
// as the first two), so len + 2 = all accepted.
static int ave_write(u8 reg, const u8 *data, int len) {
    u32 level;
    _CPU_ISR_Disable(level);
    int n = 0;
    i2c_start();
    if (i2c_tx(AVE_ADDR)) {
        n++;
        if (i2c_tx(reg)) {
            n++;
            for (int i = 0; i < len && i2c_tx(data[i]); i++)
                n++;
        }
    }
    i2c_stop();
    _CPU_ISR_Restore(level);
    delay_us(2);
    return n;
}

// Reads len bytes from reg: register write, repeated start, 0xE1. Returns -1 if the address
// isn't ACKed, -2 if the register isn't, -3 if the read address isn't, else len.
static int ave_read(u8 reg, u8 *out, int len) {
    u32 level;
    int r = len;
    _CPU_ISR_Disable(level);
    i2c_start();
    if (!i2c_tx(AVE_ADDR))
        r = -1;
    else if (!i2c_tx(reg))
        r = -2;
    else {
        // repeated start
        i2c_sda(true);
        delay_us(2);
        i2c_scl(1);
        delay_us(2);
        i2c_sda(false);
        delay_us(2);
        i2c_scl(0);
        if (!i2c_tx(AVE_ADDR | 1))
            r = -3;
        else
            for (int i = 0; i < len; i++)
                out[i] = i2c_rx(i + 1 < len);
    }
    i2c_stop();
    _CPU_ISR_Restore(level);
    return r;
}

// ---------------------------------------------------------------------------------------------
// The encoder settings libogc writes (video.c, __VISetupEncoder), to return to between steps

static void ave_baseline(void) {
    const vu16 *vi = (vu16 *)0xCC002000;
    const u32 dtv = vi[55] & 1;  // VI_VISEL: component cable / progressive capable
    const u32 tv = VIDEO_GetCurrentTvMode();
    const u8 region = (tv == VI_PAL || tv == VI_EURGB60) ? 2 : tv == VI_MPAL ? 1 : 0;
    static const u8 gamma[0x21] = {0x10, 0x00, 0x10, 0x00, 0x10, 0x00, 0x10, 0x00, 0x10, 0x00, 0x10,
                                   0x00, 0x10, 0x20, 0x40, 0x60, 0x80, 0xa0, 0xeb, 0x10, 0x00, 0x20,
                                   0x00, 0x40, 0x00, 0x60, 0x00, 0x80, 0x00, 0xa0, 0x00, 0xeb, 0x00};
    u8 b[0x21];
    b[0] = 1;
    ave_write(0x6a, b, 1);
    b[0] = 3;
    ave_write(0x65, b, 1);
    b[0] = (u8)((dtv << 5) | region);
    ave_write(0x01, b, 1);
    b[0] = 0;
    ave_write(0x00, b, 1);
    b[0] = 0x8e;
    b[1] = 0x8e;
    ave_write(0x71, b, 2);
    b[0] = 7;
    ave_write(0x02, b, 1);
    memset(b, 0, sizeof(b));
    ave_write(0x05, b, 2);
    ave_write(0x08, b, 2);
    ave_write(0x7a, b, 4);
    ave_write(0x40, b, 0x1a);
    ave_write(0x0a, b, 1);
    b[0] = 1;
    ave_write(0x03, b, 1);
    ave_write(0x10, gamma, sizeof(gamma));
    b[0] = 1;
    ave_write(0x04, b, 1);
    b[0] = tv == VI_EURGB60 ? 1 : 0;
    ave_write(0x6e, b, 1);
}

// ---------------------------------------------------------------------------------------------
// Picture: drawn straight into the YUY2 framebuffer

static u32 *xfb;
static GXRModeObj *rmode;

static u32 yuy2(int r, int g, int b) {
    const int y = (299 * r + 587 * g + 114 * b) / 1000;
    const int u = 128 + (-169 * r - 331 * g + 500 * b) / 1000;
    const int v = 128 + (500 * r - 419 * g - 81 * b) / 1000;
    const int y8 = 16 + y * 219 / 255, u8 = 128 + (u - 128) * 224 / 255, v8 = 128 + (v - 128) * 224 / 255;
    return (u32)(y8 << 24 | u8 << 16 | y8 << 8 | v8);
}

static void fill(int x0, int y0, int x1, int y1, u32 c) {
    const int w2 = rmode->fbWidth / 2;
    for (int y = y0; y < y1; y++)
        for (int x = x0 / 2; x < x1 / 2; x++)
            xfb[y * w2 + x] = c;
}

static void draw_pattern(void) {
    const int w = rmode->fbWidth, h = rmode->xfbHeight;
    static const int bars[8][3] = {{255, 255, 255}, {255, 255, 0}, {0, 255, 255}, {0, 255, 0},
                                   {255, 0, 255},   {255, 0, 0},   {0, 0, 255},   {0, 0, 0}};
    for (int i = 0; i < 8; i++)
        fill(w * i / 8, 0, w * (i + 1) / 8, h * 55 / 100, yuy2(bars[i][0], bars[i][1], bars[i][2]));
    for (int i = 0; i < 16; i++) {
        const int g = i * 255 / 15;
        fill(w * i / 16, h * 55 / 100, w * (i + 1) / 16, h * 75 / 100, yuy2(g, g, g));
    }
    // fine detail: 1-pixel-pair vertical lines and a horizontal line pattern
    for (int y = h * 75 / 100; y < h * 82 / 100; y++)
        for (int x = 0; x < w / 2; x++)
            xfb[y * (w / 2) + x] = (x & 1) ? yuy2(255, 255, 255) : yuy2(0, 0, 0);
    for (int y = h * 82 / 100; y < h * 86 / 100; y++)
        fill(w / 2, y, w, y + 1, (y & 1) ? yuy2(255, 255, 255) : yuy2(0, 0, 0));
}

// Step code: guard, 12 bits MSB first, guard; white = 1, on a mid-grey band
static void draw_code(unsigned n) {
    const int w = rmode->fbWidth, h = rmode->xfbHeight;
    const int y0 = h * 88 / 100, y1 = h * 97 / 100, cell = w / 16;
    fill(0, y0, w, y1, yuy2(96, 96, 96));
    fill(cell / 4, y0, cell * 3 / 4, y1, yuy2(255, 255, 255));
    for (int i = 0; i < 12; i++) {
        const bool one = (n >> (11 - i)) & 1;
        fill(cell * (i + 2) + cell / 8, y0, cell * (i + 3) - cell / 8, y1, one ? yuy2(255, 255, 255) : yuy2(0, 0, 0));
    }
    fill(cell * 14 + cell / 4, y0, cell * 15 - cell / 4, y1, yuy2(255, 255, 255));
    DCFlushRange(xfb, rmode->fbWidth * rmode->xfbHeight * 2);
}

// Holds the picture for ms, telling the bench's freeze watchdog we're alive every frame
static void hold_for(unsigned ms) {
    const u64 end = gettime() + millisecs_to_ticks(ms);
    while (gettime() < end) {
        VIDEO_WaitVSync();
        wtb_alive();
    }
}

// ---------------------------------------------------------------------------------------------
// Steps

typedef struct {
    u8 reg[MAX_WRITES];
    u8 len[MAX_WRITES];
    u8 data[MAX_WRITES][8];
    int n;
} step_t;

static step_t steps[MAX_STEPS];
static int nsteps;

static int hexval(char c) {
    if (c >= '0' && c <= '9')
        return c - '0';
    c |= 0x20;
    return (c >= 'a' && c <= 'f') ? c - 'a' + 10 : -1;
}

// "RR:VV[VV..][+RR:VV..]"
static bool parse_step(const char *s, step_t *st) {
    st->n = 0;
    while (*s && st->n < MAX_WRITES) {
        int a = hexval(s[0]), b = s[0] ? hexval(s[1]) : -1;
        if (a < 0 || b < 0 || s[2] != ':')
            return false;
        st->reg[st->n] = (u8)(a << 4 | b);
        s += 3;
        int len = 0;
        while (len < 8 && hexval(s[0]) >= 0 && hexval(s[1]) >= 0) {
            st->data[st->n][len++] = (u8)(hexval(s[0]) << 4 | hexval(s[1]));
            s += 2;
        }
        if (len == 0)
            return false;
        st->len[st->n++] = (u8)len;
        if (*s == '+')
            s++;
        else if (*s)
            return false;
    }
    return st->n > 0;
}

static void step_text(const step_t *st, char *out, size_t sz) {
    size_t o = 0;
    out[0] = 0;
    for (int i = 0; i < st->n && o + 24 < sz; i++) {
        o += (size_t)snprintf(out + o, sz - o, "%s%02X:", i ? "+" : "", st->reg[i]);
        for (int k = 0; k < st->len[i]; k++)
            o += (size_t)snprintf(out + o, sz - o, "%02X", st->data[i][k]);
    }
}

static void dump(void) {
    u8 v[1];
    say("dump: I2C ident %u (released SDA reads %s)", (unsigned)i2c_ident, i2c_ident ? "high" : "low");
    int answered = 0;
    char line[160];
    for (int row = 0; row < 256; row += 16) {
        size_t o = (size_t)snprintf(line, sizeof(line), "dump %02X:", row);
        for (int r = row; r < row + 16; r++) {
            const int res = ave_read((u8)r, v, 1);
            if (res == 1) {
                answered++;
                o += (size_t)snprintf(line + o, sizeof(line) - o, " %02X", v[0]);
            } else
                o += (size_t)snprintf(line + o, sizeof(line) - o, " %s", res == -1 ? "na" : res == -2 ? "nr" : "nd");
        }
        say("%s", line);
    }
    say("dump: %d of 256 registers answered a read (na = address not ACKed, nr = register not ACKed, "
        "nd = read address not ACKed)", answered);
}

int main(int argc, char **argv) {
    VIDEO_Init();
    rmode = VIDEO_GetPreferredMode(NULL);
    xfb = MEM_K0_TO_K1(SYS_AllocateFramebuffer(rmode));
    VIDEO_Configure(rmode);
    VIDEO_SetNextFramebuffer(xfb);
    VIDEO_SetBlack(FALSE);
    VIDEO_Flush();
    VIDEO_WaitVSync();
    if (rmode->viTVMode & VI_NON_INTERLACE)
        VIDEO_WaitVSync();
    draw_pattern();
    draw_code(0);

    const bool sd = fatInitDefault();
    wtb_init(argc, argv, "sd:/apps/aveprobe/wtb.txt", "aveprobe", __DATE__ " " __TIME__);
    if (sd)
        fatUnmount("sd:");
    t_start = gettime();
    i2c_check();

    unsigned hold = 3000;
    bool do_dump = false, keep = false;
    for (int i = 0; i < wtb_cfg_count(); i++) {
        const char *n = wtb_cfg_name(i), *v = wtb_cfg_value(i);
        if (strcmp(n, "hold") == 0)
            hold = (unsigned)atoi(v);
        else if (strcmp(n, "dump") == 0)
            do_dump = atoi(v) != 0;
        else if (strcmp(n, "keep") == 0)
            keep = atoi(v) != 0;
        else if (strcmp(n, "w") == 0 && nsteps < MAX_STEPS) {
            if (parse_step(v, &steps[nsteps]))
                nsteps++;
            else
                say("bad step '%s' (want RR:VV[+RR:VV], hex)", v);
        } else if (strcmp(n, "sweep") == 0) {
            unsigned reg, a, b;
            if (sscanf(v, "%x:%x-%x", &reg, &a, &b) == 3 && reg < 256 && a <= b && b < 256) {
                for (unsigned x = a; x <= b && nsteps < MAX_STEPS; x++) {
                    steps[nsteps].n = 1;
                    steps[nsteps].reg[0] = (u8)reg;
                    steps[nsteps].len[0] = 1;
                    steps[nsteps].data[0][0] = (u8)x;
                    nsteps++;
                }
            } else
                say("bad sweep '%s' (want RR:A-B, hex)", v);
        }
    }

    const vu16 *vi = (vu16 *)0xCC002000;
    say("aveprobe: video %ux%u%s (VI tv mode %u, VISEL %u), %d steps of %u ms%s", rmode->fbWidth,
        rmode->xfbHeight, (rmode->viTVMode & VI_NON_INTERLACE) ? "p" : "i",
        (unsigned)VIDEO_GetCurrentTvMode(), (unsigned)(vi[55] & 1), nsteps, hold, keep ? ", writes kept" : "");

    if (do_dump)
        dump();

    say("step 0 at %u ms: baseline (libogc's encoder settings)", ms_now());
    ave_baseline();
    hold_for(hold);

    for (int s = 0; s < nsteps; s++) {
        if (!keep)
            ave_baseline();
        char t[160], acks[96];
        size_t o = 0;
        step_text(&steps[s], t, sizeof(t));
        draw_code((unsigned)(s + 1));
        VIDEO_WaitVSync();
        acks[0] = 0;
        for (int i = 0; i < steps[s].n; i++) {
            const int got = ave_write(steps[s].reg[i], steps[s].data[i], steps[s].len[i]);
            o += (size_t)snprintf(acks + o, sizeof(acks) - o, "%s%d/%d", i ? "," : "", got, steps[s].len[i] + 2);
        }
        say("step %d at %u ms: %s (acks %s)", s + 1, ms_now(), t, acks);
        hold_for(hold);
    }

    ave_baseline();
    draw_code(0);
    say("restored the baseline at %u ms", ms_now());
    hold_for(1000);
    wtb_send_file("ave.txt", logbuf, loglen);
    wtb_done("ok");
    return 0;
}
