# VI Mode Test

A Homebrew Channel app for trying non-standard Wii video modes. It boots at the console's
normal safe mode (whatever `VIDEO_GetPreferredMode` returns: 480i, or 480p with component
and progressive enabled) and lists 24 test modes. Pick one, then page through the test
patterns. **START** (GameCube pad) or **HOME / +** (Wii Remote) returns to the list at the
safe mode from any point.

| Control | Menu | In a test mode |
| --- | --- | --- |
| D-pad | move (Left/Right switches column) | previous / next pattern |
| A | start the selected mode | info overlay on/off |
| START, HOME, + | HOME quits to the loader | back to the menu |

Each mode's actual line rate, field rate and frame line count are measured when you enter
it (against the CPU timebase, so this works even if the display doesn't sync) and shown in
the overlay. The menu's detail line shows them too. Everything is also logged, along with
every register value written, to `vimodetest.txt` on the USB drive (or SD).

## Modes

All timings are computed from published standards (BT.601/1358, SMPTE 274M/296M), checked
against Swiss's hardware-tested 960i and 1080i60 tables (see Credits), and the VI
PLL law in [`docs/VI_PLL.md`](../../docs/VI_PLL.md): **F = 54 MHz x Nfb / V**, pixel rate on
the bus **P = F/2**. "Line" is the horizontal total in pixels.

| Mode | Bus pixels | Clock F | Line | Line rate | Field rate | Notes |
| --- | --- | --- | --- | --- | --- | --- |
| 240p | 720x240 | 27 (stock) | 858 | 15.73 kHz | 59.83 Hz | libogc stock timing |
| 240p (double wide chroma) | 1440x240 | 54 (stock) | 1716 | 15.73 | 59.83 | 720 rendered, each pixel output twice |
| 1440x240p | 1440x240 | 54 (stock) | 1716 | 15.73 | 59.83 | 1440 distinct pixels |
| 240p 120Hz | 720x240 | 54 (stock) | 858 | 31.47 | 119.65 | |
| 480i | 720x480 | 27 (stock) | 858 | 15.73 | 59.94 | libogc stock timing |
| 480i (double wide chroma) | 1440x480 | 54 | 1716 | 15.73 | 59.94 | |
| 1440x480i | 1440x480 | 54 | 1716 | 15.73 | 59.94 | |
| 480i 120Hz | 720x480 | 54 | 858 | 31.47 | 119.88 | |
| 480p | 720x480 | 54 (stock) | 858 | 31.47 | 59.94 | libogc stock timing |
| 480p (double wide chroma) | 1440x480 | **108** | 1716 | 31.47 | 59.94 | Nfb 14, V 7 |
| 1440x480p | 1440x480 | **108** | 1716 | 31.47 | 59.94 | |
| 480p 120Hz | 720x480 | **108** | 858 | 62.94 | 119.88 | |
| 720x960i | 720x960 | 54 | 858 | 31.47 | 60.00 | field rendered, 1049 lines/frame (Swiss's 960i timing) |
| 720x960i (double wide chroma) | 1440x960 | **108** | 1716 | 31.47 | 60.00 | field rendered |
| 1280x960i | 1280x960 | **96** | 1526 | 31.45 | 59.97 | field rendered, Nfb 16, V 9 |
| 720x960i 120Hz | 720x960 | **108** | 858 | 62.94 | 119.99 | field rendered |
| 720x1080i | 720x1080 | 54 | 800 | 33.75 | 60.00 | field rendered, 1125 lines/frame (Swiss's 1080i60 timing) |
| 1440x1080i | 1440x1080 | **108** | 1600 | 33.75 | 60.00 | field rendered |
| 1920x1080i | 1920x1080 | **138** | 2046 | 33.72 | 59.95 | field rendered, Nfb 23, V 9 |
| 720x1080i 120Hz | 720x1080 | **108** | 800 | 67.50 | 120.00 | field rendered |
| 960x720p | 960x720 | **108** | 1200 | 45.00 | 60.00 | 750 lines/frame |
| 1280x720p | 1280x720 | **148.5** | 1650 | 45.00 | 60.00 | SMPTE 296M, Nfb 22, V 8 |
| 960x720i 120Hz | 960x720 | **108** | 1200 | 45.00 | 119.84 | field rendered, 751 lines/frame |
| 1280x720i 120Hz | 1280x720 | **148.5** | 1650 | 45.00 | 119.84 | field rendered |

The 4:3 1080i and 720 variants keep the same line and frame timing as their 16:9 siblings
but use fewer, wider pixels, so the picture fills the same active time.

Limits these were designed around: the half-line width register (HLW) is 10 bits, so a line
can be at most 2046 pixels. That is why 1920x1080i uses a 2046-pixel line (126 pixels of
blanking, tighter than the 280 in the broadcast standard) instead of 2200. Active lines per
field are at most 1023. In interlaced modes the framebuffer fetch stride must fit 8 bits of
16-pixel units, so 1920 wide is the maximum.

## Patterns

1. Color bars + gray ramp
2. Grid / geometry: border, 16x12 grid, center cross, colored corner blocks
3. Luma stripes, 1 / 2 / 4 px periods: horizontal resolution
4. Line pairs, 1 / 2 / 4 lines: vertical resolution and interlace flicker
5. Checkerboard, 1 px
6. Chroma: magenta/green 1 px stripes
7. Chroma: red/gray 1 px stripes of equal luma: the cleanest 4:2:2 test
8. Chroma vs luma wedge: periods 1 to 16 px, chroma on top, luma below
9. White, 10. 50% gray, 11. Black: uniformity / levels
12. Motion: a bar moves one step per VI field and a field counter is shown. Use this to
    confirm a 120 Hz mode really delivers 120 distinct images per second.

## How it works

- The VI is programmed directly: timing (VTR, HTR0/1, VTO/VTE, burst blanking, display
  interrupt), framebuffer fetch (PICCONF, scaler off, field base addresses), clock select,
  and the VI PLL (`0x1C4` byte 1 = multiplier, `0x1C8` bits 20-23 = divider). `0x1C4` byte 2
  is never written.
- libogc is left configured for the safe mode the whole time. Going back runs
  `VIDEO_Configure(safe)` plus `VIDEO_Flush`, which rewrites every register touched here, and
  the PLL is restored to its boot value first.
- Patterns are drawn with GX in tiles of at most 640x528 (the embedded framebuffer limit), so
  the console's own RGB to YUV 4:2:2 copy path is what's being tested. Field-rendered modes draw
  each field as its own half-height pass with a half-line vertical offset, into an interleaved
  framebuffer. Static patterns are double-buffered; the motion pattern renders one field per
  retrace.

## Things to report back

The overlay's "meas" line (or `vimodetest.txt`) says what the VI actually produced: line rate,
field rate, and `DPV` (the beam counter's maximum, which equals the lines in two fields).
Together with what the display shows (sync, geometry, color), that's enough to correct a mode.
If the motion pattern looks combed in a field-rendered mode, the field order is reversed;
that's a one-line fix.

## Credits

The 960i and 1080i timings follow the video mode tables in
[Swiss](https://github.com/emukidid/swiss-gc) by emu_kidid, Extrems and contributors. Swiss's
960i uses the 480p line with a 1049-half-line interlaced frame; its 1080i60 is 720 active
pixels in an 800-clock line at the stock 27 MHz pixel rate, with a 16-clock sync, an
active window of 75 to 795, a 2.5-line vsync and 29/30 + 1/0 half-lines of vertical
blanking. That matches SMPTE 274M's sync and back-porch durations more closely than the
first version of this tool did, so the 1080i modes here use it, scaled for the 1440 and 1920 variants. Only
timing values were taken; no Swiss code is included.
