# Wii VI pixel-clock PLL — register documentation and calculations

Clean-room reverse engineering of the Nintendo Wii (Hollywood) Video Interface pixel-clock
PLL, carried out by writing candidate registers on a retail console and measuring the real
VI clock against the CPU timebase. The headline result:

> **The Wii VI pixel clock is software-reprogrammable from its stock 54 MHz up to at least
> 193 MHz, by a simple, exact law:**
>
> ### F = 54 MHz × Nfb / V
>
> - **Nfb** = `((HW_0x1C4 >> 8) & 0xFF) + 1`  — feedback multiplier (stock byte = `0x06` → Nfb = 7)
> - **V**  = `(HW_0x1C8 >> 20) & 0xF`          — output divider (stock = 7)
>
> At stock, Nfb = 7 and V = 7, giving 54 MHz. Exact HD pixel clocks (74.25 MHz, 148.5 MHz)
> are reachable, and a 720p raster at 74.25 MHz has been locked by a real monitor.

Everything here was measured on hardware. All registers reset on power cycle, and nothing
written here damaged the console.

---

## 1. Register map

The clock registers live in the Hollywood register block. They are reached through the
**full-access mirror at physical `0x0D800000` (cached-bypass `0xCD800000`)** and require
`HW_AHBPROT` to be open (`<ahb_access/>` in the Homebrew Channel meta.xml; `HW_AHBPROT`
reads `0xFFFFFFFF`). The PPC-subset mirror at `0xCD0001C4/8` reads as `0`.

The Video Interface timing registers are the separate 16-bit block at `0xCC002000`.

### 1.1 Stock (boot) values on a retail Wii

| Register | Address | Boot value | Role |
| --- | --- | --- | --- |
| VICLK | VI `0x6C` | `0x0001` (54 MHz modes) | 27/54 MHz select (0 = 27, 1 = 54) |
| HW_0x1A0 | `0x0D8001A0` | `0x0C0000EE` (54 MHz) / `0x8C0000EE` (27 MHz) | bit 31 follows the 27/54 ÷2 select |
| **HW_0x1C4** | `0x0D8001C4` | **`0x002006C0`** | **VI PLL control (multiplier + output divider)** |
| **HW_0x1C8** | `0x0D8001C8` | **`0xD870700E`** | **VI PLL "EXT" (output divider V + coarse divider)** |
| HW_0x1C0 | `0x0D8001C0` | `0xD8000000` | (unmapped; unchanged across all VI modes) |
| HW_0x1CC | `0x0D8001CC` | `0x02587FCE` | (unmapped; identical to HW_0x1AC) |

A read-only probe (`tools/vi_probe`) first established that switching between stock 27 MHz
and 54 MHz modes changes **only** `HW_0x1A0` bit 31 and leaves `0x1C4`/`0x1C8` untouched —
i.e. 54 MHz is the *undivided* output of a fixed PLL, and going faster means reprogramming
that PLL. That is what `tools/vi_pll` then did.

---

## 2. HW_0x1C4 — VI PLL control

Decoded by perturbing one nibble at a time and measuring the resulting clock (run 1), then
sweeping the multiplier across its full range (run 2).

```
 bit 31                     23      15       7       0
 +------+-------------------+-------+--------+--------+
 | lock |   byte2 (0x20)    | byte1 | byte0 (divider) |
 +------+-------------------+-------+--------+--------+
   31    bits 16-23          8-15    0-7
```

| Field | Bits | Stock | Behaviour |
| --- | --- | --- | --- |
| **Feedback multiplier** | 8–15 (byte1) | `0x06` | **Nfb = byte1 + 1.** Clock is linear in Nfb over the whole tested range (Nfb 7…25 → 54…193 MHz). This is the primary clock knob. |
| **Output divider** | 0–7 (byte0) | `0xC0` | Post-divider. `0xC0` = ÷1 (stock), `0xC1` = ÷2 (27 MHz), `0xC2` = ÷3 (18 MHz). Low-nibble values ≥ `0x4` lost lock in the base config. Bits 6–7 act as a small fine trim (~2 MHz/bit at Nfb 7): `0x80`→52.1, `0x40`→50.1, `0x00`→48.2 MHz. |
| **byte2 — DO NOT WRITE** | 16–23 | `0x20` | **Writing `0x21` (byte2 += 1) HARD-HANGS the console** (CPU/bus lock, recoverable only by power cycle). This field feeds a shared/critical clock. Keep byte2 = `0x20` at all times. |
| **PLL lock status** | 31 | set by HW | Read-only status; the hardware sets it after the PLL relocks. Writes to it do not stick. |

### Feedback-multiplier linearity (run 2, V = 7, byte0 = `0xC0`)

| byte1 | Nfb | Predicted 54·Nfb/7 (kHz) | Measured (kHz) |
| ---: | ---: | ---: | ---: |
| 0x06 | 7 | 54 000 | 54 000 |
| 0x07 | 8 | 61 714 | 61 714 |
| 0x08 | 9 | 69 429 | 69 430 |
| 0x09 | 10 | 77 143 | 77 142 |
| 0x0A | 11 | 84 857 | 84 862 |
| 0x0C | 13 | 100 286 | 100 292 |
| 0x0E | 15 | 115 714 | 115 722 |
| 0x10 | 17 | 131 143 | 131 146 |
| 0x11 | 18 | 138 857 | 138 868 |
| 0x12 | 19 | 146 571 | 146 584 |
| 0x13 | 20 | 154 286 | 154 294 |
| 0x14 | 21 | 162 000 | 162 008 |
| 0x16 | 23 | 177 429 | 177 436 |
| 0x18 | 25 | 192 857 | 192 862 |

The fit is exact to within the measurement's integer-nanosecond quantisation. The clock did
not lose lock anywhere up to 193 MHz.

---

## 3. HW_0x1C8 — VI PLL "EXT" (output divider)

Decoded by nibble perturbation (run 2) and a dedicated divider sweep (run 3).

```
 bit 31      27      23      19                     0
 +-----------+-------+-------+----------------------+
 |           | coarse|   V   |   (no effect)        |
 +-----------+-------+-------+----------------------+
              24-27   20-23   0-19
```

| Field | Bits | Stock | Behaviour |
| --- | --- | --- | --- |
| **Output divider V** | 20–23 | `7` | **F = 54 · Nfb / V.** Linear divider, verified V = 4…10. At stock Nfb (7), F = 378/V MHz. Smaller V → higher clock (V = 4 stable). |
| Coarse divider | 24–27 | `8` | Incrementing it (`8`→`9`) dropped the clock to ~16.4 MHz — a much larger divide. Not used for HD. |
| (no effect) | 0–19, 28–31 | — | Perturbing these did not change the VI clock in the tested range. |

### V divider verified (run 3, Nfb = 7)

| HW_0x1C8 | V | Predicted 378/V (kHz) | Measured (kHz) |
| --- | ---: | ---: | ---: |
| `0xD840700E` | 4 | 94 500 | (via 148.5 target, see §4) |
| `0xD850700E` | 5 | 75 600 | 75 600 |
| `0xD860700E` | 6 | 63 000 | 63 000 |
| `0xD870700E` | 7 | 54 000 | 54 000 (stock) |
| `0xD880700E` | 8 | 47 250 | 47 250 |
| `0xD890700E` | 9 | 42 000 | 42 000 |
| `0xD8A0700E` | 10 | 37 800 | 37 800 |

---

## 4. The clock law and HD targets

Combining the two knobs:

### **F = 54 MHz × Nfb / V**,  Nfb = `((0x1C4>>8)&0xFF)+1`,  V = `(0x1C8>>20)&0xF`

Because both knobs are integers, HD pixel clocks factor cleanly:

| Target | Factorisation | Nfb | V | `0x1C4` | `0x1C8` | Measured |
| --- | --- | ---: | ---: | --- | --- | --- |
| 54 MHz (stock) | 54 × 7/7 | 7 | 7 | `0x002006C0` | `0xD870700E` | 54 000 kHz |
| **74.25 MHz** (720p / 1080i) | 54 × 11/8 | 11 | 8 | `0x00200AC0` | `0xD880700E` | **74 250 kHz (exact)** |
| **148.5 MHz** | 54 × 22/8 | 22 | 8 | `0x002015C0` | `0xD880700E` | **148 512 kHz (+0.008%)** |
| **148.5 MHz** (alt) | 54 × 11/4 | 11 | 4 | `0x00200AC0` | `0xD840700E` | **148 506 kHz (+0.004%)** |

The ±12 kHz on 148.5 MHz is integer-nanosecond rounding in the measurement, not the clock.

> **Why 148.5 MHz matters (Extrems' figure):** a progressive HD pixel stream of 74.25 Mpx/s
> on a 2-byte-per-pixel digital bus that moves one byte per clock needs a **148.5 MHz byte
> clock**. 74.25 MHz is the pixel rate; 148.5 MHz is the bus rate for the same picture.

---

## 5. Horizontal timing relation

The VI horizontal counter (and the `HTR0` half-line-width field `HLW`) runs at **half** the
effective pixel clock. Anchored to the measured 1280×240p mode (HLW = 858, F = 54 MHz →
63 553 ns line):

### **line_ns = 4 × HLW × 10⁶ / F_kHz**

- `HLW` is `VI[0x06] & 0x3FF` — **10-bit, max 1023.**
- To hit a 45 kHz (22 222 ns) 720p line at F = 74.25 MHz: `HLW = 22222 × 74250 / 4e6 ≈ 413`.
- Confirmed: HLW = 413 at 74.25 MHz measured **44.94 kHz** (720p wants 45.00).

**HTR0 10-bit limit and the resolutions it allows at 74.25 MHz:**

| Mode | Line rate | htotal | HLW needed | Fits 10-bit? |
| --- | --- | ---: | ---: | --- |
| 720p60 | 45 kHz | 1650 | 413 | yes |
| 1080i | 33.75 kHz | 2200 | 550 | yes* |
| 1080i (half-line field) | — | — | 1100 | **no** (overflows 1023) |

\* 1080i's line rate fits HLW, but its 2200-wide htotal in half-line field terms (1100)
overflows the 10-bit field, so 1080i needs a different timing approach than a naive htotal.
720p60 is the natural first real-signal target.

---

## 6. Measurement method

The VI clock is measured **without a working display**, because the CPU timebase is clocked
independently of the VI:

1. **Field rate** — count VI retrace interrupts over ~120 fields, timed with `gettime()`.
2. **Line period** — time the VI vertical beam counter (`VI[0x2C]`, which counts half-lines)
   crossing from line 40 to line 200, best of 8, timed with `gettime()`.
3. **Implied clock** — `counter_kHz = 2 × HLW × 10⁶ / line_ns`; the tool reports the effective
   pixel clock as `2 × counter_kHz` (the stock 54 MHz convention).
4. **Total lines** — the maximum beam line seen (`maxline`) ÷ 2 (half-lines → lines).

All measurement loops are bounded by the timebase, so a stalled VI is reported, never hung on.

---

## 7. 720p result (vi720test)

At **74.25 MHz**, HLW = 413, progressive, VTR ACV = 720, the VI produced a raster that a
real monitor **locked onto as 720p**:

| Metric | Measured | 720p60 ideal |
| --- | --- | --- |
| Line rate | 44.94 kHz | 45.00 kHz |
| Field rate | 58.52 Hz | 60.00 Hz |
| Total lines | 768 | 750 |
| Pixel clock | 74 252 kHz | 74 250 kHz |

The field rate is low because the frame came out 768 lines instead of 750 (vertical blanking
to be trimmed by ~18 lines); the monitor accepted it regardless.

**Open issue:** the picture was a green/magenta mess — the signature of a chroma/YUYV phase
or sampling error. It is not yet known whether this originates on the Wii (horizontal pixel
phase at the new clock) or in the GCVideo digitiser, which is only rated to 54 MHz. A
grayscale (chroma-neutral) test is the decisive discriminator and is the current step.

---

## 8. Safety

- **Never write `0x1C4` byte2 to anything but `0x20`** — `0x21` hard-hangs the console.
- All of `0x1C4`, `0x1C8`, `0x1A0` reset to boot values on power cycle.
- `0x1A0` bit 31 is an inert status mirror, not a usable divide control.
- The tools use **write-ahead logging**: each intended register write is flushed to the USB
  log *before* the poke, so if a value hangs the console the log names the exact write.
- Bad VI *timing* only produces a bad/absent picture; it does not hang.

---

## 9. Open questions / next steps

1. Fix the 720p **colour** (determine Wii chroma phase vs GCVideo >54 MHz undersampling).
2. Trim vertical blanking to land exactly 750 lines / 60.00 Hz.
3. Characterise `0x1C4` byte0 (output divider) encoding fully, and `0x1C8` bits 24–27.
4. Confirm the law on other consoles (GameCube-mode Wii, PAL, vWii, board revisions).
5. Identify whether the `0x1C4`/`0x1C8` PLL is shared with any other block (byte2 hang
   suggests at least one adjacent field is), to bound the risk envelope.
6. 720p scanout from a real framebuffer (XFB width/stride, scaler) for an actual picture.
