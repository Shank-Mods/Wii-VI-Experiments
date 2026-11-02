# VI Clock Max

Finds the highest pixel clock the Wii's Video Interface will actually run at. It climbs
the VI PLL step by step, measures the real clock at every step, drops back to the normal
video mode between steps to show and log the result, and keeps going until the VI stops
working.

## Using it

1. Launch it with a USB drive (or SD card) inserted. Results go to `viclkmax.txt`.
2. Press **A** to start. Each step blanks the picture for about 2 seconds (the test clock is
   far outside what a TV or GCVideo accepts), then the menu comes back with the result
   added to the table.
3. When it finishes it shows the highest working clock and which video modes that clock
   could drive.

**B, START, + or HOME** stops after the current step. **A** later continues where it left
off; **B** at the menu starts over.

### Sign of life

| Disc-slot LED | Meaning |
| --- | --- |
| slow blink (2/s) | at the menu / between steps |
| fast blink (8/s) | a test clock is running (screen is blank, that's expected) |
| stopped | the console hung |

### If it hangs

Every step is written to `viclkmax_state.txt` *before* the clock is changed. Power-cycle
(all clock registers reset), launch again: the menu says which clock hung, records it as
a failure, and **A** continues the sweep from there. A sweep might need a few of these
near the top.

## What a step does

1. Logs the step, then stops the VI, resets it, writes the PLL (`0x1C4` byte 1 = multiplier
   Nfb, `0x1C8` bits 20-23 = divider V; byte 2 of `0x1C4` is never written) and waits
   50 ms for it to relock.
2. Starts a fixed 1920x960 progressive raster with 2046-clock lines (the longest the VI
   allows), so the VI is also fetching a full-width line from memory.
3. Counts scan lines on the VI's beam counter (`DPV`) against the CPU timebase for 250 ms,
   twice, half a second apart. The clock is **F = 2 x 2046 x line rate**.
4. Restores the safe mode from a register image captured at boot.

A step **passes** when both measurements agree with the programmed clock within 0.3 % and
the beam counter wraps at the same line each time. Otherwise the result says why: `VI
stopped` (counter frozen), `wrong clock` (PLL didn't reach the setting, or capped), or
`unstable`.

## The sweep

Reachable clocks are F = 54 MHz x Nfb / V. Nfb stays within 7-25 (the PLL's internal VCO
at 108 x Nfb = 756-2700 MHz, the range already verified to lock), and V goes from 15 down
to 1. That covers 54 MHz to 1350 MHz. Above 337.5 MHz only V = 1-3 can reach, and those
divider values haven't been verified (the log flags them).

- **Climb:** ~6 % steps from the stock 54 MHz.
- **A failed clock** is retried once with a different Nfb/V combination for the same
  frequency, so one bad divider setting isn't mistaken for the VI's limit. A clock that
  hung is not retried.
- **Refine:** every reachable clock between the last pass and the first failure is tried.
- **Soak:** the best clock runs for 10 s with ten measurements. If it fails, the next one
  down is soaked instead.

## Reading the log (illustrative example)

```
TRY  Nfb=17 V=4  F=229.500 MHz  VCO=1836 MHz  0x1C4=002010C0 0x1C8=D840700E
  meas 1: moving  lines=14019 in 249993888 ns  line 56076.6 Hz  F=229.4658 MHz (-0.015 %)  DPV top 1000  max gap 18000 ns
  ...
  PLL lock bit 1, 140 retrace IRQs in ... us (56.07 Hz; expected 56.08 Hz), back to safe mode: yes
  RESULT OK  measured 229.4658 MHz
```

The summary at the end lists the best clock, the first failure, and a table of video
modes (720p, 1080i, 120 Hz variants, the tallest progressive/interlaced rasters the
registers allow) with the clock each needs and whether it fits.

What this does **not** check is whether the VI's memory fetch keeps up at a given clock:
the beam counter only proves the timing logic runs. Seeing a clean picture needs a display
path that accepts the signal.
