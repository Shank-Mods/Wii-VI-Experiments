Chroma subsampling test - 480p + brute-force X2 (1440 wide)
===========================================================

Alternate build of chromatest with ONLY two modes:

  480P STD        standard 480p, ~720 wide on the bus (stock ~54 MHz-class clock)
  480P X2 (1440)  480p internally, every source pixel duplicated to 1440 wide

Goal: overcome YUV 4:2:2 chroma subsampling by brute force. In the X2 mode each
source pixel is output twice, so it occupies a whole 4:2:2 Cb/Cr pair on the bus
and its colour is carried intact -- an FPGA (PixelFX/GCVideo) can pair the
duplicates back into full 4:4:4.

480P X2 needs ~108 MHz of VI pixel clock (double stock 54 MHz). It is set with
the VI PLL law from this repo (see ../../docs/VI_PLL.md):

    F = 54 MHz * Nfb / V      Nfb = (0x1C4>>8 & 0xFF)+1,  V = (0x1C8>>20 & 0xF)
    108 MHz = 54 * 14/7  ->   0x1C4 = 0x00200DC0 (Nfb=14),  0x1C8 = 0xD870700E (V=7)

Horizontally it sets viWidth=1440 (libogc then frames a 1440-wide active window)
and doubles HLW 429->858 so the line holds it; 480p vertical timing is kept, so
the frame stays 59.94 Hz. The PLL is restored on mode-change-away and on exit,
and all clock registers reset on power cycle.

Controls
  D-pad Left/Right : mode (480P STD / 480P X2)
  D-pad Up/Down    : pattern P0..P5
  HOME (Wii) / START (GC pad) : exit

Patterns are the same as chromatest: 1px magenta/green, red/blue, isoluma
red/gray, a chroma-vs-luma wedge, a colour+gray reference, and a sharp chroma
edge. In 480P STD the chroma-only patterns smear toward grey; switch the same
pattern to 480P X2 and compare what your capture/FPGA shows.

Your display may briefly lose sync entering/leaving X2; that is expected. The X2
mode is the experimental one -- if the 1440 active window is cropped or offset,
that is a horizontal-timing tweak, not a clock problem.

Built from source with devkitPPC + libogc.
