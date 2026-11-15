# beam_race: a second take on racing the beam

`beam_race` (`src/beam_race.*`, `src/beam_dpi.*`) is an independent implementation of
the idea behind this library's [raster band](README-raster.md): no framebuffer for the
picture, every 16-row chunk drawn just before the display DMA reads it. It was built
from scratch on a JC8012P4A1C, without looking at the raster band's code, to learn the
hardware, and then compared notes with it. The two arrive at the same place by
different routes, so both are kept here.

It owns the panel itself (it doesn't use `chipguy_JC8012P4A1C_display`), always covers
the whole screen, and suits landscape sketches that redraw everything every frame.
The board library supplies the I2C bus for touch.

| | Raster band (`setRasterRows`) | `beam_race` |
|---|---|---|
| Knowing which chunk is out | a block-done interrupt per chunk, `LLP` to catch merged ones | polling the DMA's live source address (`SAR`) and `LLP` from a 120 µs timer |
| Buffers | 2 to 8 chunk buffers | an 8-chunk ring (205 KB internal RAM) |
| Who draws | one top-priority task, half of each chunk handed to a helper on the other core | two tasks, one pinned to each core, claiming whole chunks in scan order |
| Mixed sources | framebuffer outside the band; LVGL strip copied in by GDMA one chunk ahead | everything drawn by the CPU (an LVGL band is copied in by the CPU) |
| DSI bridge | multi-block counting off (any number of blocks) | multi-block counting on, so 10 equal blocks of 8 chunks |

## Using it

```cpp
#include "beam_race.h"

// Once per frame, normally halfway through the previous frame. Per-frame state
// is double-buffered by (frame & 1).
static void on_frame(int32_t frame, void *ctx) { /* physics, snapshots */ }

// Fill 16 native rows x 800 px (RGB565) for strip 0..79.
static void on_strip(int strip, int32_t frame, uint16_t *dst, void *ctx) { /* draw */ }

void setup() {
    beam_race_reserve();                      // first: the ring needs one 205 KB block
    // ... your own allocations ...
    BeamRaceConfig cfg = { on_frame, on_strip, nullptr };
    beam_race_begin(cfg);
}
```

Native row r is landscape column x = 1279 − r, and native column c is landscape y,
so in landscape each strip is a 16-pixel-wide vertical slice, scanned right to left.
`beam_race_stats()` reports fps, time per strip against the budget (2 cores × 240.5 µs),
late and skipped strips, overload naps, bridge underruns and the longest `on_frame`.

## Examples

- **BeamOniFaces**: up to 250 bouncing pixel-art faces (horned skulls in twelve
  colours and oni in red, blue and green), opaque or see-through, above an LVGL
  strip with sliders, copied into every chunk. At 250 see-through faces: about 350 µs
  of 481 µs per chunk, 0 late, 0 underruns.
- **BeamSynthwaveClock**: a synthwave clock face: gradient sky, twinkling stars, a
  striped sun behind mountains, a neon grid scrolling toward you, neon 7-segment time
  (LVGL rasterizes it into a canvas once a minute), and weather on top: rain, snow or
  embers. Tap to cycle the weather. 185–222 µs per chunk, 0 late in every mode.

Both print stats on `Serial0` and take test commands there (see the top of each sketch).

## What the hardware taught us

Measured on a JC8012P4A1C with an ESP32-P4 rev v1.3, Arduino-ESP32 3.3.11.

1. **The DMA's source address is live.** `DW_GDMA.ch[n].sar0` climbs about 2 KB at a
   time during a block, about 102 bytes/µs (the panel's 1600 bytes per 14.67 µs row),
   and pauses ~490 µs in vertical blanking. With `LLP` it gives the scan position to
   better than a chunk without any per-chunk interrupt.
2. **Two aliases for one address.** `dw_gdma_link_list_get_item()` hands out list items
   at `0x8FF…`, while the channel's `LLP` register reports the same memory as `0x4FF…`.
   Compare with the top nibble masked off. Right after the channel is restarted, `LLP`
   holds item 0's own address until the fetch; after that, the next item's.
3. **Multi-block counting.** With the DMA as flow controller and the bridge's
   multi-block mode on (4-bit count), a frame must be at most 15 blocks *of equal size*:
   a short last block stalls the DMA at its start. 80 chunks split as 10 × 8. The raster
   band turns the mode off instead (count 1) and is free of both limits, which is the
   better route.
4. **Re-arm every item.** The DMA marks list items consumed; without setting each item's
   valid bit again in the frame-done interrupt, the second frame stalls.
5. **Seqlock the restart.** A reader on the other core can pair the new frame number with
   the old frame's `SAR`/`LLP` and see the scan a whole frame ahead. The frame-done
   interrupt marks itself in progress while it restarts the channel.
6. **The bridge VSYNC interrupt needs chip revision 3.0.** On v1.x it isn't there.

## What the CPU side taught us

- **Allocate the ring first.** Allocated after other big internal buffers, the 205 KB ring
  didn't fit in one block and silently fell back to PSRAM: strips took 50 % longer. Hence
  `beam_race_reserve()` and the log line saying where it landed.
- **Arduino builds at `-Os`.** Hot loops at `-O2` (`__attribute__((optimize("O2")))`),
  sprites packed one word per pixel and transparent row ends skipped took a strip from
  291 to about 220 µs.
- **Keep PSRAM traffic small.** A 4-byte-per-pixel overlay layer (16 KB of PSRAM reads
  per chunk) slowed every stage, not just its own; one byte per pixel (4 KB) fixed it.
- **The per-frame setup is on the critical path.** Run when a frame's first chunk is
  claimed, it held up both cores right where the frame starts (the right edge in
  landscape). `beam_race` now runs it halfway through the previous frame.
- **Arduino's `random()` and `sinf()` are slow** for per-frame work: `random()` reads
  the hardware RNG. Respawning particles with it stretched the frame setup to
  milliseconds (110 ms when the weather switched). Use a software PRNG and a sine table.
- **Overload starves the idle task.** Two render tasks that never block trip the task
  watchdog. After 100 ms without blocking, a task naps one tick.
- **LVGL below the renderers.** Above them, some LVGL refresh steps ran for milliseconds,
  longer than the renderers' lead, and chunks went late even at light load. Partial
  updates keep LVGL's work small enough for the renderers' idle time.
- **Blocking `printf`** from a busy task stalls it; give `Serial0` a 4 KB TX buffer.
- **The LVGL band needs no cache write-back** when only the CPUs read it (both cores
  share the L1 data cache). Flushing 450 KB per swap cost milliseconds.

## From the raster band, worth adopting next

Copying the LVGL strip by GDMA one chunk ahead, keeping the display and copy
interrupts off LVGL's core, pausing LVGL while a copy hits PSRAM, and turning the
bridge's multi-block mode off. See [README-engineering.md](README-engineering.md#the-raster-band).

## Credits

The raster band, the board support and everything else in this library are by
chipguyhere (MIT). `beam_race` and its examples were written by voodoogumbo with
Claude, and are offered under the same license.
