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
| Buffers | 2 to 8 chunk buffers | an 8-chunk ring (205 KB, internal RAM, or PSRAM next to Wi-Fi) |
| Who draws | one top-priority task, half of each chunk handed to a helper on the other core | two tasks, one pinned to each core, claiming whole chunks in scan order; a third sets up each frame in their idle time |
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
                                              // (beam_race_reserve(false) with Wi-Fi: see below)
    // ... your own allocations ...
    BeamRaceConfig cfg = { on_frame, on_strip, nullptr };
    beam_race_begin(cfg);
}
```

`on_frame` runs in a frame task (priority 23, core 1) while the renderers (priority 24,
one per core) wait for the beam. Don't use PIE in it. `beam_pie.h` has the PIE
helpers for `on_strip`: an RGB565 blend, a masked copy, a row fill into 16 rows (or
into every 4th row, for dithering), and an aligned copy. `beam_dpi_set_base()`
retargets a framebuffer-scanning layout at the next frame boundary, for tear-free
double buffering.

Native row r is landscape column x = 1279 − r, and native column c is landscape y,
so in landscape each strip is a 16-pixel-wide vertical slice, scanned right to left.
`beam_race_stats()` reports fps, time per strip against the budget (2 cores × 240.5 µs),
late and skipped strips, overload naps, bridge underruns and the longest `on_frame`.

## Examples

- **BeamOniFaces**: up to 1000 bouncing pixel-art faces (horned skulls in twelve
  colours and oni in red, blue and green), opaque or see-through, above an LVGL strip
  with sliders, copied into every chunk. The blend, background and band copy use PIE,
  checked against plain C at boot; `p` on serial switches between the two. 1000
  see-through faces: 408 of 481 µs per chunk, 0 late. In plain C, 400 didn't fit.
- **BeamSynthwaveClock**: a synthwave clock face: dithered gradient sky, twinkling
  stars, a striped sun behind mountains, a neon grid scrolling toward you, neon
  7-segment time (LVGL rasterizes it into a canvas once a minute). The weather plays
  out in chunky 8-bit pixels: rain that splashes on the grid, pouring rain, storms
  with forked lightning, snow, fog banks, or shooting stars when it's clear. The
  digits take the weather's colours. Night mode sets the sun and dims everything.
  Tap to cycle the weather. 150–185 µs per chunk, 0 late in every mode. A networked
  version (MQTT weather and time, OTA) runs a kitchen clock at 280–320 µs with Wi-Fi up.

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

## Next to Wi-Fi

The JC8012's Wi-Fi is an ESP32-C6 on SDIO, and the P4 side of that link wants its
own RAM and CPU. What it took to race the beam with Wi-Fi, MQTT over TLS and OTA up:

1. **The ring can't be in internal RAM.** With the 205 KB ring there, the Wi-Fi link
   can't allocate its buffers, asserts (`sdio_mempool_create: no mem`) and the board
   reboot-loops. `beam_race_reserve(false)` puts the ring in PSRAM. Tables only
   `on_frame` reads can go there too. Even so, internal RAM bottoms out around 45 KB
   once TLS is up.
2. **A PSRAM ring has to fit the L2 cache** (256 KB on this core). At 8 chunks
   (205 KB) strips cost about what they did in internal RAM. At 16 chunks (409 KB),
   tried for more lead, every chunk had to be fetched back from PSRAM before it was
   drawn, and strips went from ~290 µs to ~495 µs.
3. **The renderers have to outrank the link.** Its tasks (`sdio_read`, `sdio_write`,
   `sdio_rx_buf`, `sdio_process_rx`, `rpc_rx`, `rpc_tx`) run at priority 23 on either
   core. At 22, every Wi-Fi burst preempted a renderer mid-chunk. At 23, time slicing
   still handed them whole 1 ms ticks. At 24, Wi-Fi runs whenever a renderer is
   waiting for the beam (about a third of each core), and the overload nap still
   gives it a tick every 100 ms. (`tasks` on the networked clock's serial port lists
   them all with priority and core.)
4. **The per-frame setup comes off the renderers.** Run inline, a 2 ms `on_frame`
   stopped one core drawing and cost several chunks a frame. A frame task one level
   below the renderers does it in their idle time. At equal priority it time-sliced
   in mid-chunk instead.
5. **Blank the backlight for OTA.** Flash writes stall the renderers along with
   everything else running from flash, so the panel shows garbage for the upload.

## PIE

PIE is the P4's 128-bit SIMD extension: eight RGB565 pixels per instruction.
Measured in BeamOniFaces at 400 faces:

| | plain C | PIE |
|---|---|---|
| see-through faces, whole chunk | 577 µs (over budget) | 192 µs |
| the face blend | 482 µs (~22 cycles per pixel) | 137 µs (~6) |
| background fill | 51 µs | 14 µs |
| LVGL band copy from PSRAM | 38 µs | ~30 µs (PSRAM-bound) |

- **The blend.** Sprites are stored premultiplied by the opacity (20/32), so each
  pixel is background × 12/32 per channel plus the sprite. R and B scale in one
  multiply (their products don't overlap) and G in another. A zero sprite pixel is
  transparent: a compare builds the mask, and the result is
  `blend ^ ((blend ^ bg) & mask)`.
- **Row ends.** Sprite rows are padded with transparent pixels, so a row is always
  drawn as whole aligned groups of 8 without edge cases.
- **Unaligned sources.** `esp.ld.128.usar` + `esp.src.q` funnel-shift two aligned
  loads into the 8 pixels at any address.

What the hardware does, found the hard way:

- **Only some registers work.** Address and scalar operands must be in x8..x15
  (`s0`, `s1`, `a0`..`a5`). `t0` or `s7` won't assemble, so pin C variables with
  `register ... asm("a0")`.
- **Loads ignore the low 4 address bits.** A buffer that isn't 16-byte aligned gives
  wrong results with no error. Our first background buffer did exactly that.
- **`esp.vadd.u16` saturates.** A random-data self-test "failed" on it while real
  premultiplied pixels never overflow. Test with data shaped like the real thing.
- **`esp.vmul.u16` keeps the full product** and shifts it right by SAR
  (`esp.movx.w.sar`) before taking the low 16 bits.
- **Self-test at boot.** Run every vector path against the C code at all 8
  alignments, and fall back to C on any mismatch.
- **The context-switch bug.** Arduino core 3.3.11/3.3.12 (ESP-IDF v5.5.5) saves PIE
  state wrongly on a context switch: q3 isn't saved and the SAR fields get swapped.
  It's fixed after IDF v5.5.6 (`git cherry-pick 9d09290782 51d5738ad769` for older
  v5.5). FreeRTOS switches lazily, saving only when a second task on the same core
  touches PIE, so the rule is one PIE task per core: the two renderers, nothing in
  `on_frame`, LVGL with `LV_DRAW_SW_ASM_NONE`. Two PIE tasks on one core show up as
  white speckles.

## Smaller lessons

- **Dither gradients.** RGB565 bands badly in dark gradients, worst in a night sky.
  Build the y-only background column in RGB888, quantize it four times with an
  ordered-dither row (4×4 Bayer), and fill chunk rows `r % 4` from column `r % 4`
  (`pie_fill_rows`). Cache the result while its inputs hold still.
- **Sort sprites by chunk once per frame.** Narrow sprites (16 px or less) are
  counting-sorted by the chunk their left edge falls in, so chunk k only looks at
  the lists for k and k+1. Without that, ~1000 raindrops were scanned 80 times a
  frame.
- **Cycle sums overflow 32 bits** in about 10 s at this load, so stats keep them as
  64-bit.
- **Framebuffer plus edits.** For an LVGL UI with sparse effects, the DMA can scan
  an LVGL framebuffer straight from PSRAM (`beam_dpi_set_base()` flips it at a frame
  boundary) while an edit task trails the beam. As each chunk is read, the task puts
  back the background under the last frame's effects and draws the next frame's, a
  whole frame ahead. Effects only replace background-coloured pixels, so they pass
  behind text and putting back is exact. It cost 1–3 ms of one core per frame, with
  no ring and no chunk copies.

## From the raster band, worth adopting next

Copying the LVGL strip by GDMA one chunk ahead, keeping the display and copy
interrupts off LVGL's core, pausing LVGL while a copy hits PSRAM, and turning the
bridge's multi-block mode off. That last one would also allow a 4-chunk ring, half
the RAM, if the lead holds up. See [README-engineering.md](README-engineering.md#the-raster-band).

## Credits

The raster band, the board support and everything else in this library are by
chipguyhere (MIT). `beam_race` and its examples were written by voodoogumbo with
Claude, and are offered under the same license.
