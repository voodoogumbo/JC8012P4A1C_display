// beam_race: render the JC8012 screen one 16-row strip at a time, just ahead of
// the display DMA, on both cores. No framebuffer: the DMA reads an 8-strip ring
// in internal RAM (10 equal blocks per frame) and two tasks refill it in scan order.
//
// Native layout: 800 x 1280, rows scanned first to last. Mounted landscape, native
// row r is landscape column x = 1279 - r and native column c is landscape y, so a
// strip is a 16-pixel-wide vertical slice and the scan runs right to left.
#pragma once
#include <stdint.h>
#include "beam_dpi.h"

struct BeamRaceConfig {
    // Once per frame, before any strip of that frame is drawn, on one render core:
    // normally halfway through the previous frame, while its strips are still
    // drawing. Keep per-frame state double-buffered by (frame & 1); when it runs,
    // every strip of frame-2 (the buffer being reused) is finished.
    void (*on_frame)(int32_t frame, void *ctx);
    // Fill dst (16 rows x 800 px RGB565, row-major) for strip 0..79 of frame.
    void (*on_strip)(int strip, int32_t frame, uint16_t *dst, void *ctx);
    void *ctx;
};

struct BeamRaceStats {
    float fps;            // frames scanned out per second
    float strip_avg_us;   // CPU time per strip (render + cache flush)
    float strip_max_us;
    float budget_us;      // per-strip budget with both cores: 2 x strip period
    uint32_t late;        // strips finished after the DMA reached them (total)
    uint32_t skipped;     // strips abandoned because the DMA was already there (total)
    uint32_t naps;        // overload pauses that let idle tasks and LVGL run (total)
    uint32_t underruns;   // DSI bridge FIFO underruns (total)
    float frame_max_us;   // longest on_frame call
};

// Allocate the 205 KB strip ring in internal RAM. It needs one contiguous block,
// so call this first thing in setup(), before other large internal allocations;
// beam_race_begin() calls it if you didn't. Falls back to PSRAM (much slower,
// logged) if internal RAM is too fragmented.
bool beam_race_reserve();

// Bring up the panel and start racing. Call once from setup().
bool beam_race_begin(const BeamRaceConfig &cfg);

// Newest frame whose on_frame has run. Every strip of frames below (this - 1)
// is finished, which is how callers know a buffer they swapped out is free.
int32_t beam_race_frame_done();

// Stats since the previous call.
void beam_race_stats(BeamRaceStats *out);
