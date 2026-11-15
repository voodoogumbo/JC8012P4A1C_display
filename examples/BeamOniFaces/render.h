#pragma once
#include <stdint.h>

bool render_init();      // sprites, background, band buffers
bool render_start();     // bring up the panel and start racing the beam

void render_set_params(int faces, float speed, bool see_through);

struct RenderStats {
    float fps;            // frames scanned out per second
    float strip_avg_us;   // CPU time per strip
    float strip_max_us;
    float budget_us;      // per-strip budget with both cores
    float copy_avg_us;    // LVGL band copy share of each strip
    float flips;          // LVGL band flips per second
    uint32_t late, skipped, naps, underruns;   // totals
};
void render_get_stats(RenderStats *out);

// LVGL band (native layout, 1280 rows x BAND_H): draw into band_back(), then
// band_flip_and_wait() publishes it at a frame boundary and returns once the old
// front is no longer read. band_front() is what the strips copy from now.
uint16_t *band_back();
uint16_t *band_front();
void band_flip_and_wait();
