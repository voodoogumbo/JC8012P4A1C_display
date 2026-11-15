// beam_dpi: JC8012 (JD9365, 800x1280 MIPI-DSI) scanout where we own the DMA.
//
// The frame is described as up to 15 equal-size DMA blocks (the DSI bridge counts
// blocks per frame in a 4-bit field and assumes they are all frame/N long, so
// 80 strips split as 2, 4, 5, 8 or 10 blocks). Each block reads a run of whole strips
// (16 native rows = 25,600 bytes) from anywhere: an SRAM ring the CPU renders
// into, or a PSRAM framebuffer. The DMA's live source address (SAR) tells us
// which strip is going out, so renderers can race it strip by strip.
#pragma once
#include <stdint.h>
#include "esp_err.h"
#include "esp_lcd_mipi_dsi.h"

#ifdef __cplusplus
extern "C" {
#endif

#define BEAM_H_RES        800                 // native row length (landscape height)
#define BEAM_V_RES        1280                // native rows (landscape width), scanned first to last
#define BEAM_STRIP_ROWS   16
#define BEAM_STRIPS       (BEAM_V_RES / BEAM_STRIP_ROWS)   // 80 per frame
#define BEAM_STRIP_PX     (BEAM_H_RES * BEAM_STRIP_ROWS)
#define BEAM_STRIP_BYTES  (BEAM_STRIP_PX * 2)
#define BEAM_MAX_BLOCKS   15

typedef struct {
    uint16_t first_strip;   // first strip of the frame this block covers
    uint16_t num_strips;    // consecutive strips, read from contiguous memory
    const void *src;        // where those strips' pixels live
} beam_block_t;

// Create the DSI bus, panel timing and DMA chain for this block layout. Does not
// start scanout. The blocks must cover strips 0..79 in order.
esp_err_t beam_dpi_init(const beam_block_t *blocks, int num_blocks);

// Send the JD9365 init sequence, start the DMA and video mode, light the backlight.
esp_err_t beam_dpi_start(void);

// Absolute index (frame * 80 + strip) of the strip the DMA is reading right now.
// Strips below this have been fully read and their memory may be reused.
int64_t beam_cur_strip(void);

uint32_t beam_frame_count(void);
uint32_t beam_underrun_count(void);
void beam_debug_trace(void);   // debug: print DMA register changes over ~25 ms

#ifdef __cplusplus
}
#endif
