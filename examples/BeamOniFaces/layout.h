#pragma once
// Landscape 1280x800 as the viewer sees it. The panel's native rows are landscape
// columns (native row = 1279 - x) and native column = landscape y, so each
// 16-row strip is a 16-pixel-wide vertical slice, scanned right to left.

#define SCREEN_W     1280
#define SCREEN_H     800

#define PLAY_H       624              // landscape y 0..623: faces, drawn per strip
#define BAND_Y0      PLAY_H           // landscape y 624..799: LVGL band, copied per strip
#define BAND_H       (SCREEN_H - PLAY_H)
#define BAND_BYTES   (SCREEN_W * BAND_H * 2)

#define RING_SLOTS   8                // strip buffers; 80 strips / 8 -> 10 equal DMA blocks
                                      // (the DSI bridge assumes every block is the same size)
#define FACE_D       48
#define FACE_R       (FACE_D / 2)
#define MAX_FACES    250              // 300 still fits (419 of 481 us); 400 does not, yet
