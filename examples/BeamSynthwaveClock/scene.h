#pragma once
#include <stdint.h>

enum Weather { WEATHER_CLEAR, WEATHER_RAIN, WEATHER_SNOW, WEATHER_EMBERS, WEATHER_COUNT };

bool scene_init();                                        // tables, particles, digit buffers
void scene_on_frame(int32_t frame, void *ctx);            // beam_race callbacks
void scene_on_strip(int strip, int32_t frame, uint16_t *dst, void *ctx);

void scene_set_weather(int w);
int scene_weather();

// Publish a new time image, rendered white-on-black into an RGB565 canvas
// (w x h used, stride in pixels). Shown from the next frame; blocks only if the
// previous image is still on screen.
void scene_set_digits(const uint16_t *canvas, int w, int h, int stride);
