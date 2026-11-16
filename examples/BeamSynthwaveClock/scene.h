#pragma once
#include <stdint.h>

// The weather, as the scene shows it. Map your weather source onto these (with
// heavy and storm for pouring rain and lightning); this example cycles them by tap.
enum Weather { WEATHER_CLEAR, WEATHER_RAIN, WEATHER_SNOW, WEATHER_FOG, WEATHER_COUNT };

bool scene_init();                                        // tables, particles, digit buffers
void scene_on_frame(int32_t frame, void *ctx);            // beam_race callbacks
void scene_on_strip(int strip, int32_t frame, uint16_t *dst, void *ctx);

// Weather changes cross-fade (the old weather thins out, the new one comes in).
// heavy: pouring rain. storm: lightning (with rain).
void scene_set_weather(int w, bool heavy = false, bool storm = false);
int scene_weather();
bool scene_heavy();
bool scene_storm();

// Night mode: eases in over ~20 s. The sun sets below the horizon, the sky goes to
// deep night, the grid dims and slows, the digits turn deep magenta, the weather
// dims. Off: the sun comes back up.
void scene_set_night(bool night);
void scene_prof_print();                                 // debug: per-stage strip timing since the last call
void scene_shoot();                                       // a shooting star, now

// Publish a new time image, rendered white-on-black into an RGB565 canvas
// (w x h used, stride in pixels). Shown from the next frame; blocks only if the
// previous image is still on screen.
void scene_set_digits(const uint16_t *canvas, int w, int h, int stride);
