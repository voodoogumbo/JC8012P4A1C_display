// Synthwave clock scene, drawn one 16-column strip at a time.
//
// Back to front: sky gradient, twinkling stars, shooting stars and lightning, a
// striped sun, a mountain silhouette, a neon grid (horizontal lines scroll toward
// you, vertical lines converge on the horizon), the time in neon digits, then the
// weather in chunky 8-bit pixels: rain with splashes on the grid, snow, fog banks.
//
// Everything that depends only on y (sky, ground, scrolling grid lines, fog haze,
// sun stripes) is baked once per frame into one 800-pixel column, so each strip
// starts as that column copied into 16 rows (PIE) and only the x-dependent parts
// are drawn per pixel. Night mode and fog change the per-frame colour tables, not
// the strip code.

#include <Arduino.h>
#include <math.h>
#include "esp_heap_caps.h"
#include "esp_cpu.h"
#include "beam_race.h"
#include "beam_pie.h"
#include "scene.h"

#define W 1280
#define H 800
#define HORIZON 470
#define VPX 640                   // vanishing point x
#define SUN_CX 640
#define SUN_CY 300
#define SUN_R 200
#define SUN_SET (HORIZON - SUN_CY + SUN_R + 8)   // how far the sun sinks at night
#define DIGIT_CY 300              // time image centred here (landscape)
#define DIG_MAXW 1200
#define DIG_H 250
#define SHADOW 7                  // drop shadow offset, px right and down
#define NSTARS 220
#define PX 3                      // one 8-bit weather pixel

static const float FRAME_DT = 880.0f * 1312.0f / 60e6f;   // 19.24 ms
static const float NIGHT_FADE_S = 20;
static const float WEATHER_OUT_S = 1.5f, WEATHER_IN_S = 2.5f;

// ---------------------------------------------------------------- colour helpers

#define MASK 0x07E0F81Fu
static inline uint32_t ex(uint32_t c) { return (c | (c << 16)) & MASK; }
static inline uint16_t pk(uint32_t e) { return (uint16_t)(e | (e >> 16)); }
static inline uint16_t blend_e(uint32_t fe, uint16_t bg, uint32_t a) {   // a 0..32
    uint32_t b = ex(bg);
    return pk((b + (((fe - b) * a) >> 5)) & MASK);
}
static inline uint32_t lerp_e(uint32_t ae, uint32_t be, uint32_t a) {    // a 0..32, spread form
    return (ae + (((be - ae) * a) >> 5)) & MASK;
}
static inline uint16_t add_e(uint16_t bg, uint32_t le) {                  // saturating add
    uint32_t s = ex(bg) + le, o = s & 0x08010020u;
    uint32_t sat = ((o & 0x10020u) - ((o & 0x10020u) >> 5)) | ((o & 0x8000000u) - ((o & 0x8000000u) >> 6));
    return pk((s | sat) & MASK);
}
static inline uint32_t scale_e(uint32_t e, uint32_t w) { return ((e * w) >> 5) & MASK; }   // w 0..32

static uint16_t rgb(uint32_t c) {
    return (((c >> 16) & 0xF8) << 8) | (((c >> 8) & 0xFC) << 3) | ((c & 0xF8) >> 3);
}
static uint32_t mix(uint32_t a, uint32_t b, float t) {
    t = t < 0 ? 0 : (t > 1 ? 1 : t);
    int r = ((a >> 16) & 255) + (((int)((b >> 16) & 255) - (int)((a >> 16) & 255)) * t);
    int g = ((a >> 8) & 255) + (((int)((b >> 8) & 255) - (int)((a >> 8) & 255)) * t);
    int bl = (a & 255) + (((int)(b & 255) - (int)(a & 255)) * t);
    return (r << 16) | (g << 8) | bl;
}
static uint32_t ramp(const uint32_t *stops, int n, float t) {   // evenly spaced colour stops
    t = t < 0 ? 0 : (t > 1 ? 1 : t);
    float f = t * (n - 1);
    int i = (int)f;
    if (i >= n - 1) return stops[n - 1];
    return mix(stops[i], stops[i + 1], f - i);
}
static inline uint32_t lerp8(uint32_t a, uint32_t b, uint32_t t) {   // RGB888, t 0..32
    int r = (a >> 16) & 255, g = (a >> 8) & 255, bl = a & 255;
    r += ((((int)(b >> 16) & 255) - r) * (int)t) >> 5;
    g += ((((int)(b >> 8) & 255) - g) * (int)t) >> 5;
    bl += ((((int)b & 255) - bl) * (int)t) >> 5;
    return (r << 16) | (g << 8) | bl;
}
static inline uint32_t add8(uint32_t a, uint32_t b, uint32_t t) {    // a + b * t/32, saturating
    int r = ((a >> 16) & 255) + ((((b >> 16) & 255) * t) >> 5);
    int g = ((a >> 8) & 255) + ((((b >> 8) & 255) * t) >> 5);
    int bl = (a & 255) + (((b & 255) * t) >> 5);
    return ((r > 255 ? 255 : r) << 16) | ((g > 255 ? 255 : g) << 8) | (bl > 255 ? 255 : bl);
}
// RGB888 -> RGB565 with an ordered-dither threshold b (0..15): smooth gradients
// instead of 16-bit bands, most visible in the dark night sky
static inline uint16_t dither565(uint32_t c, int b) {
    int r = (((c >> 16) & 255) + (b >> 1)) >> 3, g = (((c >> 8) & 255) + (b >> 2)) >> 2, bl = ((c & 255) + (b >> 1)) >> 3;
    return ((r > 31 ? 31 : r) << 11) | ((g > 63 ? 63 : g) << 5) | (bl > 31 ? 31 : bl);
}
static const uint8_t BAYER[4][4] = { { 0, 8, 2, 10 }, { 12, 4, 14, 6 }, { 3, 11, 1, 9 }, { 15, 7, 13, 5 } };

static inline uint32_t a32(float f) { return f <= 0 ? 0 : f >= 1 ? 32 : (uint32_t)(f * 32 + 0.5f); }

// Fast PRNG for particles and stars. Arduino's random() reads the hardware RNG,
// which is slow here: respawning raindrops with it stretched the per-frame setup
// to milliseconds (110 ms when the weather switched), and strips went late.
static uint32_t s_rng = 0x9E3779B9u;
static inline uint32_t rng() { s_rng ^= s_rng << 13; s_rng ^= s_rng >> 17; s_rng ^= s_rng << 5; return s_rng; }
static inline int rnd(int n) { return n > 0 ? (int)(rng() % (uint32_t)n) : 0; }
static inline int rnd(int lo, int hi) { return lo + rnd(hi - lo); }
static inline float frnd(float lo, float hi) { return lo + (hi - lo) * (rng() & 0xFFFF) / 65535.0f; }

// Table sine for the per-frame setup (stars, sway): sinf() on ever-growing time
// arguments kept the per-frame work in the milliseconds.
static float s_sin[256];
static inline float fsin(float x) { return s_sin[(int32_t)(x * (256.0f / 6.2831853f)) & 255]; }

// ---------------------------------------------------------------- weather particles, cold tables

#define NDROPS 470
#define NFLAKES 260
#define NPUFFS 46
struct Drop { float x, y, vy, splash; int16_t ty; uint8_t len, a; };
struct Flake { float x0, y, vy, amp, freq, ph; uint8_t size, a; };
struct Puff { float x, v; int16_t y; uint8_t w, a; };

struct Cold {
    uint32_t sky_d[HORIZON], sky_n[HORIZON];       // RGB888: dithered down to RGB565 per frame
    uint32_t ground_d[H], ground_n[H];             // RGB888, [HORIZON..H) used
    uint32_t digpal[WEATHER_COUNT][DIG_H];         // digit gradient per weather, spread form
    uint16_t sky_cache[4][HORIZON];                // the dithered sky, while night/fog/flash hold still
    uint16_t gnd_cache[4][H];                      // the dithered ground under the grid lines, likewise
    uint32_t cache_key;                            // (night, fog, flash) they were made for
    uint16_t star_lut[33];                         // star brightness 0..32 -> colour
    uint32_t dig_night[DIG_H];
    uint32_t suncol_e[H];
    float fog_prof[H];                             // fog haze by height: thickest at the horizon
    float gz[H], gpix[H], gfade[H];                // ground rows: depth, depth per pixel, line fade
    Drop drop[NDROPS];
    Flake flake[NFLAKES];
    Puff puff[NPUFFS];
    int nwx_tmp;
    struct { int16_t x, y; uint8_t w, h, a; uint16_t c; } wx_tmp[1150];   // the weather, before sorting by strip
};
static Cold *C;
#define s_sky_d (C->sky_d)
#define s_sky_n (C->sky_n)
#define s_ground_d (C->ground_d)
#define s_ground_n (C->ground_n)
#define s_suncol_e (C->suncol_e)
#define s_fog_prof (C->fog_prof)
#define s_gz (C->gz)
#define s_gpix (C->gpix)
#define s_gfade (C->gfade)
#define s_drop (C->drop)
#define s_flake (C->flake)
#define s_puff (C->puff)

// ---------------------------------------------------------------- static scene
// Day and night versions of every colour table, spread form, blended per frame.

// Tables only the once-a-frame setup reads live in PSRAM (struct Cold, above):
// internal RAM is Wi-Fi's (its SDIO buffers fail to allocate with ~45 KB less).
static uint32_t s_fog_e;                           // fog colour
static int16_t s_sun_h[W];                         // sun half-height at each x (0 = no sun)
static int16_t s_mount[W];                         // mountain height above the horizon
static uint32_t s_mount_d[128], s_mount_n[128];
static uint32_t s_rim_d, s_rim_n, s_line_d, s_line_n;
#define CAMZ 1555.0f
static int16_t s_sx[NSTARS], s_sy[NSTARS];
static float s_sbase[NSTARS], s_sspeed[NSTARS], s_sphase[NSTARS];

static void build_static() {
    const uint32_t sky_d[] = { 0x07021A, 0x1C0840, 0x4A1260, 0xA0207A };
    const uint32_t sky_n[] = { 0x010006, 0x03010E, 0x08031C, 0x16062E };
    for (int y = 0; y < HORIZON; y++) {
        float t = y / (float)(HORIZON - 1);
        s_sky_d[y] = ramp(sky_d, 4, t);
        s_sky_n[y] = ramp(sky_n, 4, t);
    }
    for (int y = HORIZON; y < H; y++) {
        float t = (y - HORIZON) / (float)(H - HORIZON);
        s_ground_d[y] = mix(0x1E0838, 0x040110, t);
        s_ground_n[y] = mix(0x0A0318, 0x010006, t);
    }
    const uint32_t sun[] = { 0xFFF27A, 0xFFB347, 0xFF6A4D, 0xFF2E88 };
    for (int y = 0; y < H; y++) s_suncol_e[y] = ex(rgb(ramp(sun, 4, (y - (SUN_CY - SUN_R)) / (float)(HORIZON - (SUN_CY - SUN_R)))));
    for (int x = 0; x < W; x++) {
        int dx = x - SUN_CX;
        s_sun_h[x] = abs(dx) < SUN_R ? (int16_t)sqrtf((float)(SUN_R * SUN_R - dx * dx)) : 0;
        float m = 44 + 26 * sinf(x * 0.0105f + 1.0f) + 17 * sinf(x * 0.027f + 2.3f) + 8 * sinf(x * 0.063f + 0.4f);
        m *= 0.35f + 0.65f * fabsf(dx) / 640.0f;     // lower in the middle so the sun shows
        s_mount[x] = m < 0 ? 0 : (int16_t)m;
    }
    for (int i = 0; i < 128; i++) {
        s_mount_d[i] = ex(rgb(mix(0x2C1150, 0x0C0420, i / 80.0f)));
        s_mount_n[i] = ex(rgb(mix(0x0E0522, 0x030108, i / 80.0f)));
    }
    s_rim_d = ex(rgb(0xD040A8)); s_rim_n = ex(rgb(0x5A1648));
    s_line_d = ex(rgb(0xFF2BD6)); s_line_n = ex(rgb(0x6A1262));
    s_fog_e = ex(rgb(0xC8A8DC));
    for (int y = 0; y < H; y++) {
        float d = (y - HORIZON) / (y < HORIZON ? 150.0f : 260.0f);   // thins faster upward
        float p = 1 - d * d;
        s_fog_prof[y] = p < 0 ? 0 : p;
    }
    for (int y = HORIZON; y < H; y++) {
        float dy = y - HORIZON + 0.5f;
        s_gz[y] = CAMZ / dy;
        s_gpix[y] = CAMZ / (dy * dy);
        float fade = (dy - 3) / 70.0f;
        fade = fade < 0 ? 0 : (fade > 1 ? 1 : fade);
        if (s_gpix[y] > 0.6f) fade *= fmaxf(0, 1.2f - s_gpix[y]);   // lines merge near the horizon
        s_gfade[y] = fade;
    }
    for (int i = 0; i < NSTARS; i++) {
        s_sx[i] = rnd(W);
        s_sy[i] = 6 + rnd(HORIZON - 80);
        s_sbase[i] = 0.35f + rnd(65) / 100.0f;
        s_sspeed[i] = 0.8f + rnd(300) / 100.0f;
        s_sphase[i] = rnd(628) / 100.0f;
    }
}

// Vertical grid lines: world x = i projects to screen x = VPX + i * spacing(dy).
// They never move, so each strip's line pixels are listed once at boot.
static uint32_t *s_grid_hits;           // per hit: (rr * 800 + y) << 8 | weight 0..32
static uint32_t s_grid_start[BEAM_STRIPS + 1];

static int grid_strip(int k, uint32_t *out) {
    const int R0 = k * BEAM_STRIP_ROWS, X0 = W - BEAM_STRIP_ROWS * (k + 1);
    int n = 0;
    for (int y = HORIZON + 4; y < H; y++) {
        float dy = y - HORIZON, sp = dy * 0.36f;
        float fade = (dy - 4) / 110.0f;
        if (fade > 1) fade = 1;
        int imin = (int)ceilf((X0 - 1 - VPX) / sp), imax = (int)floorf((X0 + BEAM_STRIP_ROWS - VPX) / sp);
        for (int i = imin; i <= imax; i++) {
            float xs = VPX + i * sp;
            int xi = (int)floorf(xs);
            float f = xs - xi;
            for (int e = 0; e < 2; e++) {
                unsigned rr = (unsigned)(1279 - (xi + e) - R0);
                if (rr >= BEAM_STRIP_ROWS) continue;
                uint32_t wgt = (uint32_t)((e ? f : 1 - f) * fade * 32);
                if (!wgt) continue;
                if (out) out[n] = ((rr * BEAM_H_RES + y) << 8) | wgt;
                n++;
            }
        }
    }
    return n;
}

static bool build_grid() {
    uint32_t total = 0;
    for (int k = 0; k < BEAM_STRIPS; k++) { s_grid_start[k] = total; total += grid_strip(k, nullptr); }
    s_grid_start[BEAM_STRIPS] = total;
    // PSRAM: read in order, a couple of KB per strip; internal RAM is Wi-Fi's
    s_grid_hits = (uint32_t *)heap_caps_malloc(total * 4, MALLOC_CAP_SPIRAM);
    if (!s_grid_hits) return false;
    for (int k = 0; k < BEAM_STRIPS; k++) grid_strip(k, s_grid_hits + s_grid_start[k]);
    Serial0.printf("[scene] grid: %lu line pixels, %lu bytes\n", (unsigned long)total, (unsigned long)total * 4);
    return true;
}

// ---------------------------------------------------------------- the time image
// Built once a minute, native layout, ONE BYTE per pixel: glyph coverage in the
// high nibble, drop-shadow coverage in the low nibble (0..15 each). Colours come
// from small per-frame tables. Kept compact on purpose: a 4-byte-per-pixel layer
// meant 16 KB of PSRAM reads per strip, and the cache thrash slowed every other
// stage too.

#define LAYER_H (DIG_H + SHADOW)
#define LAYER_STRIDE 260                // multiple of 4: empty stretches skipped a word at a time
struct DigitBuf { uint8_t *px; int r0, nrows, y0; };
static DigitBuf s_dig[2];
static volatile int s_dig_front = -1, s_dig_pending = -1;
static volatile int32_t s_dig_latched = -1;
static uint32_t s_shadow_e;
static uint8_t s_glyph_a[16], s_shadow_a[16];           // nibble -> alpha 0..32

void scene_set_digits(const uint16_t *canvas, int w, int h, int stride) {
    if (w > DIG_MAXW) w = DIG_MAXW;
    if (h > DIG_H) h = DIG_H;
    while (s_dig_pending >= 0) delay(1);
    // the buffer we're about to overwrite was swapped out at frame L; it's free after L+1
    while (s_dig_latched >= 0 && beam_race_frame_done() < s_dig_latched + 1) delay(1);
    int back = s_dig_front == 0 ? 1 : 0;
    DigitBuf &d = s_dig[back];
    int x0 = 640 - w / 2, lw = w + SHADOW;
    d.r0 = 1279 - (x0 + lw - 1);
    d.nrows = lw;
    d.y0 = DIGIT_CY - DIG_H / 2;
    auto cov = [&](int x, int y) -> uint32_t {          // glyph coverage 0..15 at canvas (x, y)
        if (x < 0 || x >= w || y < 0 || y >= h) return 0;
        return (((canvas[y * stride + x] >> 5) & 63) + 2) >> 2;
    };
    for (int x = 0; x < lw; x++) {
        uint8_t *row = d.px + (lw - 1 - x) * LAYER_STRIDE;   // native row = 1279 - landscape x
        for (int y = 0; y < LAYER_STRIDE; y++) {
            uint32_t g = y < DIG_H ? cov(x, y) : 0, sh = cov(x - SHADOW, y - SHADOW);
            if (g > 15) g = 15;
            if (sh > 15) sh = 15;
            row[y] = (uint8_t)((g << 4) | (g == 15 ? 0 : sh));
        }
    }
    s_dig_pending = back;
}

// ---------------------------------------------------------------- 8-bit blocks
// Weather, shooting stars and lightning are all lists of landscape rectangles with
// a colour and an opacity, snapped to the PX grid.

struct Blk { int16_t x, y; uint8_t w, h, a; uint16_t c; };
#define MAX_SKY 200
#define MAX_WX 1150

static inline void put(Blk *list, int *n, int max, int x, int y, int w, int h, uint32_t a, uint16_t c) {
    if (*n >= max || !a || w <= 0 || h <= 0) return;
    x = x >= 0 ? x / PX * PX : -((-x + PX - 1) / PX * PX);
    y = y >= 0 ? y / PX * PX : -((-y + PX - 1) / PX * PX);
    if (x >= W || y >= H || x + w <= 0 || y + h <= 0) return;
    list[(*n)++] = { (int16_t)x, (int16_t)y, (uint8_t)(w > 255 ? 255 : w), (uint8_t)(h > 255 ? 255 : h),
                     (uint8_t)(a > 32 ? 32 : a), c };
}

// ---------------------------------------------------------------- weather state (on_frame only)


static volatile int s_want = WEATHER_CLEAR;
static volatile bool s_want_heavy = false, s_want_storm = false;
static volatile bool s_night_want = false, s_shoot_req = false;
static int s_cur = -1;              // weather being simulated
static float s_wamt = 0;            // its strength 0..1 (cross-fades)
static float s_night = 0;           // 0 day .. 1 night
static float s_fog = 0;             // fog strength 0..1

static void spawn_drop(Drop &p, bool initial) {
    p.x = rnd(-60, W + 220);
    p.y = initial ? rnd(-H, H) : rnd(-260, -20);
    p.vy = 900 + rnd(500);
    p.len = (uint8_t)((4 + (int)((p.vy - 900) / 500 * 4)) * PX);   // 12..24 px: faster drops longer
    p.ty = (int16_t)rnd(HORIZON + 8, H - 4);                         // where it lands on the grid
    p.a = 16 + rnd(10);
    p.splash = -1;
}
static void spawn_flake(Flake &p, bool initial) {
    p.x0 = rnd(W);
    p.y = initial ? rnd(0, H) : -10 - rnd(140);
    p.vy = 28 + rnd(64);
    p.size = p.vy > 78 ? 3 : p.vy > 52 ? 2 : 1;                        // in 8-bit pixels; near = bigger + faster
    p.amp = 8 + rnd(22);
    p.freq = 0.25f + rnd(90) / 100.0f;
    p.ph = rnd(628) / 100.0f;
    p.a = 20 + rnd(12);
}
static void spawn_puff(Puff &p, bool initial) {
    p.y = (int16_t)(HORIZON - 120 + rnd(250));
    p.w = (uint8_t)(16 + rnd(20)) ;                                    // in 8-bit pixels (x3 below)
    p.v = frnd(6, 20) * (rnd(2) ? 1 : -1);
    p.x = initial ? rnd(-200, W + 200) : (p.v > 0 ? -p.w * PX * 2 - rnd(200) : W + rnd(200));
    p.a = 4 + rnd(5);
}

static void start_weather(int w) {
    s_cur = w;
    if (w == WEATHER_RAIN) for (int i = 0; i < NDROPS; i++) spawn_drop(s_drop[i], true);
    if (w == WEATHER_SNOW) for (int i = 0; i < NFLAKES; i++) spawn_flake(s_flake[i], true);
    if (w == WEATHER_FOG) for (int i = 0; i < NPUFFS; i++) spawn_puff(s_puff[i], true);
}

void scene_set_weather(int w, bool heavy, bool storm) {
    s_want = ((w % WEATHER_COUNT) + WEATHER_COUNT) % WEATHER_COUNT;
    s_want_heavy = heavy;
    s_want_storm = storm;
}
int scene_weather() { return s_want; }
bool scene_heavy() { return s_want_heavy; }
bool scene_storm() { return s_want_storm; }
void scene_set_night(bool n) { s_night_want = n; }
void scene_shoot() { s_shoot_req = true; }

// Shooting star and lightning
struct Shoot { bool on; float x, y, vx, vy, age; };
static Shoot s_shoot;
static float s_shoot_wait = 5;
#define BOLT_PTS 24
#define BRANCH_PTS 8
static int16_t s_bolt_x[BOLT_PTS], s_bolt_y[BOLT_PTS];
static int16_t s_br_x[2][BRANCH_PTS], s_br_y[2][BRANCH_PTS];   // up to two forks off the main bolt
static int s_bolt_n, s_br_n[2];
static float s_bolt_age = -1, s_storm_wait = 3;

static void spawn_shoot() {
    s_shoot.on = true;
    s_shoot.x = frnd(420, W + 60);
    s_shoot.y = frnd(10, 150);
    float sp = frnd(720, 1050), ang = frnd(0.14f, 0.34f);     // down and to the left
    s_shoot.vx = -sp * cosf(ang); s_shoot.vy = sp * sinf(ang);
    s_shoot.age = 0;
}

static void spawn_bolt() {
    int x = rnd(80, W - 80), y = -6;
    s_bolt_n = 0;
    while (s_bolt_n < BOLT_PTS) {
        s_bolt_x[s_bolt_n] = (int16_t)x; s_bolt_y[s_bolt_n] = (int16_t)y; s_bolt_n++;
        int floor_y = HORIZON - s_mount[x < 0 ? 0 : x >= W ? W - 1 : x];
        if (y >= floor_y) break;
        y += rnd(12, 27);
        x += rnd(-24, 25);
    }
    // forks: from a point on the upper half, a shorter jagged run off to one side
    for (int b = 0; b < 2; b++) {
        s_br_n[b] = 0;
        if (s_bolt_n < 5 || (b == 1 && rnd(2))) continue;
        int i = rnd(1, s_bolt_n / 2 + 1), dir = rnd(2) ? 1 : -1;
        int bx = s_bolt_x[i], by = s_bolt_y[i], len = rnd(4, BRANCH_PTS + 1);
        for (int j = 0; j < len; j++) {
            s_br_x[b][j] = (int16_t)bx; s_br_y[b][j] = (int16_t)by; s_br_n[b]++;
            by += rnd(9, 20);
            bx += dir * rnd(6, 22);
        }
    }
    s_bolt_age = 0;
}

// ---------------------------------------------------------------- per frame

struct FrameState {
    uint16_t col[4][H] __attribute__((aligned(16)));   // sky + ground + grid lines + fog haze,
                                                       // in 4 ordered-dither phases (strip rows r % 4)
    uint8_t sunvis[H];                              // 0 where a sun stripe cuts through
    uint16_t suncol[H];
    uint16_t mount[128], rim;
    uint32_t line_e;
    int sun_dy;                                     // how far the sun has set
    uint16_t star[NSTARS];
    uint16_t digcol[DIG_H];
    uint32_t digcol_e[DIG_H];
    int digits;
    int nsky, nwx, nwide;
    Blk sky[MAX_SKY];                               // shooting star, lightning (behind the mountains)
    // Weather (in front of everything): wide blocks first (fog), then narrow ones
    // sorted by the strip their left edge falls in (strips run right to left). A
    // narrow block (<= 16 px) spans at most two strips, so strip k only looks at the
    // blocks filed under k and k+1.
    Blk wx[MAX_WX];
    uint16_t wx_start[BEAM_STRIPS + 2];
};
static FrameState s_fs[2];
static float s_grid_ph = 0;
static int32_t s_last_frame = -1;

static void weather_blocks(FrameState &fs, float t, float dt) {
    // cross-fade: thin the old weather out, then bring the new one in
    int want = s_want;
    if (want != s_cur) {
        s_wamt -= dt / WEATHER_OUT_S;
        if (s_wamt <= 0 || s_cur < 0) { s_wamt = 0; start_weather(want); }
    } else if (s_wamt < 1) {
        s_wamt += dt / WEATHER_IN_S;
        if (s_wamt > 1) s_wamt = 1;
    }
    float dim = s_wamt * (1 - 0.45f * s_night);
    const int w = s_cur;
    C->nwx_tmp = 0;
    Blk *const WXT = (Blk *)C->wx_tmp;

    if (w == WEATHER_RAIN) {
        static const uint16_t head_c = rgb(0xC8ECFF), body_c = rgb(0x7FB8F0), splash_c = rgb(0xD8F4FF);
        int n = s_want_heavy ? NDROPS : NDROPS * 11 / 20;
        for (int i = 0; i < n; i++) {
            Drop &p = s_drop[i];
            if (p.splash >= 0) {                           // a little 8-bit splash on the grid
                p.splash += dt;
                if (p.splash > 0.11f) { spawn_drop(p, false); continue; }
                uint32_t a = a32(0.6f * dim);
                put(WXT, &C->nwx_tmp, MAX_WX, (int)p.x - 2 * PX, p.ty - PX, PX, PX, a, splash_c);
                put(WXT, &C->nwx_tmp, MAX_WX, (int)p.x + 2 * PX, p.ty - PX, PX, PX, a, splash_c);
                if (p.splash < 0.05f) put(WXT, &C->nwx_tmp, MAX_WX, (int)p.x, p.ty - 2 * PX, PX, PX, a, splash_c);
                continue;
            }
            p.x -= 90 * dt;                                // a little wind
            p.y += p.vy * (s_want_heavy ? 1.15f : 1.0f) * dt;
            if (p.y >= p.ty) { p.y = p.ty; p.splash = 0; continue; }
            if (p.x < -40) { spawn_drop(p, false); continue; }
            int x = (int)p.x, y = (int)p.y, len = p.len, head = len / 3 / PX * PX;
            put(WXT, &C->nwx_tmp, MAX_WX, x, y - len, PX, len - head, (uint32_t)(p.a * 0.45f * dim), body_c);
            put(WXT, &C->nwx_tmp, MAX_WX, x, y - head, PX, head, (uint32_t)(p.a * dim), head_c);
        }
    } else if (w == WEATHER_SNOW) {
        static const uint16_t snow_c = rgb(0xF4F6FF);
        for (int i = 0; i < NFLAKES; i++) {
            Flake &p = s_flake[i];
            p.y += p.vy * dt;
            if (p.y > H + 5) { spawn_flake(p, false); continue; }
            int x = (int)(p.x0 + fsin(t * p.freq * 6.283f + p.ph) * p.amp), y = (int)p.y;
            uint32_t a = (uint32_t)(p.a * dim);
            if (p.size == 3) {                             // near flakes: a chunky plus
                put(WXT, &C->nwx_tmp, MAX_WX, x - PX, y, 3 * PX, PX, a, snow_c);
                put(WXT, &C->nwx_tmp, MAX_WX, x, y - PX, PX, PX, a, snow_c);
                put(WXT, &C->nwx_tmp, MAX_WX, x, y + PX, PX, PX, a, snow_c);
            } else {
                put(WXT, &C->nwx_tmp, MAX_WX, x, y, p.size * PX, p.size * PX, a, snow_c);
            }
        }
    } else if (w == WEATHER_FOG) {
        static const uint16_t fog_c = rgb(0xD8C0EA);
        for (int i = 0; i < NPUFFS; i++) {
            Puff &p = s_puff[i];
            p.x += p.v * dt;
            int wpx = p.w * PX;
            if ((p.v > 0 && p.x > W + 20) || (p.v < 0 && p.x + wpx * 1.4f < -20)) { spawn_puff(p, false); continue; }
            // a stepped 8-bit cloud: wide middle row, narrower rows above and below
            uint32_t a = (uint32_t)(p.a * dim);
            int x = (int)p.x;
            put(WXT, &C->nwx_tmp, MAX_WX, x + wpx / 5, p.y - 2 * PX, wpx * 3 / 5, 2 * PX, a, fog_c);
            put(WXT, &C->nwx_tmp, MAX_WX, x, p.y, wpx, 3 * PX, a, fog_c);
            put(WXT, &C->nwx_tmp, MAX_WX, x + wpx / 4, p.y + 3 * PX, wpx * 3 / 4, 2 * PX, a, fog_c);
        }
    }
    // fog haze strength follows the fog weather
    float fog_target = (w == WEATHER_FOG) ? s_wamt : 0;
    s_fog += (fog_target - s_fog) * (dt * 2 > 1 ? 1 : dt * 2);
}

// Counting sort of the weather blocks by strip (see FrameState::wx)
static inline int strip_of(int x) { x = x < 0 ? 0 : x > W - 1 ? W - 1 : x; return (W - 1 - x) / BEAM_STRIP_ROWS; }

static void sort_weather(FrameState &fs) {
    const Blk *src = (const Blk *)C->wx_tmp;
    const int n = C->nwx_tmp;
    uint16_t count[BEAM_STRIPS + 2] = {};
    int nw = 0;
    for (int i = 0; i < n; i++) {
        if (src[i].w > BEAM_STRIP_ROWS) nw++;
        else count[strip_of(src[i].x) + 1]++;
    }
    fs.nwide = nw;
    fs.wx_start[0] = nw;
    for (int k = 0; k <= BEAM_STRIPS; k++) fs.wx_start[k + 1] = fs.wx_start[k] + count[k + 1];
    uint16_t at[BEAM_STRIPS + 1];
    for (int k = 0; k <= BEAM_STRIPS; k++) at[k] = fs.wx_start[k];
    int iw = 0;
    for (int i = 0; i < n; i++) {
        if (src[i].w > BEAM_STRIP_ROWS) fs.wx[iw++] = src[i];
        else fs.wx[at[strip_of(src[i].x)]++] = src[i];
    }
    fs.nwx = n;
}

static void sky_blocks(FrameState &fs, float dt) {
    fs.nsky = 0;
    const bool clear = s_cur == WEATHER_CLEAR && s_wamt > 0.5f;
    if (s_shoot.on) {
        s_shoot.age += dt;
        s_shoot.x += s_shoot.vx * dt; s_shoot.y += s_shoot.vy * dt;
        if (s_shoot.x < -300 || s_shoot.y > HORIZON - 60 || s_shoot.age > 2.5f) s_shoot.on = false;
    } else if (s_shoot_req || clear) {
        s_shoot_wait -= dt;
        if (s_shoot_req || s_shoot_wait <= 0) { spawn_shoot(); s_shoot_req = false; s_shoot_wait = frnd(6, 18); }
    }
    if (s_shoot.on) {
        static const uint16_t head_c = rgb(0xFFFFFF), tail1 = rgb(0x9FF4FF), tail2 = rgb(0xFF6AD8);
        float len = sqrtf(s_shoot.vx * s_shoot.vx + s_shoot.vy * s_shoot.vy);
        float ux = s_shoot.vx / len, uy = s_shoot.vy / len;
        float in = s_shoot.age < 0.12f ? s_shoot.age / 0.12f : 1;
        float out = s_shoot.y > HORIZON - 160 ? (HORIZON - 60 - s_shoot.y) / 100.0f : 1;   // fade before the mountains
        float k = in * (out < 0 ? 0 : out);
        for (int j = 13; j >= 0; j--) {
            float bx = s_shoot.x - ux * j * 13, by = s_shoot.y - uy * j * 13;
            uint32_t a = (uint32_t)(32 * (1 - j / 14.0f) * k);
            int sz = j < 2 ? 2 * PX : PX;
            put(fs.sky, &fs.nsky, MAX_SKY, (int)bx, (int)by, sz, sz, a, j < 2 ? head_c : j < 7 ? tail1 : tail2);
        }
    }

    // lightning: in a storm, every 6-20 s, a bolt and a sky flash
    if (s_want_storm && s_cur == WEATHER_RAIN && s_wamt > 0.5f) {
        if (s_bolt_age < 0) {
            s_storm_wait -= dt;
            if (s_storm_wait <= 0) { spawn_bolt(); s_storm_wait = frnd(3, 10); }
        }
    }
    if (s_bolt_age >= 0) {
        s_bolt_age += dt;
        if (s_bolt_age > 0.35f) { s_bolt_age = -1; return; }
        bool lit = s_bolt_age < 0.09f || (s_bolt_age > 0.16f && s_bolt_age < 0.22f);
        if (!lit) return;
        static const uint16_t bolt_c = rgb(0xF2F6FF), fork_c = rgb(0xC8D2FF);
        auto run = [&](const int16_t *px, const int16_t *py, int n, int sz, uint32_t a, uint16_t c) {
            for (int i = 0; i + 1 < n; i++) {               // each segment as stepped 8-bit blocks
                int x0 = px[i], y0 = py[i], x1 = px[i + 1], y1 = py[i + 1];
                int steps = (y1 - y0) / PX;
                for (int s = 0; s < steps; s += 2)
                    put(fs.sky, &fs.nsky, MAX_SKY, x0 + (x1 - x0) * s / (steps ? steps : 1), y0 + s * PX, sz, sz, a, c);
            }
        };
        run(s_bolt_x, s_bolt_y, s_bolt_n, 2 * PX, 32, bolt_c);
        for (int b = 0; b < 2; b++) run(s_br_x[b], s_br_y[b], s_br_n[b], PX, 26, fork_c);
    }
}

void scene_on_frame(int32_t F, void *) {
    FrameState &fs = s_fs[F & 1];
    float dt = s_last_frame < 0 ? FRAME_DT : (F - s_last_frame) * FRAME_DT;
    if (dt > 0.1f) dt = 0.1f;
    s_last_frame = F;
    float t = F * FRAME_DT;

    // night eases in and out
    s_night += (s_night_want ? dt : -dt) / NIGHT_FADE_S;
    s_night = s_night < 0 ? 0 : s_night > 1 ? 1 : s_night;
    const uint32_t na = a32(s_night);

    weather_blocks(fs, t, dt);
    sort_weather(fs);
    sky_blocks(fs, dt);

    // lightning flash lights up the sky and the ground a little
    uint32_t flash = 0;
    if (s_bolt_age >= 0 && (s_bolt_age < 0.09f || (s_bolt_age > 0.16f && s_bolt_age < 0.22f)))
        flash = s_bolt_age < 0.09f ? 13 : 7;
    static const uint32_t FLASH8 = 0xB8C4FF, FOG8 = 0xC8A8DC;
    const uint32_t fa = a32(s_fog * 0.7f);

    // the dithered sky and ground base only change with night, fog and lightning
    const uint32_t key = 1u << 31 | na << 16 | fa << 8 | flash;
    const bool remake = key != C->cache_key;
    C->cache_key = key;
    if (remake) {
        for (int y = 0; y < HORIZON; y++) {
            uint32_t c = lerp8(s_sky_d[y], s_sky_n[y], na);
            if (fa) c = lerp8(c, FOG8, (fa * (uint32_t)(s_fog_prof[y] * 32)) >> 5);
            if (flash) c = add8(c, FLASH8, flash);
            for (int p = 0; p < 4; p++) C->sky_cache[p][y] = dither565(c, BAYER[y & 3][p]);
        }
        for (int y = HORIZON; y < H; y++) {
            uint32_t c = lerp8(s_ground_d[y], s_ground_n[y], na);
            uint32_t fy = fa ? (fa * (uint32_t)(s_fog_prof[y] * 32)) >> 5 : 0;
            if (fy) c = lerp8(c, FOG8, fy);
            if (flash) c = add8(c, FLASH8, flash / 3);
            for (int p = 0; p < 4; p++) C->gnd_cache[p][y] = dither565(c, BAYER[y & 3][p]);
        }
    }
    for (int p = 0; p < 4; p++) memcpy(fs.col[p], C->sky_cache[p], HORIZON * 2);

    // Ground: horizontal lines at world depth n - phase (per-row depth precomputed)
    float speed = 1.1f - 0.75f * s_night;
    s_grid_ph += speed * dt;
    s_grid_ph -= (int)s_grid_ph;
    fs.line_e = lerp_e(s_line_d, s_line_n, na);
    if (fa) fs.line_e = lerp_e(fs.line_e, scale_e(s_fog_e, 12), fa / 2);
    for (int y = HORIZON; y < H; y++) {
        float v = s_gz[y] + s_grid_ph;
        float d = fabsf(v - (float)(int)(v + 0.5f));
        float wgt = 1.6f - d / s_gpix[y];
        wgt = wgt < 0 ? 0 : (wgt > 1 ? 1 : wgt);
        uint32_t fy = fa ? (fa * (uint32_t)(s_fog_prof[y] * 32)) >> 5 : 0;
        uint32_t line = scale_e(fs.line_e, (uint32_t)(wgt * s_gfade[y] * (32 - fy)));   // fog veils the lines
        if (!line) { for (int p = 0; p < 4; p++) fs.col[p][y] = C->gnd_cache[p][y]; continue; }
        for (int p = 0; p < 4; p++) fs.col[p][y] = add_e(C->gnd_cache[p][y], line);
    }

    // Sun: sets at night; stripes slide down (integer math); fog washes it out
    fs.sun_dy = (int)(s_night * s_night * SUN_SET);           // eases into the horizon
    const int top = SUN_CY - SUN_R + fs.sun_dy, s0 = SUN_CY - 10 + fs.sun_dy;
    int off = (int)(t * 12.0f) % 24;
    const uint32_t sfa = a32(s_fog * 0.55f);
    for (int y = 0; y < H; y++) {
        if (y < top || y >= HORIZON) { fs.sunvis[y] = 0; continue; }
        if (y < s0) fs.sunvis[y] = 1;
        else {
            int pos = ((y - s0) - off) % 24;
            if (pos < 0) pos += 24;
            fs.sunvis[y] = pos >= 2 + (11 * (y - s0)) / (HORIZON - s0 > 0 ? HORIZON - s0 : 1);
        }
        uint32_t e = s_suncol_e[y - fs.sun_dy];
        fs.suncol[y] = pk(sfa ? lerp_e(e, s_fog_e, sfa) : e);
    }

    for (int i = 0; i < 128; i++) {
        uint32_t e = lerp_e(s_mount_d[i], s_mount_n[i], na);
        fs.mount[i] = pk(fa ? lerp_e(e, s_fog_e, fa * 2 / 3) : e);
    }
    fs.rim = pk(lerp_e(s_rim_d, s_rim_n, na));

    const float star_k = (1 - 0.75f * s_fog) * (flash ? 0.3f : 1.0f);
    for (int i = 0; i < NSTARS; i++) {
        float b = s_sbase[i] * (0.55f + 0.45f * fsin(t * s_sspeed[i] + s_sphase[i])) * star_k;
        fs.star[i] = C->star_lut[b <= 0 ? 0 : b >= 1 ? 32 : (int)(b * 32)];
    }

    // digits take the weather's colours (cross-fading with it), and night's over that
    const uint32_t wa = a32(s_wamt);
    const uint32_t *pal = C->digpal[s_cur < 0 ? WEATHER_CLEAR : s_cur], *clr = C->digpal[WEATHER_CLEAR];
    for (int c = 0; c < DIG_H; c++) {
        uint32_t e = wa >= 32 ? pal[c] : lerp_e(clr[c], pal[c], wa);
        if (na) e = lerp_e(e, C->dig_night[c], na);
        fs.digcol_e[c] = e;
        fs.digcol[c] = pk(e);
    }

    if (s_dig_pending >= 0) { s_dig_front = s_dig_pending; s_dig_pending = -1; s_dig_latched = F; }
    fs.digits = s_dig_front;
}

// ---------------------------------------------------------------- per strip

__attribute__((optimize("O2"), always_inline)) static inline void draw_blocks(const Blk *b, int n, uint16_t *dst, int X0, int R0) {
    for (int i = 0; i < n; i++) {
        const Blk &q = b[i];
        int xa = q.x > X0 ? q.x : X0, xb = q.x + q.w < X0 + BEAM_STRIP_ROWS ? q.x + q.w : X0 + BEAM_STRIP_ROWS;
        if (xa >= xb) continue;
        int ya = q.y > 0 ? q.y : 0, yb = q.y + q.h < H ? q.y + q.h : H;
        if (ya >= yb) continue;
        const uint32_t ce = ex(q.c);
        for (int xx = xa; xx < xb; xx++) {
            uint16_t *d = dst + (1279 - xx - R0) * BEAM_H_RES;
            if (q.a >= 32) for (int y = ya; y < yb; y++) d[y] = q.c;
            else for (int y = ya; y < yb; y++) d[y] = blend_e(ce, d[y], q.a);
        }
    }
}

// Stage timing (cycles, per core): fill, sky, sun+mountains, grid, digits, weather
static volatile uint64_t s_prof[2][7];
#define PROF(i) do { uint32_t _t = esp_cpu_get_cycle_count(); pc[i] += _t - t; t = _t; } while (0)

void scene_prof_print() {
    static const char *names[] = { "fill", "sky", "sun+mtn", "grid", "digits", "weather" };
    uint64_t sum[7] = {};
    for (int c = 0; c < 2; c++) for (int i = 0; i < 7; i++) { sum[i] += s_prof[c][i]; s_prof[c][i] = 0; }
    if (!sum[6]) return;
    float mhz = getCpuFrequencyMhz();
    Serial0.print("[Prof] per strip:");
    for (int i = 0; i < 6; i++) Serial0.printf(" %s %.0f", names[i], sum[i] / mhz / sum[6]);
    Serial0.println(" us");
}

__attribute__((optimize("O2"))) void scene_on_strip(int k, int32_t F, uint16_t *dst, void *) {
    volatile uint64_t *pc = s_prof[xPortGetCoreID()];
    uint32_t t = esp_cpu_get_cycle_count();
    const FrameState &fs = s_fs[F & 1];
    const int R0 = k * BEAM_STRIP_ROWS;
    const int X0 = W - BEAM_STRIP_ROWS * (k + 1);            // strip covers landscape x X0..X0+15

    for (int p = 0; p < 4; p++)                               // the column into all 16 rows, 4 dither phases
        pie_fill_rows(dst + p * BEAM_H_RES, fs.col[p], H / 8, BEAM_STRIP_ROWS / 4, 4 * BEAM_H_RES * 2);
    PROF(0);

    // stars, shooting star and lightning (the sun and mountains then cover what's behind them)
    for (int i = 0; i < NSTARS; i++) {
        unsigned rr = (unsigned)(1279 - s_sx[i] - R0);
        if (rr < BEAM_STRIP_ROWS) dst[rr * BEAM_H_RES + s_sy[i]] = fs.star[i];
    }
    draw_blocks(fs.sky, fs.nsky, dst, X0, R0);
    PROF(1);

    for (int rr = 0; rr < BEAM_STRIP_ROWS; rr++) {
        uint16_t *d = dst + rr * BEAM_H_RES;
        int x = 1279 - (R0 + rr);
        int h = s_sun_h[x];
        if (h > 0) {
            int y0 = SUN_CY - h + fs.sun_dy, y1 = SUN_CY + h + fs.sun_dy;
            if (y1 > HORIZON) y1 = HORIZON;
            for (int y = y0 < 0 ? 0 : y0; y < y1; y++) if (fs.sunvis[y]) d[y] = fs.suncol[y];
        }
        int m = s_mount[x];
        if (m > 0) {
            int yt = HORIZON - m;
            d[yt] = fs.rim;
            for (int y = yt + 1; y < HORIZON; y++) d[y] = fs.mount[y - yt];
        }
    }

    PROF(2);
    // vertical grid lines: replay this strip's precomputed pixels (offset << 8 | weight)
    for (uint32_t i = s_grid_start[k], e = s_grid_start[k + 1]; i < e; i++) {
        uint32_t h = s_grid_hits[i];
        uint16_t *p = dst + (h >> 8);
        *p = add_e(*p, scale_e(fs.line_e, h & 0xFF));
    }

    PROF(3);
    // the time: drop shadow, then the glyphs in a neon gradient
    if (fs.digits >= 0) {
        const DigitBuf &db = s_dig[fs.digits];
        for (int rr = 0; rr < BEAM_STRIP_ROWS; rr++) {
            int g = R0 + rr - db.r0;
            if ((unsigned)g >= (unsigned)db.nrows) continue;
            const uint8_t *p = db.px + g * LAYER_STRIDE;
            const uint32_t *pw = (const uint32_t *)p;
            uint16_t *dd = dst + rr * BEAM_H_RES + db.y0;
            for (int c4 = 0; c4 < LAYER_H; c4 += 4) {
                const uint32_t w4 = pw[c4 >> 2];
                if (!w4) continue;                            // four empty pixels
                if (w4 == 0xF0F0F0F0u && c4 + 4 <= LAYER_H) {  // four solid glyph pixels: plain stores
                    dd[c4] = fs.digcol[c4]; dd[c4 + 1] = fs.digcol[c4 + 1];
                    dd[c4 + 2] = fs.digcol[c4 + 2]; dd[c4 + 3] = fs.digcol[c4 + 3];
                    continue;
                }
                for (int c = c4; c < c4 + 4 && c < LAYER_H; c++) {
                    uint32_t v = p[c];
                    if (!v) continue;
                    uint32_t gl = v >> 4;
                    if (gl == 15) { dd[c] = fs.digcol[c]; continue; }   // inside a glyph: plain store
                    uint16_t px = dd[c];
                    if (v & 15) px = blend_e(s_shadow_e, px, s_shadow_a[v & 15]);
                    if (gl) px = blend_e(fs.digcol_e[c], px, s_glyph_a[gl]);
                    dd[c] = px;
                }
            }
        }
    }

    PROF(4);
    // weather on top: the wide blocks, then the narrow ones filed under this strip and the next
    draw_blocks(fs.wx, fs.nwide, dst, X0, R0);
    {
        int a = fs.wx_start[k], b = fs.wx_start[k + 2];
        draw_blocks(fs.wx + a, b - a, dst, X0, R0);
    }
    PROF(5);
    pc[6]++;
}

// ---------------------------------------------------------------- init

bool scene_init() {
    C = (Cold *)heap_caps_calloc(1, sizeof(Cold), MALLOC_CAP_SPIRAM);
    if (!C) return false;
    for (int i = 0; i < 256; i++) s_sin[i] = sinf(i * 6.2831853f / 256);
    build_static();
    if (!build_grid()) return false;
    for (int i = 0; i <= 32; i++) C->star_lut[i] = rgb(mix(0x000000, 0xEFE6FF, i / 32.0f));
    static const uint32_t DIGPAL[WEATHER_COUNT][4] = {
        { 0xE0FFFF, 0x7FE8FF, 0xFF6ADB, 0xFF2E7E },   // clear: cyan to hot pink
        { 0xF2D2FF, 0xCB7BFF, 0xA24BFF, 0x7B2BFF },   // rain: neon purple
        { 0xFFFFFF, 0xD6F6FF, 0x96DFFF, 0x52BEFF },   // snow: ice blue
        { 0xF6EEFF, 0xDCCBF2, 0xBCA4DE, 0x9C84C8 },   // fog: soft lavender
    };
    const uint32_t dig_n[] = { 0xC04A9A, 0xA02A7E, 0x8A1658, 0x700C34 };   // night: deep magenta to red
    for (int c = 0; c < DIG_H; c++) {
        float t = c / (float)(DIG_H - 1);
        for (int w = 0; w < WEATHER_COUNT; w++) C->digpal[w][c] = ex(rgb(ramp(DIGPAL[w], 4, t)));
        C->dig_night[c] = ex(rgb(ramp(dig_n, 4, t)));
    }
    s_shadow_e = ex(rgb(0x14001E));
    for (int i = 0; i < 16; i++) { s_glyph_a[i] = (i * 32 + 7) / 15; s_shadow_a[i] = (i * 22 + 7) / 15; }
    for (int i = 0; i < 2; i++) {
        s_dig[i].px = (uint8_t *)heap_caps_aligned_calloc(4, 1, (DIG_MAXW + SHADOW) * LAYER_STRIDE, MALLOC_CAP_SPIRAM);
        if (!s_dig[i].px) return false;
    }
    return true;
}
