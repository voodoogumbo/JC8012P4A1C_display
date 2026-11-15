// Synthwave clock scene, drawn one 16-column strip at a time.
//
// Back to front: sky gradient, twinkling stars, a striped sun, a mountain
// silhouette, a neon grid (horizontal lines scroll toward you, vertical lines
// converge on the horizon), the time in neon digits, then the weather.
//
// Everything that depends only on y (sky, ground, scrolling grid lines, sun
// stripes) is baked once per frame into one 800-pixel column, so each strip
// starts as 16 memcpys of it and only the x-dependent parts are drawn per pixel.

#include <Arduino.h>
#include <math.h>
#include "esp_heap_caps.h"
#include "beam_race.h"
#include "scene.h"

#define W 1280
#define H 800
#define HORIZON 470
#define VPX 640                   // vanishing point x
#define SUN_CX 640
#define SUN_CY 300
#define SUN_R 200
#define DIGIT_CY 300              // time image centred here (landscape)
#define DIG_MAXW 1200
#define DIG_H 250
#define SHADOW 7                  // drop shadow offset, px right and down
#define NSTARS 220
#define MAXP 360

static const float FRAME_DT = 880.0f * 1312.0f / 60e6f;   // 19.24 ms

// ---------------------------------------------------------------- colour helpers

#define MASK 0x07E0F81Fu
static inline uint32_t ex(uint32_t c) { return (c | (c << 16)) & MASK; }
static inline uint16_t pk(uint32_t e) { return (uint16_t)(e | (e >> 16)); }
static inline uint16_t blend_e(uint32_t fe, uint16_t bg, uint32_t a) {   // a 0..32
    uint32_t b = ex(bg);
    return pk((b + (((fe - b) * a) >> 5)) & MASK);
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

// Fast PRNG for particles and stars. Arduino's rnd() reads the hardware RNG,
// which is slow here: respawning raindrops with it stretched the per-frame setup
// to milliseconds (110 ms when the weather switched), and strips went late.
static uint32_t s_rng = 0x9E3779B9u;
static inline uint32_t rng() { s_rng ^= s_rng << 13; s_rng ^= s_rng >> 17; s_rng ^= s_rng << 5; return s_rng; }
static inline int rnd(int n) { return n > 0 ? (int)(rng() % (uint32_t)n) : 0; }
static inline int rnd(int lo, int hi) { return lo + rnd(hi - lo); }

// Table sine for the per-frame setup (stars, sway, flicker): sinf() on ever-growing
// time arguments kept the ember update in the milliseconds.
static float s_sin[256];
static uint16_t s_ember_col[64];         // hot yellow at the bottom to red at the top
static inline float fsin(float x) { return s_sin[(int32_t)(x * (256.0f / 6.2831853f)) & 255]; }

// ---------------------------------------------------------------- static scene

static uint16_t s_sky[HORIZON];
static uint16_t s_ground[H];
static uint16_t s_suncol[H];
static int16_t s_sun_h[W];        // sun half-height at each x (0 = no sun)
static int16_t s_mount[W];        // mountain height above the horizon
static uint16_t s_mountcol[128];
static uint16_t s_rim;
static float s_gz[H], s_gpix[H], s_gfade[H];   // ground rows: depth, depth per pixel, line fade
#define CAMZ 1555.0f
static uint32_t s_line_e;         // grid colour, spread form
static int16_t s_sx[NSTARS], s_sy[NSTARS];
static float s_sbase[NSTARS], s_sspeed[NSTARS], s_sphase[NSTARS];

static void build_static() {
    const uint32_t sky[] = { 0x07021A, 0x1C0840, 0x4A1260, 0xA0207A };
    for (int y = 0; y < HORIZON; y++) s_sky[y] = rgb(ramp(sky, 4, y / (float)(HORIZON - 1)));
    for (int y = HORIZON; y < H; y++) s_ground[y] = rgb(mix(0x1E0838, 0x040110, (y - HORIZON) / (float)(H - HORIZON)));

    const uint32_t sun[] = { 0xFFF27A, 0xFFB347, 0xFF6A4D, 0xFF2E88 };
    for (int y = 0; y < H; y++) s_suncol[y] = rgb(ramp(sun, 4, (y - (SUN_CY - SUN_R)) / (float)(HORIZON - (SUN_CY - SUN_R))));
    for (int x = 0; x < W; x++) {
        int dx = x - SUN_CX;
        s_sun_h[x] = abs(dx) < SUN_R ? (int16_t)sqrtf((float)(SUN_R * SUN_R - dx * dx)) : 0;
        float m = 44 + 26 * sinf(x * 0.0105f + 1.0f) + 17 * sinf(x * 0.027f + 2.3f) + 8 * sinf(x * 0.063f + 0.4f);
        m *= 0.35f + 0.65f * fabsf(dx) / 640.0f;     // lower in the middle so the sun shows
        s_mount[x] = m < 0 ? 0 : (int16_t)m;
    }
    for (int i = 0; i < 128; i++) s_mountcol[i] = rgb(mix(0x2C1150, 0x0C0420, i / 80.0f));
    s_rim = rgb(0xD040A8);
    for (int y = HORIZON; y < H; y++) {
        float dy = y - HORIZON + 0.5f;
        s_gz[y] = CAMZ / dy;
        s_gpix[y] = CAMZ / (dy * dy);
        float fade = (dy - 3) / 70.0f;
        fade = fade < 0 ? 0 : (fade > 1 ? 1 : fade);
        if (s_gpix[y] > 0.6f) fade *= fmaxf(0, 1.2f - s_gpix[y]);   // lines merge near the horizon
        s_gfade[y] = fade;
    }
    s_line_e = ex(rgb(0xFF2BD6));

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
    s_grid_hits = (uint32_t *)heap_caps_malloc(total * 4, MALLOC_CAP_INTERNAL);
    if (!s_grid_hits) s_grid_hits = (uint32_t *)heap_caps_malloc(total * 4, MALLOC_CAP_SPIRAM);
    if (!s_grid_hits) return false;
    for (int k = 0; k < BEAM_STRIPS; k++) grid_strip(k, s_grid_hits + s_grid_start[k]);
    Serial0.printf("[scene] grid: %lu line pixels, %lu bytes\n", (unsigned long)total, (unsigned long)total * 4);
    return true;
}

// ---------------------------------------------------------------- the time image
// Built once a minute, native layout, ONE BYTE per pixel: glyph coverage in the
// high nibble, drop-shadow coverage in the low nibble (0..15 each). Colours come
// from small tables in internal RAM. Kept compact on purpose: a 4-byte-per-pixel
// layer meant 16 KB of PSRAM reads per strip, and the cache thrash slowed every
// other stage too (Mike's point about PSRAM contention).

#define LAYER_H (DIG_H + SHADOW)
#define LAYER_STRIDE 260                // multiple of 4: empty stretches skipped a word at a time
struct DigitBuf { uint8_t *px; int r0, nrows, y0; };
static DigitBuf s_dig[2];
static volatile int s_dig_front = -1, s_dig_pending = -1;
static volatile int32_t s_dig_latched = -1;
static uint16_t s_digcol[DIG_H];        // glyph colour per row, RGB565
static uint32_t s_digcol_e[DIG_H];      // same, spread form
static uint32_t s_shadow_e;
static uint8_t s_glyph_a[16], s_shadow_a[16];   // nibble -> alpha 0..32

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

// ---------------------------------------------------------------- weather

struct Particle { int16_t x, y; uint8_t size, alpha; uint16_t color; };
struct Sim { float x, y, x0, vy, amp, freq, ph; uint8_t size, alpha; };
static Sim s_sim[MAXP];
static int s_nsim;
static volatile int s_weather = WEATHER_CLEAR, s_sim_weather = -1;

static int weather_count(int w) { return w == WEATHER_RAIN ? 320 : w == WEATHER_SNOW ? 240 : w == WEATHER_EMBERS ? 150 : 0; }

static void spawn(Sim &p, int w, bool initial) {
    switch (w) {
    case WEATHER_RAIN:
        p.x = rnd(-60, W + 220);
        p.y = initial ? rnd(-H, H) : rnd(-220, -20);
        p.vy = 950 + rnd(450);
        p.size = 14 + (int)((p.vy - 950) / 450 * 20);        // streak length
        p.alpha = 10 + rnd(9);
        break;
    case WEATHER_SNOW:
        p.x0 = rnd(W);
        p.y = initial ? rnd(0, H) : -10 - rnd(120);
        p.vy = 30 + rnd(60);
        p.size = 2 + (int)((p.vy - 30) / 30);                // nearer flakes: bigger and faster
        p.amp = 8 + rnd(22);
        p.freq = 0.25f + rnd(90) / 100.0f;
        p.ph = rnd(628) / 100.0f;
        p.alpha = 20 + rnd(12);
        break;
    case WEATHER_EMBERS:
        p.x0 = rnd(W);
        p.y = initial ? rnd(HORIZON, H + 40) : H + rnd(40);
        p.vy = -(45 + rnd(95));
        p.size = 2 + rnd(2);
        p.amp = 6 + rnd(16);
        p.freq = 0.3f + rnd(80) / 100.0f;
        p.ph = rnd(628) / 100.0f;
        break;
    }
}

static void step_weather(float t, float dt, Particle *out, int *np) {
    int w = s_weather;
    if (w != s_sim_weather) {
        s_sim_weather = w;
        s_nsim = weather_count(w);
        for (int i = 0; i < s_nsim; i++) spawn(s_sim[i], w, true);
    }
    static const uint16_t rain_c = rgb(0xB8D4FF), snow_c = rgb(0xF2F6FF);
    for (int i = 0; i < s_nsim; i++) {
        Sim &p = s_sim[i];
        Particle &o = out[i];
        switch (w) {
        case WEATHER_RAIN:
            p.x -= 110 * dt;                                  // a little wind
            p.y += p.vy * dt;
            if (p.y - p.size > H || p.x < -40) spawn(p, w, false);
            o = { (int16_t)p.x, (int16_t)p.y, p.size, p.alpha, rain_c };
            break;
        case WEATHER_SNOW:
            p.y += p.vy * dt;
            p.x = p.x0 + fsin(t * p.freq * 6.283f + p.ph) * p.amp;
            if (p.y > H + 5) spawn(p, w, false);
            o = { (int16_t)p.x, (int16_t)p.y, p.size, p.alpha, snow_c };
            break;
        case WEATHER_EMBERS: {
            p.y += p.vy * dt;
            p.x = p.x0 + fsin(t * p.freq * 6.283f + p.ph) * p.amp;
            if (p.y < 60) spawn(p, w, false);
            float up = (H - p.y) / (float)(H - 60);           // 0 at the bottom, 1 at the top
            up = up < 0 ? 0 : (up > 1 ? 1 : up);
            float flick = 0.7f + 0.3f * fsin(t * 13.0f + p.ph * 7);
            float a = 30 * (1 - up) * flick;
            o = { (int16_t)p.x, (int16_t)p.y, p.size, (uint8_t)a, s_ember_col[(int)(up * 63)] };
            break;
        }
        }
    }
    *np = s_nsim;
}

void scene_set_weather(int w) { s_weather = ((w % WEATHER_COUNT) + WEATHER_COUNT) % WEATHER_COUNT; }
int scene_weather() { return s_weather; }

// ---------------------------------------------------------------- per frame

struct FrameState {
    uint16_t col[H];              // sky + ground with the scrolling horizontal lines
    uint8_t sunvis[H];            // 0 where a sun stripe cuts through
    uint16_t star[NSTARS];
    int digits;
    uint8_t weather;
    int np;
    Particle p[MAXP];
};
static FrameState s_fs[2];

void scene_on_frame(int32_t F, void *) {
    FrameState &fs = s_fs[F & 1];
    float t = F * FRAME_DT;

    memcpy(fs.col, s_sky, sizeof s_sky);

    // Ground: horizontal lines at world depth n - phase (per-row depth precomputed)
    const float SPEED = 1.1f;
    float ph = t * SPEED;
    ph -= (int)ph;
    for (int y = HORIZON; y < H; y++) {
        float v = s_gz[y] + ph;
        float d = fabsf(v - (float)(int)(v + 0.5f));
        float wgt = 1.6f - d / s_gpix[y];
        wgt = wgt < 0 ? 0 : (wgt > 1 ? 1 : wgt);
        fs.col[y] = add_e(s_ground[y], scale_e(s_line_e, (uint32_t)(wgt * s_gfade[y] * 32)));
    }

    // Sun stripes: gaps that thicken toward the horizon and slide down (integer math)
    const int s0 = SUN_CY - 10;
    int off = (int)(t * 12.0f) % 24;
    for (int y = 0; y < H; y++) {
        if (y < s0 || y >= HORIZON) { fs.sunvis[y] = 1; continue; }
        int pos = ((y - s0) - off) % 24;
        if (pos < 0) pos += 24;
        fs.sunvis[y] = pos >= 2 + (11 * (y - s0)) / (HORIZON - s0);
    }

    for (int i = 0; i < NSTARS; i++) {
        float b = s_sbase[i] * (0.55f + 0.45f * fsin(t * s_sspeed[i] + s_sphase[i]));
        fs.star[i] = rgb(mix(0x000000, 0xEFE6FF, b));
    }

    if (s_dig_pending >= 0) { s_dig_front = s_dig_pending; s_dig_pending = -1; s_dig_latched = F; }
    fs.digits = s_dig_front;

    fs.weather = s_weather;
    step_weather(t, FRAME_DT, fs.p, &fs.np);
}

// ---------------------------------------------------------------- per strip

__attribute__((optimize("O2"))) void scene_on_strip(int k, int32_t F, uint16_t *dst, void *) {
    const FrameState &fs = s_fs[F & 1];
    const int R0 = k * BEAM_STRIP_ROWS;
    const int X0 = W - BEAM_STRIP_ROWS * (k + 1);            // strip covers landscape x X0..X0+15

    for (int rr = 0; rr < BEAM_STRIP_ROWS; rr++) memcpy(dst + rr * BEAM_H_RES, fs.col, sizeof fs.col);

    // stars (the sun and mountains then cover the ones behind them)
    for (int i = 0; i < NSTARS; i++) {
        unsigned rr = (unsigned)(1279 - s_sx[i] - R0);
        if (rr < BEAM_STRIP_ROWS) dst[rr * BEAM_H_RES + s_sy[i]] = fs.star[i];
    }

    for (int rr = 0; rr < BEAM_STRIP_ROWS; rr++) {
        uint16_t *d = dst + rr * BEAM_H_RES;
        int x = 1279 - (R0 + rr);
        int h = s_sun_h[x];
        if (h > 0) {
            int y0 = SUN_CY - h, y1 = SUN_CY + h < HORIZON ? SUN_CY + h : HORIZON;
            for (int y = y0 < 0 ? 0 : y0; y < y1; y++) if (fs.sunvis[y]) d[y] = s_suncol[y];
        }
        int m = s_mount[x];
        if (m > 0) {
            int yt = HORIZON - m;
            d[yt] = s_rim;
            for (int y = yt + 1; y < HORIZON; y++) d[y] = s_mountcol[y - yt];
        }
    }

    // vertical grid lines: replay this strip's precomputed pixels (offset << 8 | weight)
    for (uint32_t i = s_grid_start[k], e = s_grid_start[k + 1]; i < e; i++) {
        uint32_t h = s_grid_hits[i];
        uint16_t *p = dst + (h >> 8);
        *p = add_e(*p, scale_e(s_line_e, h & 0xFF));
    }

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
                if (!pw[c4 >> 2]) continue;                   // four empty pixels
                for (int c = c4; c < c4 + 4 && c < LAYER_H; c++) {
                    uint32_t v = p[c];
                    if (!v) continue;
                    uint32_t gl = v >> 4;
                    if (gl == 15) { dd[c] = s_digcol[c]; continue; }   // inside a glyph: plain store
                    uint16_t px = dd[c];
                    if (v & 15) px = blend_e(s_shadow_e, px, s_shadow_a[v & 15]);
                    if (gl) px = blend_e(s_digcol_e[c], px, s_glyph_a[gl]);
                    dd[c] = px;
                }
            }
        }
    }

    // weather on top
    for (int i = 0; i < fs.np; i++) {
        const Particle &p = fs.p[i];
        if (!p.alpha) continue;
        uint32_t ce = ex(p.color);
        if (fs.weather == WEATHER_RAIN) {
            unsigned rr = (unsigned)(1279 - p.x - R0);
            if (rr >= BEAM_STRIP_ROWS) continue;
            uint16_t *d = dst + rr * BEAM_H_RES;
            int top = p.y - p.size;
            for (int y = top < 0 ? 0 : top, y1 = p.y < H ? p.y : H - 1; y <= y1; y++)
                d[y] = blend_e(ce, d[y], p.alpha * (y - top) / p.size);   // tail fades out
        } else {
            int s = p.size, halo = fs.weather == WEATHER_EMBERS;
            for (int pass = halo ? 0 : 1; pass < 2; pass++) {
                int grow = pass == 0 ? 1 : 0;                  // embers: dim halo first
                int x0 = p.x - grow, y0 = p.y - grow, sz = s + 2 * grow;
                uint32_t a = pass == 0 ? p.alpha / 3 : p.alpha;
                for (int xx = x0; xx < x0 + sz; xx++) {
                    unsigned rr = (unsigned)(1279 - xx - R0);
                    if (rr >= BEAM_STRIP_ROWS) continue;
                    uint16_t *d = dst + rr * BEAM_H_RES;
                    for (int y = y0 < 0 ? 0 : y0; y < y0 + sz && y < H; y++) d[y] = blend_e(ce, d[y], a);
                }
            }
        }
    }
}

// ---------------------------------------------------------------- init

bool scene_init() {
    for (int i = 0; i < 256; i++) s_sin[i] = sinf(i * 6.2831853f / 256);
    for (int i = 0; i < 64; i++) s_ember_col[i] = rgb(mix(0xFFD27A, 0xFF3A14, i / 63.0f * 1.4f));
    build_static();
    if (!build_grid()) return false;
    const uint32_t dig[] = { 0xE0FFFF, 0x7FE8FF, 0xFF6ADB, 0xFF2E7E };
    for (int c = 0; c < DIG_H; c++) {
        s_digcol[c] = rgb(ramp(dig, 4, c / (float)(DIG_H - 1)));
        s_digcol_e[c] = ex(s_digcol[c]);
    }
    s_shadow_e = ex(rgb(0x14001E));
    for (int i = 0; i < 16; i++) { s_glyph_a[i] = (i * 32 + 7) / 15; s_shadow_a[i] = (i * 22 + 7) / 15; }
    for (int i = 0; i < 2; i++) {
        s_dig[i].px = (uint8_t *)heap_caps_aligned_calloc(4, 1, (DIG_MAXW + SHADOW) * LAYER_STRIDE, MALLOC_CAP_SPIRAM);
        if (!s_dig[i].px) return false;
    }
    return true;
}
