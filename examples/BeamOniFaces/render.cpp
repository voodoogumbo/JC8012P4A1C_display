// Oni faces stress test: up to 1000 bouncing pixel-art faces (horned skulls and
// oni), drawn strip by strip just ahead of the display DMA via beam_race, with an
// LVGL band along the bottom copied into every strip. The blend, the background
// and the band copy use PIE (beam_pie.h), checked against plain C at boot.

#include <Arduino.h>
#include <math.h>
#include "esp_heap_caps.h"
#include "esp_memory_utils.h"
#include "esp_cpu.h"
#include "beam_race.h"
#include "layout.h"
#include "render.h"
#include "beam_pie.h"

// ---------------------------------------------------------------- memory

static uint16_t *s_band[2];              // band in native layout: 1280 rows x BAND_H px
static uint16_t s_bg[PLAY_H] __attribute__((aligned(16)));   // background gradient down the screen

// Sprites: 12 skull colours + 3 oni skins. One RGB565 colour per pixel,
// 0 = transparent (no sprite colour is pure black), native layout, D x D.
#define NSKULL 12
#define NONI   3
#define NSPR   (NSKULL + NONI)
// Each sprite row has SPR_PAD transparent pixels before it, which also serve as
// the previous row's tail, so the PIE path can read whole 8-pixel groups past
// either end of a row.
#define SPR_PAD    8
#define SPR_STRIDE (FACE_D + SPR_PAD)
#define SPR_PLANE  (FACE_D * SPR_STRIDE)
#define SPR_BYTES  ((NSPR * SPR_PLANE + SPR_PAD + 16) * 2)
static uint16_t *s_spr;                   // colours, for opaque faces
static uint16_t *s_pre;                   // premultiplied by SEE_ALPHA/32, for see-through
static uint8_t s_span[NSPR][FACE_D][2];   // per sprite row: first and last+1 visible column
static bool s_pie_ok;                     // PIE self-test passed
#define SPR_MASK 0x07E0F81Fu
#define SEE_ALPHA 20                      // see-through opacity, out of 32

// ---------------------------------------------------------------- the faces
// 16x16 grids scaled 3x, left half stored, right half mirrored. Characters:
//   X skin (the sprite's tint)  H horn/tusk  E eye  B dark (brows, mouth)  . clear

static const uint32_t SKULL_TINTS[NSKULL] = {
    0xFF0000, 0xFF6600, 0xF5B800, 0x77EE00, 0x00A040, 0x00A0A0,
    0x00EEFF, 0x0022CC, 0x3300CC, 0x8800DD, 0xDD00DD, 0xEE1088,
};
static const uint32_t ONI_TINTS[NONI] = { 0xD8261C, 0x2A5BD8, 0x2E9E3A };   // red, blue, green

// A horned 8-bit skull: domed head, square eye sockets, nose hole, teeth.
static const char *SKULL_LEFT[16] = {
    "X.......",
    "XX......",
    ".XX.XXXX",
    "..XXXXXX",
    "..XXXXXX",
    ".XXXXXXX",
    ".XXXXXXX",
    ".XX...XX",
    ".XX...XX",
    ".XXX.XXX",
    "..XXXXXX",
    "..XXXXX.",
    "...XXXXX",
    "...X.X.X",
    "...XXXXX",
    "....XXXX",
};

// An oni: curved horns, angry brows, yellow eyes, tusks rising from an open jaw.
static const char *ONI_LEFT[16] = {
    "H.......",
    "HH......",
    ".HH.....",
    "..HH.XXX",
    "...HXXXX",
    "..XXXXXX",
    ".XBBBXXX",
    ".XXBEEXX",
    ".XXXEBXX",
    ".XXXXXXX",
    ".XXXXX.X",
    ".XXBBBBB",
    ".XXHBBBB",
    "..XHBBBB",
    "..XXXXXX",
    "...XXXXX",
};

static uint16_t rgb565(uint32_t rgb) {
    uint16_t c = (((rgb >> 16) & 0xF8) << 8) | (((rgb >> 8) & 0xFC) << 3) | ((rgb & 0xF8) >> 3);
    return c ? c : 0x0821;   // keep 0 for "transparent"
}

static void build_sprite(int id, const char **grid, uint32_t tint) {
    const int SCALE = FACE_D / 16;
    uint16_t skin = rgb565(tint), horn = rgb565(0xF0E2B6), eye = rgb565(0xFFD400), dark = rgb565(0x2A0A0A);
    for (int sr = 0; sr < FACE_D; sr++) {
        uint16_t *spr = s_spr + id * SPR_PLANE + sr * SPR_STRIDE + SPR_PAD;
        uint16_t *pre = s_pre + id * SPR_PLANE + sr * SPR_STRIDE + SPR_PAD;
        for (int sc = 0; sc < FACE_D; sc++) {
            // native (sr, sc) -> landscape (x = D-1-sr, y = sc) -> grid cell
            int gx = (FACE_D - 1 - sr) / SCALE, gy = sc / SCALE;
            char ch = grid[gy][gx < 8 ? gx : 15 - gx];
            uint32_t w = ch == 'X' ? skin : ch == 'H' ? horn : ch == 'E' ? eye : ch == 'B' ? dark : 0;
            spr[sc] = w;
            uint32_t p = ((((w & 0xF81F) * SEE_ALPHA) >> 5) & 0xF81F) | ((((w & 0x07E0) * SEE_ALPHA) >> 5) & 0x07E0);
            pre[sc] = w ? (p ? p : 1) : 0;   // 0 stays "transparent"
        }
        int lo = FACE_D, hi = 0;
        for (int sc = 0; sc < FACE_D; sc++)
            if (spr[sc]) { if (sc < lo) lo = sc; hi = sc + 1; }
        if (lo > hi) lo = hi = 0;
        s_span[id][sr][0] = lo; s_span[id][sr][1] = hi;
    }
}

// ---------------------------------------------------------------- physics

struct Face { float x, y, vx, vy; uint8_t sprite; };
static Face s_faces[MAX_FACES];

struct Snap {
    int n;
    bool see;
    int band;
    bool pie;
    int16_t row0[MAX_FACES];   // native top-left of each sprite box
    int16_t col0[MAX_FACES];
    uint8_t sprite[MAX_FACES];
};
static Snap s_snap[2];         // frame F renders from s_snap[F & 1]

static volatile int s_param_n = 100;
static volatile float s_param_speed = 1.0f;
static volatile bool s_param_see = true;
static volatile bool s_param_pie = true;
static int32_t s_phys_last = -1;

static volatile int s_band_front = 0;
static volatile int s_band_pending = -1;
static volatile int32_t s_band_latched = -1;
static volatile uint32_t s_flips = 0;

static const float FRAME_DT = 880.0f * 1312.0f / 60e6f;    // 19.24 ms

static void on_frame(int32_t F, void *) {
    float dt = s_phys_last < 0 ? 0 : (F - s_phys_last) * FRAME_DT;
    if (dt > 0.1f) dt = 0.1f;
    s_phys_last = F;
    float k = s_param_speed;
    int n = s_param_n;
    for (int i = 0; i < n; i++) {
        Face &f = s_faces[i];
        f.x += f.vx * k * dt; f.y += f.vy * k * dt;
        if (f.x < FACE_R) { f.x = FACE_R; f.vx = fabsf(f.vx); }
        if (f.x > SCREEN_W - FACE_R) { f.x = SCREEN_W - FACE_R; f.vx = -fabsf(f.vx); }
        if (f.y < FACE_R) { f.y = FACE_R; f.vy = fabsf(f.vy); }
        if (f.y > PLAY_H - FACE_R) { f.y = PLAY_H - FACE_R; f.vy = -fabsf(f.vy); }
    }
    Snap &s = s_snap[F & 1];
    s.n = n;
    s.see = s_param_see;
    s.pie = s_param_pie && s_pie_ok;
    if (s_band_pending >= 0) { s_band_front = s_band_pending; s_band_pending = -1; s_band_latched = F; }
    s.band = s_band_front;
    for (int i = 0; i < n; i++) {
        int x = (int)s_faces[i].x, y = (int)s_faces[i].y;
        s.row0[i] = (int16_t)(SCREEN_W - x - FACE_R);
        s.col0[i] = (int16_t)(y - FACE_R);
        s.sprite[i] = s_faces[i].sprite;
    }
}

// ---------------------------------------------------------------- one strip

struct CoreStats { volatile uint32_t n, copy_sum, bg_sum, face_sum, px_sum; };
static CoreStats s_cs[2];

// Hot path: built at -O2 (the Arduino default is -Os).
__attribute__((optimize("O2"))) static void on_strip(int k, int32_t F, uint16_t *dst, void *) {
    const Snap &s = s_snap[F & 1];
    const int r0 = k * BEAM_STRIP_ROWS, r1 = r0 + BEAM_STRIP_ROWS;

    // LVGL band first (Mike's ordering): if a strip runs late, only the faces glitch
    uint32_t t0 = esp_cpu_get_cycle_count();
    const uint16_t *band = s_band[s.band] + r0 * BAND_H;
    for (int r = 0; r < BEAM_STRIP_ROWS; r++) {
        if (s.pie) pie_copy(dst + r * BEAM_H_RES + PLAY_H, band + r * BAND_H, BAND_H / 8);
        else memcpy(dst + r * BEAM_H_RES + PLAY_H, band + r * BAND_H, BAND_H * 2);
    }
    CoreStats &cs = s_cs[xPortGetCoreID()];
    uint32_t t1 = esp_cpu_get_cycle_count();
    cs.copy_sum += t1 - t0;
    cs.n++;

    if (s.pie) pie_fill16(dst, s_bg, PLAY_H / 8);
    else for (int r = 0; r < BEAM_STRIP_ROWS; r++) memcpy(dst + r * BEAM_H_RES, s_bg, PLAY_H * 2);
    uint32_t t2 = esp_cpu_get_cycle_count();
    cs.bg_sum += t2 - t1;
    uint32_t px = 0;

    for (int i = 0; i < s.n; i++) {
        int fr = s.row0[i];
        if (fr >= r1 || fr + FACE_D <= r0) continue;
        int ra = fr > r0 ? fr : r0, rb = fr + FACE_D < r1 ? fr + FACE_D : r1;
        int c0 = s.col0[i];
        int ca = c0 > 0 ? c0 : 0, cb = c0 + FACE_D < PLAY_H ? c0 + FACE_D : PLAY_H;
        const int t = s.sprite[i];
        const int plane = t * SPR_PLANE + SPR_PAD - c0;   // sprite column c lives at [plane + sr*STRIDE + c]
        for (int r = ra; r < rb; r++) {
            const int sr = r - fr;
            int lo = c0 + s_span[t][sr][0], hi = c0 + s_span[t][sr][1];
            if (lo < ca) lo = ca;
            if (hi > cb) hi = cb;
            if (hi <= lo) continue;
            px += hi - lo;
            uint16_t *d = dst + (r - r0) * BEAM_H_RES;
            const int off = plane + sr * SPR_STRIDE;
            if (s.pie) {
                // whole aligned groups of 8 covering [lo, hi); the padding is transparent
                int g0 = lo & ~7, g1 = (hi + 7) & ~7;
                if (s.see) pie_blend_row(d + g0, s_pre + off + g0, (g1 - g0) >> 3);
                else pie_opaque_row(d + g0, s_spr + off + g0, (g1 - g0) >> 3);
                continue;
            }
            const uint16_t *p = s_spr + off;
            if (s.see) {
                for (int c = lo; c < hi; c++) {
                    uint32_t w = p[c];
                    if (!w) continue;
                    uint32_t f = (w | (w << 16)) & SPR_MASK, b = d[c];
                    b = (b | (b << 16)) & SPR_MASK;
                    uint32_t o = (b + (((f - b) * SEE_ALPHA) >> 5)) & SPR_MASK;
                    d[c] = (uint16_t)(o | (o >> 16));
                }
            } else {
                for (int c = lo; c < hi; c++) {
                    uint16_t w = p[c];
                    if (w) d[c] = w;
                }
            }
        }
    }
    cs.face_sum += esp_cpu_get_cycle_count() - t2;
    cs.px_sum += px;
}

// ---------------------------------------------------------------- PIE self-test
// Run the vector paths against plain C on random pixels at every source alignment.
// If anything differs, the strips stay on the C path.

static uint16_t ref_blend(uint16_t b, uint16_t pre) {
    if (!pre) return b;
    uint32_t rb = (((b & 0xF81Fu) * (32 - SEE_ALPHA)) >> 5) & 0xF81Fu;
    uint32_t g = (((b & 0x07E0u) * (32 - SEE_ALPHA)) >> 5) & 0x07E0u;
    return (uint16_t)((rb | g) + pre);
}

static bool pie_selftest() {
    static uint16_t d[32] __attribute__((aligned(16))), want[32], src[48] __attribute__((aligned(16)));
    uint32_t x = 0x12345678;
    auto rnd = [&]() { x ^= x << 13; x ^= x >> 17; x ^= x << 5; return x; };
    int bad = 0;
    for (int iter = 0; iter < 64; iter++) {
        // random colours, a quarter transparent; for the blend, premultiplied like the sprites
        uint16_t col[48], pre[48];
        for (int i = 0; i < 48; i++) {
            uint32_t v = rnd(), w = (v & 3) == 0 ? 0 : ((v >> 8) & 0xFFFF) | 1;
            uint32_t p = ((((w & 0xF81F) * SEE_ALPHA) >> 5) & 0xF81F) | ((((w & 0x07E0) * SEE_ALPHA) >> 5) & 0x07E0);
            col[i] = w; pre[i] = w ? (p ? p : 1) : 0;
        }
        int a = iter & 7;
        for (int see = 0; see < 2; see++) {
            memcpy(src, see ? pre : col, sizeof col);
            for (int i = 0; i < 32; i++) d[i] = want[i] = (uint16_t)rnd();
            for (int i = 0; i < 24; i++) {
                uint16_t s = src[a + i];
                want[i] = see ? ref_blend(want[i], s) : (s ? s : want[i]);
            }
            if (see) pie_blend_row(d, src + a, 3);
            else pie_opaque_row(d, src + a, 3);
            for (int i = 0; i < 32; i++) if (d[i] != want[i]) {
                if (bad < 6) printf("[pie] %s mismatch align %d px %d: got %04x want %04x\n",
                                    see ? "blend" : "opaque", a, i, d[i], want[i]);
                bad++;
            }
        }
    }
    uint16_t *big = (uint16_t *)heap_caps_aligned_calloc(16, 1, BEAM_STRIP_BYTES, MALLOC_CAP_INTERNAL);
    if (!big) return false;
    pie_fill16(big, s_bg, PLAY_H / 8);
    for (int r = 0; r < 16; r++) {
        if (memcmp(big + r * BEAM_H_RES, s_bg, PLAY_H * 2)) { bad++; printf("[pie] fill row %d wrong\n", r); }
        for (int c = PLAY_H; c < BEAM_H_RES; c++) if (big[r * BEAM_H_RES + c]) { bad++; printf("[pie] fill overran\n"); break; }
    }
    pie_copy(big, s_bg, PLAY_H / 8);
    if (memcmp(big, s_bg, PLAY_H * 2)) { bad++; printf("[pie] copy wrong\n"); }
    heap_caps_free(big);
    printf("[pie] self-test %s (%d mismatches)\n", bad ? "FAILED, using plain C" : "passed", bad);
    return bad == 0;
}

// ---------------------------------------------------------------- public

bool render_init() {
    for (int i = 0; i < 2; i++) s_band[i] = (uint16_t *)heap_caps_aligned_calloc(128, 1, BAND_BYTES, MALLOC_CAP_SPIRAM);
    for (uint16_t **b : { &s_pre, &s_spr }) {   // the see-through table first: it's the hot one
        *b = (uint16_t *)heap_caps_aligned_calloc(16, 1, SPR_BYTES, MALLOC_CAP_INTERNAL);
        if (!*b) *b = (uint16_t *)heap_caps_aligned_calloc(16, 1, SPR_BYTES, MALLOC_CAP_SPIRAM);
    }
    if (!s_band[0] || !s_band[1] || !s_spr || !s_pre) return false;
    printf("[faces] sprites: see-through in %s, opaque in %s\n", esp_ptr_internal(s_pre) ? "internal RAM" : "PSRAM",
           esp_ptr_internal(s_spr) ? "internal RAM" : "PSRAM");

    for (int t = 0; t < NSKULL; t++) build_sprite(t, SKULL_LEFT, SKULL_TINTS[t]);
    for (int t = 0; t < NONI; t++) build_sprite(NSKULL + t, ONI_LEFT, ONI_TINTS[t]);
    for (int y = 0; y < PLAY_H; y++) {
        float t = y / (float)(PLAY_H - 1);
        int r = 34 + t * 34, g = 26 + t * 22, b = 128 + t * 74;
        s_bg[y] = ((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3);
    }
    for (int i = 0; i < MAX_FACES; i++) {
        Face &f = s_faces[i];
        f.x = FACE_R + random(SCREEN_W - FACE_D);
        f.y = FACE_R + random(PLAY_H - FACE_D);
        float ang = random(3600) * (float)M_PI / 1800.0f, sp = 60 + random(90);
        f.vx = cosf(ang) * sp; f.vy = sinf(ang) * sp;
        // alternate skulls and oni through the crowd
        f.sprite = (i & 1) ? NSKULL + (i / 2) % NONI : (i / 2) % NSKULL;
    }
    s_pie_ok = pie_selftest();
    return true;
}

bool render_start() {
    BeamRaceConfig cfg = { on_frame, on_strip, nullptr };
    return beam_race_begin(cfg);
}

void render_set_pie(bool on) { s_param_pie = on; }
bool render_pie() { return s_param_pie && s_pie_ok; }

void render_set_params(int faces, float speed, bool see) {
    s_param_n = faces < 1 ? 1 : (faces > MAX_FACES ? MAX_FACES : faces);
    s_param_speed = speed;
    s_param_see = see;
}

void render_get_stats(RenderStats *o) {
    BeamRaceStats b;
    beam_race_stats(&b);
    static uint32_t last_ms, last_flips;
    uint32_t now = millis(), flips = s_flips;
    o->flips = last_ms ? (flips - last_flips) * 1000.0f / (now - last_ms) : 0;
    last_ms = now; last_flips = flips;
    uint32_t n = 0, copy = 0, bg = 0, face = 0, px = 0;
    for (int c = 0; c < 2; c++) {
        CoreStats &cs = s_cs[c];
        n += cs.n; copy += cs.copy_sum; bg += cs.bg_sum; face += cs.face_sum; px += cs.px_sum;
        cs.n = cs.copy_sum = cs.bg_sum = cs.face_sum = cs.px_sum = 0;
    }
    float mhz = getCpuFrequencyMhz();
    o->bg_avg_us = n ? bg / mhz / n : 0;
    o->face_avg_us = n ? face / mhz / n : 0;
    o->px_avg = n ? px / (float)n : 0;
    o->fps = b.fps;
    o->strip_avg_us = b.strip_avg_us;
    o->strip_max_us = b.strip_max_us;
    o->budget_us = b.budget_us;
    o->copy_avg_us = n ? copy / mhz / n : 0;
    o->late = b.late; o->skipped = b.skipped; o->naps = b.naps; o->underruns = b.underruns;
}

uint16_t *band_back() { return s_band[s_band_front ^ 1]; }
uint16_t *band_front() { return s_band[s_band_front]; }

void band_flip_and_wait() {
    // No cache write-back: only the CPUs read the band (the strips copy it), and
    // the P4's two cores share the L1 data cache.
    s_band_pending = s_band_front ^ 1;
    while (s_band_pending >= 0) vTaskDelay(1);
    int32_t L = s_band_latched;
    // the old front was last used by frame L-1; once frame L+1 is set up, it's done
    while (beam_race_frame_done() < L + 1) vTaskDelay(1);
    s_flips++;
}
