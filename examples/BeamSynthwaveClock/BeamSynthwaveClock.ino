// BeamSynthwaveClock: a synthwave clock face with weather, racing the beam on the
// JC8012 (ESP32-P4), landscape.
//
// No framebuffer: every 16-column strip of the scene (sky, stars, striped sun,
// mountains, scrolling neon grid, the time, weather) is drawn just before the
// display DMA sends it (beam_race). LVGL is only a rasterizer here: it draws the
// time into a canvas once a minute, as neon 7-segment digits, and the scene
// blends that in.
//
// The weather plays out in the scene, in chunky 8-bit pixels: rain that splashes on
// the grid (heavier when pouring), storms with forked lightning, snow, fog banks, or
// clear skies with shooting stars. The digits take the weather's colours. Night
// mode sets the sun and dims everything.
//
// Tap the screen to cycle the weather (clear, rain, pouring, storm, snow, fog).
// Serial0 (115200): "w clear|rain|pour|storm|snow|fog", "night on|off", "shoot",
// "h1234" sets the time to 12:34. The clock starts at the build time; there's no
// network in this example (a networked version runs a kitchen clock).

#include <Arduino.h>
#include "lvgl.h"
#include "beam_race.h"
#include "gsl3680_touch.h"
#include "pins_config.h"
#include "scene.h"

#define CANVAS_W 1200
#define CANVAS_H 250

static JC8012P4A1C_Board board;
static gsl3680_touch touch(board.Wire, TP_RST, TP_INT);
static lv_obj_t *canvas;
static uint16_t *canvas_buf;
static int canvas_stride;           // pixels
static int32_t clock_offset;        // seconds added to the build time

// The tap / serial weather cycle: scene weather, heavy, storm
struct Look { const char *name; int w; bool heavy, storm; };
static const Look LOOKS[] = {
    { "clear", WEATHER_CLEAR, false, false }, { "rain", WEATHER_RAIN, false, false },
    { "pour", WEATHER_RAIN, true, false },    { "storm", WEATHER_RAIN, true, true },
    { "snow", WEATHER_SNOW, false, false },   { "fog", WEATHER_FOG, false, false },
};
static const int NLOOKS = sizeof LOOKS / sizeof LOOKS[0];
static int s_look = 0;
static bool s_night = false;

static void set_look(int i) {
    s_look = ((i % NLOOKS) + NLOOKS) % NLOOKS;
    scene_set_weather(LOOKS[s_look].w, LOOKS[s_look].heavy, LOOKS[s_look].storm);
    Serial0.printf("[synth] weather: %s\n", LOOKS[s_look].name);
}

// ---------------------------------------------------------------- 7-segment digits

#define SEG_W 132                   // digit width
#define SEG_H 230                   // digit height
#define SEG_T 26                    // bar thickness
#define SEG_GAP 30                  // between digits
#define SEG_TOP 10

//                            a     b     c     d     e     f     g
static const uint8_t SEGS[10] = {
    0b0111111, 0b0000110, 0b1011011, 0b1001111, 0b1100110,
    0b1101101, 0b1111101, 0b0000111, 0b1111111, 0b1101111,
};

static void bar(lv_layer_t *layer, int x1, int y1, int x2, int y2) {
    lv_draw_rect_dsc_t rd;
    lv_draw_rect_dsc_init(&rd);
    rd.bg_color = lv_color_white();
    rd.bg_opa = LV_OPA_COVER;
    rd.radius = SEG_T / 2;
    lv_area_t a = { x1, y1, x2, y2 };
    lv_draw_rect(layer, &rd, &a);
}

static void draw_digit(lv_layer_t *layer, int x, int d) {
    const int T = SEG_T, W = SEG_W, H = SEG_H, y = SEG_TOP, mid = y + H / 2, in = 3;   // in: gap at the joints
    uint8_t s = SEGS[d];
    if (s & 0x01) bar(layer, x + T / 2 + in, y, x + W - T / 2 - in, y + T);                     // a
    if (s & 0x02) bar(layer, x + W - T, y + T / 2 + in, x + W, mid - in);                       // b
    if (s & 0x04) bar(layer, x + W - T, mid + in, x + W, y + H - T / 2 - in);                   // c
    if (s & 0x08) bar(layer, x + T / 2 + in, y + H - T, x + W - T / 2 - in, y + H);             // d
    if (s & 0x10) bar(layer, x, mid + in, x + T, y + H - T / 2 - in);                           // e
    if (s & 0x20) bar(layer, x, y + T / 2 + in, x + T, mid - in);                               // f
    if (s & 0x40) bar(layer, x + T / 2 + in, mid - T / 2, x + W - T / 2 - in, mid + T / 2);     // g
}

// Draw "h:mm" white on black and hand it to the scene.
static void render_time(int h, int m) {
    const int COLON = 70, DOT = 28;
    int nh = h >= 10 ? 2 : 1;
    int wh = nh * SEG_W + (nh - 1) * SEG_GAP, wm = 2 * SEG_W + SEG_GAP, total = wh + COLON + wm;

    lv_canvas_fill_bg(canvas, lv_color_black(), LV_OPA_COVER);
    lv_layer_t layer;
    lv_canvas_init_layer(canvas, &layer);
    int x = 0;
    if (nh == 2) { draw_digit(&layer, x, h / 10); x += SEG_W + SEG_GAP; }
    draw_digit(&layer, x, h % 10);
    x = wh + COLON;
    draw_digit(&layer, x, m / 10);
    draw_digit(&layer, x + SEG_W + SEG_GAP, m % 10);
    int cx = wh + COLON / 2;
    bar(&layer, cx - DOT / 2, SEG_TOP + 60, cx + DOT / 2, SEG_TOP + 60 + DOT);
    bar(&layer, cx - DOT / 2, SEG_TOP + SEG_H - 60 - DOT, cx + DOT / 2, SEG_TOP + SEG_H - 60);
    lv_canvas_finish_layer(canvas, &layer);

    scene_set_digits(canvas_buf, total, CANVAS_H, canvas_stride);
}

// ---------------------------------------------------------------- the clock

static uint32_t build_seconds() {   // __TIME__ is "HH:MM:SS"
    const char *t = __TIME__;
    return ((t[0] - '0') * 10 + (t[1] - '0')) * 3600 + ((t[3] - '0') * 10 + (t[4] - '0')) * 60 + (t[6] - '0') * 10 + (t[7] - '0');
}

static void now_hm(int *h12, int *m) {
    uint32_t s = (build_seconds() + clock_offset + millis() / 1000) % 86400;
    int h = s / 3600;
    *m = (s / 60) % 60;
    *h12 = h % 12 == 0 ? 12 : h % 12;
}

static uint32_t tick_cb(void) { return millis(); }

static void serial_commands() {
    static char line[16];
    static int len = 0;
    while (Serial0.available()) {
        char ch = Serial0.read();
        if (ch != '\n' && ch != '\r') { if (len < 15) line[len++] = ch; continue; }
        line[len] = 0;
        char *c = line;
        while (*c && !strchr("wnsh", *c)) c++;
        if (!strncmp(c, "w ", 2)) {
            for (int i = 0; i < NLOOKS; i++) if (!strcmp(c + 2, LOOKS[i].name)) set_look(i);
        } else if (!strncmp(c, "night ", 6)) {
            s_night = !strcmp(c + 6, "on");
            scene_set_night(s_night);
            Serial0.printf("[synth] night %s\n", s_night ? "on" : "off");
        } else if (!strcmp(c, "shoot")) {
            scene_shoot();
        } else if (*c == 'h' && strlen(c) >= 4) {
            int v = atoi(c + 1), hh = v / 100, mm = v % 100;
            uint32_t want = hh * 3600 + mm * 60;
            uint32_t have = (build_seconds() + millis() / 1000) % 86400;
            clock_offset = (int32_t)want - (int32_t)have;
            Serial0.printf("[synth] time set to %d:%02d\n", hh, mm);
        }
        len = 0;
    }
}

void setup() {
    Serial0.setTxBufferSize(4096);
    Serial0.begin(115200);
    delay(200);
    Serial0.println("\n[synth] start");

    beam_race_reserve();            // the strip ring needs one big internal block: take it first
    if (!scene_init()) { Serial0.println("[synth] out of memory"); while (1) delay(1000); }

    // LVGL just as a rasterizer: a display that is never refreshed, and one canvas
    lv_init();
    lv_tick_set_cb(tick_cb);
    lv_display_create(1280, 800);
    canvas_stride = lv_draw_buf_width_to_stride(CANVAS_W, LV_COLOR_FORMAT_RGB565) / 2;
    canvas_buf = (uint16_t *)heap_caps_aligned_alloc(64, canvas_stride * 2 * CANVAS_H, MALLOC_CAP_SPIRAM);
    canvas = lv_canvas_create(lv_screen_active());
    lv_canvas_set_buffer(canvas, canvas_buf, CANVAS_W, CANVAS_H, LV_COLOR_FORMAT_RGB565);

    int h, m;
    now_hm(&h, &m);
    render_time(h, m);              // ready before the first frame

    BeamRaceConfig cfg = { scene_on_frame, scene_on_strip, nullptr };
    if (!beam_race_begin(cfg)) { Serial0.println("[synth] display start failed"); while (1) delay(1000); }
    board.begin();
    touch.begin();
    Serial0.printf("[synth] running; free internal %u, PSRAM %u\n",
                   (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL), (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
}

void loop() {
    serial_commands();

    // tap to cycle the weather
    static bool was_down = false;
    static uint32_t last_touch = 0;
    if (millis() - last_touch > 25) {
        last_touch = millis();
        uint16_t tx, ty;
        bool down = touch.getTouch(&tx, &ty);
        if (down && !was_down) set_look(s_look + 1);
        was_down = down;
    }

    // new minute: redraw the time
    static int shown = -1;
    int h, m;
    now_hm(&h, &m);
    if (h * 60 + m != shown) {
        shown = h * 60 + m;
        render_time(h, m);
    }

    static uint32_t last_stats = 0;
    if (millis() - last_stats > 2000) {
        last_stats = millis();
        BeamRaceStats s;
        beam_race_stats(&s);
        Serial0.printf("[synth] %.1f fps | strip %.0f of %.0f us (worst %.0f) | frame setup %.0f us | late %lu skipped %lu naps %lu underruns %lu | %s\n",
                       s.fps, s.strip_avg_us, s.budget_us, s.strip_max_us, s.frame_max_us, (unsigned long)s.late,
                       (unsigned long)s.skipped, (unsigned long)s.naps, (unsigned long)s.underruns, LOOKS[s_look].name);
        scene_prof_print();
    }
    delay(10);
}
