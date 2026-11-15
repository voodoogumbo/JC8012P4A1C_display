// BeamOniFaces: racing-the-beam stress test on the JC8012 (ESP32-P4), landscape.
//
// The top of the screen (landscape y 0..623) has no framebuffer at all: every
// 16-column slice is drawn by the CPU just before the display DMA sends it
// (beam_race). The bottom band (y 624..799) is a normal LVGL screen whose pixels
// get copied into each slice on the way out. LVGL only knows about two sliders,
// a switch and some text. Up to 250 faces: horned skulls and oni.
//
// Serial0 (115200): "n200" sets the face count, "s3.5" the speed, "t" toggles see-through.

#include <Arduino.h>
#include "lvgl.h"
#include "beam_race.h"
#include "gsl3680_touch.h"
#include "pins_config.h"
#include "render.h"
#include "layout.h"

static JC8012P4A1C_Board board;
static gsl3680_touch touch(board.Wire, TP_RST, TP_INT);

static lv_obj_t *lbl_faces, *lbl_speed, *lbl_stats, *sld_faces, *sld_speed, *sw_see;

// ---------------------------------------------------------------- LVGL plumbing

// PARTIAL render mode: LVGL hands us only the areas that changed. Rotate each into
// the back band buffer (native row = 1279 - x, column = y) and remember it. On the
// last area of a refresh, publish the buffer, then copy the same areas from the new
// front into the new back so both buffers hold the full current band (Mike's
// dirty-rect sync), and LVGL never has to redraw what didn't change.
static lv_area_t s_dirty[32];
static int s_ndirty;
static bool s_dirty_all;

static void copy_area_native(uint16_t *dst, const uint16_t *src, const lv_area_t &a) {
    size_t bytes = (a.y2 - a.y1 + 1) * 2;
    for (int x = a.x1; x <= a.x2; x++) {
        size_t off = (size_t)(SCREEN_W - 1 - x) * BAND_H + a.y1;
        memcpy(dst + off, src + off, bytes);
    }
}

static void flush_cb(lv_display_t *d, const lv_area_t *area, uint8_t *px) {
    const uint16_t *src = (const uint16_t *)px;
    uint16_t *dst = band_back();
    int w = area->x2 - area->x1 + 1, h = area->y2 - area->y1 + 1;
    for (int x = area->x1; x <= area->x2; x++) {
        uint16_t *row = dst + (SCREEN_W - 1 - x) * BAND_H + area->y1;
        const uint16_t *s = src + (x - area->x1);
        for (int y = 0; y < h; y++) row[y] = s[y * w];
    }
    if (s_ndirty < 32) s_dirty[s_ndirty++] = *area;
    else s_dirty_all = true;

    if (lv_display_flush_is_last(d)) {
        band_flip_and_wait();
        uint16_t *front = band_front(), *back = band_back();
        if (s_dirty_all) memcpy(back, front, BAND_BYTES);
        else for (int i = 0; i < s_ndirty; i++) copy_area_native(back, front, s_dirty[i]);
        s_ndirty = 0;
        s_dirty_all = false;
    }
    lv_display_flush_ready(d);
}

// Touch arrives in panel space (800 x 1280). A press that starts in the face
// area is ignored; a drag that started on a slider keeps working above the band.
static void touch_cb(lv_indev_t *indev, lv_indev_data_t *data) {
    static bool active = false;
    uint16_t tx, ty;
    if (touch.getTouch(&tx, &ty)) {
        int x = SCREEN_W - 1 - ty, y = tx;
        if (!active && y < BAND_Y0) { data->state = LV_INDEV_STATE_RELEASED; return; }
        active = true;
        data->point.x = x;
        data->point.y = y < BAND_Y0 ? 0 : y - BAND_Y0;
        data->state = LV_INDEV_STATE_PRESSED;
    } else {
        active = false;
        data->state = LV_INDEV_STATE_RELEASED;
    }
}

static uint32_t tick_cb(void) { return millis(); }

// ---------------------------------------------------------------- the band UI

static void apply_params() {
    int n = lv_slider_get_value(sld_faces);
    float sp = lv_slider_get_value(sld_speed) / 100.0f;
    bool see = lv_obj_has_state(sw_see, LV_STATE_CHECKED);
    render_set_params(n, sp, see);
    lv_label_set_text_fmt(lbl_faces, "Faces: %d", n);
    char buf[32];
    snprintf(buf, sizeof buf, "Speed: %.2fx", sp);
    lv_label_set_text(lbl_speed, buf);
}

static void on_change(lv_event_t *e) { apply_params(); }

static void update_stats(lv_timer_t *t) {
    RenderStats s;
    render_get_stats(&s);
    char buf[240];
    snprintf(buf, sizeof buf,
             "%.1f FPS (panel 52.0 Hz) | LVGL %.0f flips/s\n"
             "strip %.0f of %.0f us (worst %.0f) | band copy %.0f us\n"
             "late %lu | skipped %lu | naps %lu | underruns %lu",
             s.fps, s.flips, s.strip_avg_us, s.budget_us, s.strip_max_us, s.copy_avg_us,
             (unsigned long)s.late, (unsigned long)s.skipped, (unsigned long)s.naps,
             (unsigned long)s.underruns);
    lv_label_set_text(lbl_stats, buf);
    Serial0.println(buf);
}

static lv_obj_t *make_slider(lv_obj_t *parent, int y, int min, int max, int val) {
    lv_obj_t *s = lv_slider_create(parent);
    lv_obj_set_pos(s, 170, y);
    lv_obj_set_size(s, 520, 14);
    lv_slider_set_range(s, min, max);
    lv_slider_set_value(s, val, LV_ANIM_OFF);
    lv_obj_set_style_bg_color(s, lv_color_hex(0x2A2670), LV_PART_MAIN);
    lv_obj_set_style_bg_color(s, lv_color_hex(0x5B54F0), LV_PART_INDICATOR);
    lv_obj_set_style_bg_color(s, lv_color_hex(0x5B54F0), LV_PART_KNOB);
    lv_obj_set_style_pad_all(s, 6, LV_PART_KNOB);
    lv_obj_set_ext_click_area(s, 18);
    lv_obj_add_event_cb(s, on_change, LV_EVENT_VALUE_CHANGED, NULL);
    return s;
}

static lv_obj_t *make_label(lv_obj_t *parent, int x, int y, const lv_font_t *font, uint32_t color) {
    lv_obj_t *l = lv_label_create(parent);
    lv_obj_set_pos(l, x, y);
    lv_obj_set_style_text_font(l, font, 0);
    lv_obj_set_style_text_color(l, lv_color_hex(color), 0);
    return l;
}

static void build_ui() {
    lv_obj_t *scr = lv_screen_active();
    lv_obj_set_style_bg_color(scr, lv_color_hex(0x0E0C2E), 0);
    lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);
    lv_obj_set_style_border_side(scr, LV_BORDER_SIDE_TOP, 0);
    lv_obj_set_style_border_width(scr, 2, 0);
    lv_obj_set_style_border_color(scr, lv_color_hex(0x3B36A8), 0);
    lv_obj_remove_flag(scr, LV_OBJ_FLAG_SCROLLABLE);

    lbl_faces = make_label(scr, 24, 18, &lv_font_montserrat_16, 0xC9C6FF);
    sld_faces = make_slider(scr, 22, 1, MAX_FACES, 100);
    lbl_speed = make_label(scr, 24, 62, &lv_font_montserrat_16, 0xC9C6FF);
    sld_speed = make_slider(scr, 66, 25, 500, 100);

    sw_see = lv_switch_create(scr);
    lv_obj_set_pos(sw_see, 730, 24);
    lv_obj_set_size(sw_see, 56, 28);
    lv_obj_add_state(sw_see, LV_STATE_CHECKED);
    lv_obj_set_style_bg_color(sw_see, lv_color_hex(0x5B54F0), LV_PART_INDICATOR | LV_STATE_CHECKED);
    lv_obj_add_event_cb(sw_see, on_change, LV_EVENT_VALUE_CHANGED, NULL);
    make_label(scr, 712, 60, &lv_font_montserrat_16, 0xC9C6FF);
    lv_label_set_text(lv_obj_get_child(scr, -1), "See-through");

    lbl_stats = make_label(scr, 830, 14, &lv_font_montserrat_16, 0xE6E4FF);
    lv_obj_set_width(lbl_stats, 430);
    lv_obj_set_style_text_align(lbl_stats, LV_TEXT_ALIGN_RIGHT, 0);
    lv_label_set_text(lbl_stats, "");

    lv_obj_t *cap = make_label(scr, 24, 116, &lv_font_montserrat_16, 0x7E7AB8);
    lv_obj_set_width(cap, 1232);
    lv_label_set_long_mode(cap, LV_LABEL_LONG_WRAP);
    lv_label_set_text(cap,
        "The panel scans this screen's columns, so every 16-column slice is put together as it is "
        "scanned: the faces drawn by the CPU, this strip copied in from LVGL's buffer.");

    apply_params();
    lv_timer_create(update_stats, 500, NULL);
}

// ---------------------------------------------------------------- setup / loop

void setup() {
    // Big TX buffer: prints return at once instead of busy-waiting on the UART,
    // which matters because they run in the high-priority LVGL task.
    Serial0.setTxBufferSize(4096);
    Serial0.begin(115200);
    delay(200);
    Serial0.println("\n[faces] start");

    beam_race_reserve();            // the strip ring needs one big internal block: take it first
    if (!render_init()) { Serial0.println("[faces] out of memory"); while (1) delay(1000); }
    board.begin();
    touch.begin();

    lv_init();
    lv_tick_set_cb(tick_cb);
    lv_display_t *disp = lv_display_create(SCREEN_W, BAND_H);
    lv_display_set_color_format(disp, LV_COLOR_FORMAT_RGB565);
    void *buf = heap_caps_aligned_alloc(128, BAND_BYTES, MALLOC_CAP_SPIRAM);
    lv_display_set_buffers(disp, buf, NULL, BAND_BYTES, LV_DISPLAY_RENDER_MODE_PARTIAL);
    lv_timer_set_period(lv_display_get_refr_timer(disp), 16);   // ~60 Hz redraws
    lv_display_set_flush_cb(disp, flush_cb);
    lv_indev_t *indev = lv_indev_create();
    lv_indev_set_type(indev, LV_INDEV_TYPE_POINTER);
    lv_indev_set_read_cb(indev, touch_cb);
    lv_timer_set_period(lv_indev_get_read_timer(indev), 10);    // touch at 100 Hz
    build_ui();

    if (!render_start()) { Serial0.println("[faces] display start failed"); while (1) delay(1000); }
    // LVGL stays below the strip renderers. Tried it above them (2026-11-15): some of
    // its refresh steps run for milliseconds, longer than the renderers' lead, and
    // strips went late even at 100 faces. Partial updates keep its work small enough
    // to fit in the renderers' idle time instead.
    Serial0.printf("[faces] rendering; free internal %u, free PSRAM %u\n",
                   (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
                   (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
}

// Serial test hooks: "n200" sets the face count, "s3.5" the speed, "t" toggles see-through.
static void serial_commands() {
    static char line[16];
    static int len = 0;
    while (Serial0.available()) {
        char ch = Serial0.read();
        if (ch != '\n' && ch != '\r') { if (len < 15) line[len++] = ch; continue; }
        line[len] = 0;
        char *cmd = line;
        while (*cmd && !strchr("nst", *cmd)) cmd++;    // skip any line noise
        memmove(line, cmd, strlen(cmd) + 1);
        if (line[0]) Serial0.printf("[cmd] %s\n", line);
        if (line[0] == 'n') lv_slider_set_value(sld_faces, atoi(line + 1), LV_ANIM_OFF);
        else if (line[0] == 's') lv_slider_set_value(sld_speed, (int)(atof(line + 1) * 100), LV_ANIM_OFF);
        else if (line[0] == 't') {
            if (lv_obj_has_state(sw_see, LV_STATE_CHECKED)) lv_obj_remove_state(sw_see, LV_STATE_CHECKED);
            else lv_obj_add_state(sw_see, LV_STATE_CHECKED);
        }
        if (len) apply_params();
        len = 0;
    }
}

void loop() {
    serial_commands();
    lv_timer_handler();
    delay(5);
}
