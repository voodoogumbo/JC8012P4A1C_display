// beam_race: see beam_race.h. Lifted from the faces stress test.

#include <Arduino.h>
#include "esp_heap_caps.h"
#include "esp_cache.h"
#include "esp_cpu.h"
#include "esp_timer.h"
#include "driver/gptimer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "beam_race.h"

// 80 strips / 8 -> 10 equal DMA blocks (the bridge needs equal blocks). In PSRAM, keep
// it at 8: 205 KB stays in the P4's 256 KB L2 cache. 16 slots (409 KB) didn't, every
// strip had to be fetched back from PSRAM before it was drawn, and strips went from
// ~290 to ~495 us (2026-11-15).
#define RING_SLOTS 8

static BeamRaceConfig s_cfg;
static uint16_t *s_ring;
static TaskHandle_t s_task[2];
static TaskHandle_t s_frame_task;           // sets up frame F+1 while frame F scans out
static volatile int32_t s_frame_req = -1;
static int64_t s_next_job;
static gptimer_handle_t s_timer;
static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;

static volatile int32_t s_frame_done = -1;
static int32_t s_frame_claim = -1;

struct CoreStats { volatile uint32_t n, cyc_max; volatile uint64_t cyc_sum; };   // 64-bit: 10 s of strips overflow 32
static CoreStats s_cs[2];
static volatile uint32_t s_late, s_skipped, s_naps;
static volatile uint32_t s_frame_cyc_max;    // longest on_frame, in cycles

static const float FRAME_DT = 880.0f * 1312.0f / 60e6f;   // 19.24 ms

// The strip that last used this strip's ring slot; it must be read out first.
static int64_t prev_user(int64_t J) {
    int k = J % BEAM_STRIPS;
    if (k >= RING_SLOTS) return J - RING_SLOTS;
    int kprev = k + RING_SLOTS * ((BEAM_STRIPS - 1 - k) / RING_SLOTS);
    return J - k - BEAM_STRIPS + kprev;
}

// Whoever first needs frame F runs on_frame(F); anyone else waits for it.
static void ensure_frame(int32_t F) {
    if (s_frame_done >= F) return;
    bool mine = false;
    portENTER_CRITICAL(&s_mux);
    if (s_frame_claim < F) { s_frame_claim = F; mine = true; }
    portEXIT_CRITICAL(&s_mux);
    if (mine) {
        uint32_t t0 = esp_cpu_get_cycle_count();
        if (s_cfg.on_frame) s_cfg.on_frame(F, s_cfg.ctx);
        uint32_t c = esp_cpu_get_cycle_count() - t0;
        if (c > s_frame_cyc_max) s_frame_cyc_max = c;
        s_frame_done = F;
    } else {
        // the frame task (lower priority, maybe on this core) is on it: block so it can run
        while (s_frame_done < F) vTaskDelay(1);
    }
}

// Per-frame setup off the renderers' critical path: run inline, a long on_frame
// stopped one core drawing for its whole length and strips went late (a 2 ms setup
// cost several strips a frame). This task does it in the renderers' idle time.
static void frame_task(void *) {
    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        int32_t F = s_frame_req;
        if (F >= 0) ensure_frame(F);
    }
}

// 120 us tick: wake the render tasks to re-check the scan position.
static bool IRAM_ATTR on_tick(gptimer_handle_t t, const gptimer_alarm_event_data_t *e, void *u) {
    BaseType_t hp = pdFALSE;
    vTaskNotifyGiveFromISR(s_task[0], &hp);
    vTaskNotifyGiveFromISR(s_task[1], &hp);
    return hp == pdTRUE;
}

static void race_task(void *arg) {
    CoreStats &cs = s_cs[(int)(intptr_t)arg];
    int64_t rested = esp_timer_get_time();
    for (;;) {
        // Overloaded we would never block: starves the idle task (watchdog reset)
        // and LVGL. After 100 ms of nonstop work, nap a tick.
        if (esp_timer_get_time() - rested > 100000) {
            vTaskDelay(1);
            rested = esp_timer_get_time();
            s_naps++;
        }

        int64_t J;
        portENTER_CRITICAL(&s_mux);
        J = s_next_job++;
        portEXIT_CRITICAL(&s_mux);

        int64_t P = prev_user(J), cur;
        while ((cur = beam_cur_strip()) <= P) { ulTaskNotifyTake(pdTRUE, 1); rested = esp_timer_get_time(); }
        if (cur >= J) {
            // DMA already on this strip: restart a few strips ahead to regain lead
            portENTER_CRITICAL(&s_mux);
            if (s_next_job <= cur + 2) s_next_job = cur + 3;
            portEXIT_CRITICAL(&s_mux);
            s_skipped++;
            continue;
        }

        int32_t F = (int32_t)(J / BEAM_STRIPS);
        int k = (int)(J % BEAM_STRIPS);
        ensure_frame(F);

        uint32_t t0 = esp_cpu_get_cycle_count();
        uint16_t *dst = s_ring + (k % RING_SLOTS) * BEAM_STRIP_PX;
        s_cfg.on_strip(k, F, dst, s_cfg.ctx);
        esp_cache_msync(dst, BEAM_STRIP_BYTES, ESP_CACHE_MSYNC_FLAG_DIR_C2M);
        uint32_t cyc = esp_cpu_get_cycle_count() - t0;

        if (beam_cur_strip() >= J) s_late++;
        cs.n++; cs.cyc_sum += cyc;
        if (cyc > cs.cyc_max) cs.cyc_max = cyc;

        // Set up the next frame mid-frame, where there's slack. Done lazily at the
        // frame's first strip, it stalled both cores right where the frame starts
        // (the right edge in landscape) and those strips came out late. Safe: every
        // strip of frame F-1, whose state buffer it reuses, is finished by now.
        if (k == BEAM_STRIPS / 2 && s_frame_req < F + 1) {
            s_frame_req = F + 1;
            xTaskNotifyGive(s_frame_task);
        }
    }
}

bool beam_race_reserve(bool internal) {
    if (s_ring) return true;
    size_t ring_bytes = (size_t)RING_SLOTS * BEAM_STRIP_BYTES;
    if (internal) s_ring = (uint16_t *)heap_caps_aligned_calloc(128, 1, ring_bytes, MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA | MALLOC_CAP_8BIT);
    bool sram = s_ring != nullptr;
    if (!s_ring) s_ring = (uint16_t *)heap_caps_aligned_calloc(128, 1, ring_bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_DMA);
    if (!s_ring) return false;
    esp_cache_msync(s_ring, ring_bytes, ESP_CACHE_MSYNC_FLAG_DIR_C2M);
    printf("[beam] strip ring: %u bytes in %s%s\n", (unsigned)ring_bytes, sram ? "internal RAM" : "PSRAM",
           sram || !internal ? "" : " (SLOW: call beam_race_reserve() before other big internal allocations)");
    return true;
}

bool beam_race_begin(const BeamRaceConfig &cfg) {
    s_cfg = cfg;
    if (!beam_race_reserve()) return false;

    beam_block_t blocks[BEAM_STRIPS / RING_SLOTS];
    int nb = 0;
    for (int s = 0; s < BEAM_STRIPS; s += RING_SLOTS) blocks[nb++] = { (uint16_t)s, RING_SLOTS, s_ring };
    if (beam_dpi_init(blocks, nb) != ESP_OK || beam_dpi_start() != ESP_OK) return false;

    while (beam_frame_count() < 2) delay(5);
    s_next_job = beam_cur_strip() + 1;
    // Priority 24, above the Wi-Fi link's SDIO/RPC tasks (unpinned, 23). Below them,
    // every Wi-Fi burst preempted a renderer mid-strip; level with them, time slicing
    // still handed them whole ticks. Above, Wi-Fi runs whenever a renderer is waiting
    // for the beam (each core idles about a third of the time), and the overload nap
    // still guarantees it a tick every 100 ms.
    // One below the renderers, on core 1: it runs whenever that core's renderer waits
    // for the beam and never preempts one mid-strip (at equal priority, time slicing
    // did, for whole ticks). A renderer that needs the frame before it's ready blocks.
    xTaskCreatePinnedToCore(frame_task, "frame", 6144, NULL, configMAX_PRIORITIES - 2, &s_frame_task, 1);
    xTaskCreatePinnedToCore(race_task, "strip0", 6144, (void *)0, configMAX_PRIORITIES - 1, &s_task[0], 0);
    xTaskCreatePinnedToCore(race_task, "strip1", 6144, (void *)1, configMAX_PRIORITIES - 1, &s_task[1], 1);

    gptimer_config_t tc = { .clk_src = GPTIMER_CLK_SRC_DEFAULT, .direction = GPTIMER_COUNT_UP, .resolution_hz = 1000000 };
    gptimer_new_timer(&tc, &s_timer);
    gptimer_event_callbacks_t cbs = { .on_alarm = on_tick };
    gptimer_register_event_callbacks(s_timer, &cbs, NULL);
    gptimer_alarm_config_t ac = {};
    ac.alarm_count = 120;
    ac.flags.auto_reload_on_alarm = true;
    gptimer_set_alarm_action(s_timer, &ac);
    gptimer_enable(s_timer);
    gptimer_start(s_timer);
    return true;
}

int32_t beam_race_frame_done() { return s_frame_done; }

void beam_race_stats(BeamRaceStats *o) {
    static uint32_t last_ms, last_frames;
    uint32_t now = millis(), frames = beam_frame_count();
    o->fps = last_ms ? (frames - last_frames) * 1000.0f / (now - last_ms) : 0;
    last_ms = now; last_frames = frames;

    uint32_t n = 0, mx = 0;
    uint64_t sum = 0;
    for (int c = 0; c < 2; c++) {
        n += s_cs[c].n; sum += s_cs[c].cyc_sum;
        if (s_cs[c].cyc_max > mx) mx = s_cs[c].cyc_max;
        s_cs[c].n = s_cs[c].cyc_sum = s_cs[c].cyc_max = 0;
    }
    float mhz = getCpuFrequencyMhz();
    o->strip_avg_us = n ? (float)sum / mhz / n : 0;
    o->strip_max_us = mx / mhz;
    o->budget_us = 2 * FRAME_DT / BEAM_STRIPS * 1e6f;
    o->late = s_late;
    o->skipped = s_skipped;
    o->naps = s_naps;
    o->underruns = beam_underrun_count();
    o->frame_max_us = s_frame_cyc_max / mhz;
    s_frame_cyc_max = 0;
}
