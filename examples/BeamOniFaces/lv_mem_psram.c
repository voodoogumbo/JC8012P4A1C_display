#include "lv_mem_psram.h"
#include <esp_heap_caps.h>
#include <stdio.h>
#include <esp_log.h>

void* lv_psram_pool_alloc(size_t size) {
    void* p = heap_caps_malloc(size, MALLOC_CAP_SPIRAM);
    printf("[LVGL Pool] Requested %u bytes from PSRAM: %s (%p)\n",
        (unsigned)size, p ? "OK" : "FAILED", p);
    return p;
}
