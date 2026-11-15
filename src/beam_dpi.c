// beam_dpi: see beam_dpi.h. Panel bring-up follows Mike's chipguy_10inchP4v2
// (same timing, same JD9365 vendor init table, which we link from his library);
// the DMA side is ours.

#include <string.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_mipi_dsi.h"
#include "esp_ldo_regulator.h"
#include "esp_intr_alloc.h"
#include "esp_clk_tree.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_attr.h"
#include "esp_timer.h"
#include <stdio.h>
#include "esp_private/dw_gdma.h"
#include "esp_private/periph_ctrl.h"
#include "esp_private/esp_clk_tree_common.h"
#include "hal/color_hal.h"
#include "hal/mipi_dsi_hal.h"
#include "hal/mipi_dsi_ll.h"
#include "soc/mipi_dsi_periph.h"
#include "soc/dw_gdma_struct.h"

#include "esp_lcd_jd9365.h"   // Mike's library: bus/IO config macros + vendor init table type
#include "beam_dpi.h"

// Private layout of esp_lcd_dsi_bus_t (same copy Mike's driver uses)
typedef struct esp_lcd_dsi_bus_t {
    int bus_id;
    mipi_dsi_hal_context_t hal;
    void *pm_lock;
} esp_lcd_dsi_bus_t;

static const char *TAG = "beam";

#define PIN_LCD_RST   27
#define PIN_BACKLIGHT 23

// Panel timing (Mike's): 60 MHz pixel clock, 880 x 1312 total -> 51.97 Hz
#define DPI_CLOCK_MHZ 60
#define HSYNC_PW 20
#define HSYNC_BP 20
#define HSYNC_FP 40
#define VSYNC_PW 4
#define VSYNC_BP 8
#define VSYNC_FP 20

static esp_lcd_dsi_bus_handle_t s_bus;
static esp_lcd_panel_io_handle_t s_io;
static dw_gdma_channel_handle_t s_chan;
static dw_gdma_link_list_handle_t s_list;
static intr_handle_t s_brg_intr;
static int s_ch_id;

static int s_nblocks;
static beam_block_t s_blocks[BEAM_MAX_BLOCKS];
static uint32_t s_lli_addr[BEAM_MAX_BLOCKS];

// The driver hands out list items through one alias of internal RAM (0x8FF...)
// while the DMA's LLP register reports the same memory through another (0x4FF...).
// Compare with the alias nibble and the low status bits dropped.
#define LLI_KEY(a) ((uint32_t)(a) & 0x0FFFFFC0u)

static volatile uint32_t s_frame;       // frame the DMA is currently reading
static volatile uint32_t s_seq;         // odd while the frame-done ISR is mid-restart
static volatile uint32_t s_underruns;

// Whole frame read: count it and restart the chain for the next frame. The DMA
// marks items consumed as it goes, so re-arm every item's markers first (Mike's
// driver and Espressif's both do this; without it the second frame stalls).
IRAM_ATTR static bool dma_frame_done(dw_gdma_channel_handle_t chan, const dw_gdma_trans_done_event_data_t *e, void *ctx)
{
    // Readers (beam_cur_strip) retry while s_seq is odd, so nobody sees the new
    // frame number paired with the old frame's DMA registers.
    s_seq++;
    for (int j = 0; j < s_nblocks; j++) {
        dw_gdma_lli_set_block_markers(dw_gdma_link_list_get_item(s_list, j),
                                      (dw_gdma_block_markers_t){ .is_valid = true, .is_last = (j == s_nblocks - 1) });
    }
    volatile dmac_channel_reg_t *c = &DW_GDMA.ch[s_ch_id];
    dw_gdma_channel_use_link_list(chan, s_list);
    dw_gdma_channel_enable_ctrl(chan, true);
    s_frame++;
    // LLP holds item 0's own address until the DMA fetches it, then item 1's.
    // Wait (bounded) for the fetch so LLP/SAR describe the new frame's first block.
    for (int i = 0; i < 400 && LLI_KEY(c->llp0.val) != s_lli_addr[1 % s_nblocks]; i++) {}
    s_seq++;
    return false;
}

IRAM_ATTR static void bridge_isr(void *arg)
{
    mipi_dsi_hal_context_t *hal = &s_bus->hal;
    uint32_t st = mipi_dsi_brg_ll_get_interrupt_status(hal->bridge);
    mipi_dsi_brg_ll_clear_interrupt_status(hal->bridge, st);
    if (st & MIPI_DSI_BRG_LL_EVENT_UNDERRUN) s_underruns++;
}

static esp_err_t build_chain(void)
{
    for (int j = 0; j < s_nblocks; j++) {
        dw_gdma_lli_handle_t lli = dw_gdma_link_list_get_item(s_list, j);
        s_lli_addr[j] = LLI_KEY((uint32_t)lli);
        size_t bytes = (size_t)s_blocks[j].num_strips * BEAM_STRIP_BYTES;
        dw_gdma_block_transfer_config_t cfg = {
            .src = { .addr = (uint32_t)s_blocks[j].src,
                     .burst_mode = DW_GDMA_BURST_MODE_INCREMENT,
                     .burst_items = DW_GDMA_BURST_ITEMS_512, .burst_len = 16,
                     .width = DW_GDMA_TRANS_WIDTH_64 },
            .dst = { .addr = MIPI_DSI_BRG_MEM_BASE,
                     .burst_mode = DW_GDMA_BURST_MODE_FIXED,
                     .burst_items = DW_GDMA_BURST_ITEMS_256, .burst_len = 16,
                     .width = DW_GDMA_TRANS_WIDTH_64 },
            .size = bytes * 8 / 64,
        };
        ESP_RETURN_ON_ERROR(dw_gdma_lli_config_transfer(lli, &cfg), TAG, "lli %d config", j);
        if (j < s_nblocks - 1) dw_gdma_lli_set_next(lli, dw_gdma_link_list_get_item(s_list, j + 1));
        dw_gdma_lli_set_block_markers(lli, (dw_gdma_block_markers_t){ .is_valid = true, .is_last = (j == s_nblocks - 1) });
    }
    return ESP_OK;
}

esp_err_t beam_dpi_init(const beam_block_t *blocks, int num_blocks)
{
    ESP_RETURN_ON_FALSE(num_blocks > 0 && num_blocks <= BEAM_MAX_BLOCKS, ESP_ERR_INVALID_ARG, TAG, "1..15 blocks");
    int next = 0;
    for (int j = 0; j < num_blocks; j++) {
        ESP_RETURN_ON_FALSE(blocks[j].first_strip == next && blocks[j].num_strips > 0 &&
                            ((uint32_t)blocks[j].src & 15) == 0, ESP_ERR_INVALID_ARG, TAG, "bad block %d", j);
        // The bridge splits the frame into num_blocks equal parts on its own; a block of
        // a different size stalls the DMA at that block (seen on hardware 2026-11-15).
        ESP_RETURN_ON_FALSE(blocks[j].num_strips == blocks[0].num_strips, ESP_ERR_INVALID_ARG, TAG,
                            "block %d is %d strips, block 0 is %d: blocks must be equal", j,
                            blocks[j].num_strips, blocks[0].num_strips);
        next += blocks[j].num_strips;
        s_blocks[j] = blocks[j];
    }
    ESP_RETURN_ON_FALSE(next == BEAM_STRIPS, ESP_ERR_INVALID_ARG, TAG, "blocks cover %d strips, need 80", next);
    s_nblocks = num_blocks;

    // DSI PHY power + backlight off
    esp_ldo_channel_handle_t ldo = NULL;
    esp_ldo_channel_config_t ldo_cfg = { .chan_id = 3, .voltage_mv = 2500 };
    ESP_RETURN_ON_ERROR(esp_ldo_acquire_channel(&ldo_cfg, &ldo), TAG, "ldo");
    gpio_config_t bl = { .pin_bit_mask = 1ULL << PIN_BACKLIGHT, .mode = GPIO_MODE_OUTPUT };
    gpio_config(&bl);
    gpio_set_level(PIN_BACKLIGHT, 0);

    esp_lcd_dsi_bus_config_t bus_cfg = JD9365_PANEL_BUS_DSI_2CH_CONFIG();
    ESP_RETURN_ON_ERROR(esp_lcd_new_dsi_bus(&bus_cfg, &s_bus), TAG, "dsi bus");
    esp_lcd_dbi_io_config_t dbi_cfg = JD9365_PANEL_IO_DBI_CONFIG();
    ESP_RETURN_ON_ERROR(esp_lcd_new_panel_io_dbi(s_bus, &dbi_cfg, &s_io), TAG, "dbi io");

    // DMA channel: memory -> DSI bridge, linked list of our blocks
    dw_gdma_channel_alloc_config_t ch_cfg = {
        .src = { .block_transfer_type = DW_GDMA_BLOCK_TRANSFER_LIST, .role = DW_GDMA_ROLE_MEM,
                 .handshake_type = DW_GDMA_HANDSHAKE_HW, .num_outstanding_requests = 5 },
        .dst = { .block_transfer_type = DW_GDMA_BLOCK_TRANSFER_LIST, .role = DW_GDMA_ROLE_PERIPH_DSI,
                 .handshake_type = DW_GDMA_HANDSHAKE_HW, .num_outstanding_requests = 2 },
        .flow_controller = DW_GDMA_FLOW_CTRL_SELF,
        .chan_priority = 1,
    };
    ESP_RETURN_ON_ERROR(dw_gdma_new_channel(&ch_cfg, &s_chan), TAG, "dma channel");
    dw_gdma_channel_get_id(s_chan, &s_ch_id);
    dw_gdma_link_list_config_t ll_cfg = { .num_items = s_nblocks, .link_type = DW_GDMA_LINKED_LIST_TYPE_SINGLY };
    ESP_RETURN_ON_ERROR(dw_gdma_new_link_list(&ll_cfg, &s_list), TAG, "link list");
    dw_gdma_event_callbacks_t cbs = { .on_full_trans_done = dma_frame_done };
    ESP_RETURN_ON_ERROR(dw_gdma_channel_register_event_callbacks(s_chan, &cbs, NULL), TAG, "dma cbs");
    ESP_RETURN_ON_ERROR(build_chain(), TAG, "chain");

    // DPI clock
    int bus_id = s_bus->bus_id;
    mipi_dsi_hal_context_t *hal = &s_bus->hal;
    mipi_dsi_dpi_clock_source_t clk_src = MIPI_DSI_DPI_CLK_SRC_DEFAULT;
    uint32_t src_hz = 0;
    ESP_RETURN_ON_ERROR(esp_clk_tree_src_get_freq_hz(clk_src, ESP_CLK_TREE_SRC_FREQ_PRECISION_CACHED, &src_hz), TAG, "clk");
    uint32_t div = mipi_dsi_hal_host_dpi_calculate_divider(hal, src_hz / 1000 / 1000, DPI_CLOCK_MHZ);
    ESP_RETURN_ON_ERROR(esp_clk_tree_enable_src((soc_module_clk_t)clk_src, true), TAG, "clk en");
    PERIPH_RCC_ATOMIC() {
        mipi_dsi_ll_set_dpi_clock_source(bus_id, clk_src);
        mipi_dsi_ll_set_dpi_clock_div(bus_id, div);
        mipi_dsi_ll_enable_dpi_clock(bus_id, true);
    }
    ESP_RETURN_ON_ERROR(esp_intr_alloc(soc_mipi_dsi_signals[bus_id].brg_irq_id, ESP_INTR_FLAG_LOWMED, bridge_isr, NULL, &s_brg_intr),
                        TAG, "bridge irq");

    // DSI host + bridge, RGB565 in and out
    lcd_color_format_t fmt = COLOR_TYPE_ID(COLOR_SPACE_RGB, LCD_COLOR_PIXEL_FORMAT_RGB565);
    mipi_dsi_host_ll_dpi_set_vcid(hal->host, 0);
    mipi_dsi_host_ll_dpi_set_color_coding(hal->host, fmt, 0);
    mipi_dsi_host_ll_dpi_set_timing_polarity(hal->host, false, false, false, false, false);
    mipi_dsi_host_ll_dpi_enable_lp_horizontal_timing(hal->host, true, true);
    mipi_dsi_host_ll_dpi_enable_lp_vertical_timing(hal->host, true, true, true, true);
    mipi_dsi_host_ll_dpi_enable_lp_command(hal->host, true);
    mipi_dsi_host_ll_dpi_enable_frame_ack(hal->host, true);
    mipi_dsi_host_ll_dpi_set_video_burst_type(hal->host, MIPI_DSI_LL_VIDEO_BURST_WITH_SYNC_PULSES);
    mipi_dsi_host_ll_dpi_set_video_packet_pixel_num(hal->host, BEAM_H_RES);
    mipi_dsi_host_ll_dpi_set_trunks_num(hal->host, 0);
    mipi_dsi_host_ll_dpi_set_null_packet_size(hal->host, 0);
    mipi_dsi_hal_host_dpi_set_horizontal_timing(hal, HSYNC_PW, HSYNC_BP, BEAM_H_RES, HSYNC_FP);
    mipi_dsi_hal_host_dpi_set_vertical_timing(hal, VSYNC_PW, VSYNC_BP, BEAM_V_RES, VSYNC_FP);

    mipi_dsi_brg_ll_set_num_pixel_bits(hal->bridge, BEAM_H_RES * BEAM_V_RES * 16);
    mipi_dsi_brg_ll_set_underrun_discard_count(hal->bridge, BEAM_H_RES);
    mipi_dsi_brg_ll_set_input_color_format(hal->bridge, fmt);
    mipi_dsi_brg_ll_set_output_color_format(hal->bridge, fmt, 0);
    mipi_dsi_brg_ll_set_flow_controller(hal->bridge, MIPI_DSI_LL_FLOW_CONTROLLER_DMA);
    mipi_dsi_brg_ll_set_multi_block_number(hal->bridge, s_nblocks);
    mipi_dsi_brg_ll_set_burst_len(hal->bridge, 256);
    mipi_dsi_brg_ll_set_empty_threshold(hal->bridge, 1024 - 256);
    mipi_dsi_brg_ll_enable(hal->bridge, true);
    mipi_dsi_brg_ll_update_dpi_config(hal->bridge);

    // Panel reset + vendor init
    gpio_config_t rst = { .pin_bit_mask = 1ULL << PIN_LCD_RST, .mode = GPIO_MODE_OUTPUT };
    gpio_config(&rst);
    gpio_set_level(PIN_LCD_RST, 1); vTaskDelay(pdMS_TO_TICKS(5));
    gpio_set_level(PIN_LCD_RST, 0); vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(PIN_LCD_RST, 1); vTaskDelay(pdMS_TO_TICKS(120));

    esp_lcd_panel_io_tx_param(s_io, 0xE0, (uint8_t[]){0x00}, 1);
    esp_lcd_panel_io_tx_param(s_io, 0x36, (uint8_t[]){0xC0}, 1);   // MY|MX, as Mike runs it
    esp_lcd_panel_io_tx_param(s_io, 0x3A, (uint8_t[]){0x55}, 1);
    esp_lcd_panel_io_tx_param(s_io, 0x80, (uint8_t[]){0x01}, 1);
    extern const jd9365_lcd_init_cmd_t vendor_specific_init_default[];
    extern const size_t vendor_specific_init_default_size;
    for (size_t i = 0; i < vendor_specific_init_default_size; i++) {
        const jd9365_lcd_init_cmd_t *c = &vendor_specific_init_default[i];
        esp_lcd_panel_io_tx_param(s_io, c->cmd, c->data_bytes ? c->data : NULL, c->data_bytes);
        if (c->delay_ms) vTaskDelay(pdMS_TO_TICKS(c->delay_ms));
    }

    ESP_LOGI(TAG, "init: %d blocks, dma ch %d, lli0 @ 0x%08lx", s_nblocks, s_ch_id, s_lli_addr[0]);
    return ESP_OK;
}

esp_err_t beam_dpi_start(void)
{
    mipi_dsi_hal_context_t *hal = &s_bus->hal;
    s_frame = 0;
    dw_gdma_channel_use_link_list(s_chan, s_list);
    dw_gdma_channel_enable_ctrl(s_chan, true);

    mipi_dsi_host_ll_enable_video_mode(hal->host, true);
    mipi_dsi_host_ll_set_clock_lane_state(hal->host, MIPI_DSI_LL_CLOCK_LANE_STATE_AUTO);
    mipi_dsi_brg_ll_enable_dpi_output(hal->bridge, true);
    mipi_dsi_brg_ll_update_dpi_config(hal->bridge);
    mipi_dsi_brg_ll_enable_interrupt(hal->bridge, MIPI_DSI_BRG_LL_EVENT_UNDERRUN, true);

    vTaskDelay(pdMS_TO_TICKS(20));
    esp_lcd_panel_io_tx_param(s_io, 0x11, NULL, 0);   // sleep out
    vTaskDelay(pdMS_TO_TICKS(120));
    esp_lcd_panel_io_tx_param(s_io, 0x29, NULL, 0);   // display on
    vTaskDelay(pdMS_TO_TICKS(20));
    gpio_set_level(PIN_BACKLIGHT, 1);
    return ESP_OK;
}

IRAM_ATTR int64_t beam_cur_strip(void)
{
    volatile dmac_channel_reg_t *c = &DW_GDMA.ch[s_ch_id];
    for (int tries = 0; tries < 64; tries++) {
        uint32_t q1 = s_seq;
        if (q1 & 1) continue;                                     // ISR restarting the frame
        uint32_t f1 = s_frame;
        uint32_t llp1 = c->llp0.val;
        uint32_t sar1 = c->sar0.val;
        uint32_t llp2 = c->llp0.val;
        uint32_t sar2 = c->sar0.val;
        if (s_seq != q1 || llp1 != llp2 || sar2 < sar1) continue; // caught a block or frame switch

        // LLP points at the next item (item 0's own address just after a restart,
        // before the fetch); no match means the last block is running.
        uint32_t next = LLI_KEY(llp2);
        int j = s_nblocks - 1;
        if (next == s_lli_addr[0]) j = 0;
        else for (int i = 1; i < s_nblocks; i++) {
            if (s_lli_addr[i] == next) { j = i - 1; break; }
        }
        const beam_block_t *b = &s_blocks[j];
        uint32_t off = sar2 - (uint32_t)b->src;
        uint32_t s = off / BEAM_STRIP_BYTES;
        if (s > b->num_strips) s = b->num_strips;
        return (int64_t)f1 * BEAM_STRIPS + b->first_strip + s;
    }
    return -1;
}

uint32_t beam_frame_count(void) { return s_frame; }
uint32_t beam_underrun_count(void) { return s_underruns; }

// Debug: sample the DMA channel registers for ~25 ms and print every change.
void beam_debug_trace(void)
{
    volatile dmac_channel_reg_t *c = &DW_GDMA.ch[s_ch_id];
    static uint32_t t[3000], llp[3000], sar[3000], fr[3000];
    int64_t t0 = esp_timer_get_time();
    for (int i = 0; i < 3000; i++) {
        t[i] = (uint32_t)(esp_timer_get_time() - t0);
        llp[i] = c->llp0.val; sar[i] = c->sar0.val; fr[i] = s_frame;
        int64_t until = t0 + (int64_t)(i + 1) * 8;
        while (esp_timer_get_time() < until) {}
    }
    printf("[trace] lli:");
    for (int j = 0; j < s_nblocks; j++) printf(" %d=%08lx", j, s_lli_addr[j]);
    printf("\n");
    uint32_t lastllp = 1, lastf = ~0u, lastblk = ~0u;
    for (int i = 0; i < 3000; i++) {
        uint32_t off = sar[i] - (uint32_t)s_blocks[0].src;
        uint32_t blkpos = off / BEAM_STRIP_BYTES;
        if (llp[i] != lastllp || fr[i] != lastf || (blkpos == 0 && lastblk != 0)) {
            printf("[trace] t=%5lu f=%lu llp=%08lx sar_off=%6lu (strip-in-ring %lu)\n", t[i], fr[i], llp[i], off, blkpos);
        }
        lastllp = llp[i]; lastf = fr[i]; lastblk = blkpos;
    }
}
