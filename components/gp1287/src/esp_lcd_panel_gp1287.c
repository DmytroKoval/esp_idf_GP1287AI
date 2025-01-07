/*
 *  Created on: 2024-12-30
 *      Author: koshm
 *
 * SPDX-License-Identifier: GPL-3.0-only
 */

#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <sys/cdefs.h>

#include "esp_err.h"
#include "esp_heap_caps.h"
#include "freertos/projdefs.h"
#include "hal/gpio_types.h"
#include "soc/gpio_num.h"
#if CONFIG_LCD_ENABLE_DEBUG_LOG
// The local log level must be defined before including esp_log.h
// Set the maximum log level for this source file
#define LOG_LOCAL_LEVEL ESP_LOG_DEBUG
#endif

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_lcd_panel_interface.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_dev.h"
#include "esp_lcd_panel_ops.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_check.h"
#include "sdkconfig.h"

#include "esp_lcd_panel_gp1287.h"
#include "gp1287_commands.h"

static const char *TAG = "LCD PANEL GP1287";

#define LCD_CHECK(a, str, ret)  if(!(a)) {                           \
        ESP_LOGE(TAG,"%s:%d (%s):%s", __FILE__, __LINE__, __FUNCTION__, str);   \
        return (ret);                                                           \
    }

/*

// Default values.
#define GP1287_DEFAULT_COL1        0x00 //
#define GP1287_DEFAULT_COL2        0x77 //
#define GP1287_DEFAULT_ROW1        0x00 //
#define GP1287_DEFAULT_ROW2        0x7f //
#define GP1287_DEFAULT_REMAP1      0x00 //
#define GP1287_DEFAULT_REMAP2      0x01 //
#define GP1287_DEFAULT_START       0x00 //
#define GP1287_DEFAULT_OFFSET      0x00 //
#define GP1287_DEFAULT_VDD         0x01 //
#define GP1287_DEFAULT_PHASE       0x74 //
#define GP1287_DEFAULT_CLOCK       0x50 //
#define GP1287_DEFAULT_ENHANCE_A1  0xa2 //
#define GP1287_DEFAULT_ENHANCE_A2  0xb5 //
#define GP1287_DEFAULT_GPIOS       0x0a //
#define GP1287_DEFAULT_PERIOD      0x08 //
#define GP1287_DEFAULT_PRE_VOLT    0x17 //
#define GP1287_DEFAULT_COM_VOLT    0x04 //
#define GP1287_DEFAULT_CONTRAST    0x7f //
#define GP1287_DEFAULT_BRIGHTNESS  0xff //
#define GP1287_DEFAULT_MUX         0x7f //
#define GP1287_DEFAULT_ENHANCE_B1  0xa2 //
#define GP1287_DEFAULT_ENHANCE_B2  0x20 //

// GPIO states.
#define GP1287_INPUT_COMMAND 0 // Enable command mode for DC# pin.
#define GP1287_INPUT_DATA    1 // Enable data mode for DC# pin.
#define GP1287_RESET_ON      0 // Hardware reset.
#define GP1287_RESET_OFF     1 // Normal operation.

// Ranges.
#define GP1287_COLS_MIN       0x00 // Start column.
#define GP1287_COLS_MAX       0x77 // End column.
#define GP1287_ROWS_MIN       0x00 // Start row.
#define GP1287_ROWS_MAX       0x7f // End row.

// Settings.
#define GP1287_INC_COLS       0x00 // Increment cols.
#define GP1287_INC_ROWS       0x01 // Increment rows.
#define GP1287_SCAN_RIGHT     0x00 // Scan columns left to right.
#define GP1287_SCAN_LEFT      0x02 // Scan columns right to left.
#define GP1287_SCAN_DOWN      0x00 // Scan rows from top to bottom.
#define GP1287_SCAN_UP        0x10 // Scan rows from bottom to top.
#define GP1287_VDD_EXTERNAL   0x00 // Use external VDD regulator.
#define GP1287_VDD_INTERNAL   0x01 // Use internal VDD regulator (reset).

// Enable/disable.
#define GP1287_PARTIAL_ON      0x01 // Partial mode on.
#define GP1287_PARTIAL_OFF     0x00 // Partial mode off.
#define GP1287_SPLIT_DISABLE   0x00 // Disable odd/even split of COMs.
#define GP1287_SPLIT_ENABLE    0x20 // Enable odd/even split of COMS.
#define GP1287_DUAL_DISABLE    0x00 // Disable dual COM line mode.
#define GP1287_DUAL_ENABLE     0x10 // Enable dual COM line mode.
#define GP1287_REMAP_DISABLE   0x00 // Disable nibble re-map.
#define GP1287_REMAP_ENABLE    0x04 // Enable nibble re-map.
#define GP1287_COMMAND_LOCK    0x16 // Command lock.
#define GP1287_COMMAND_UNLOCK  0x12 // Command unlock.

// Resets
#define GP1287_CLOCK_DIV_RESET  0x01
#define GP1287_CLOCK_FREQ_RESET 0xc0
#define GP1287_PERIOD_RESET     0x08

// Column offset
#define GP1287_COL_OFFSET       0x1c // Based on example code.
*/
static esp_err_t panel_gp1287_reset(esp_lcd_panel_t *panel);
static esp_err_t panel_gp1287_init(esp_lcd_panel_t *panel);
static esp_err_t panel_gp1287_del(esp_lcd_panel_t *panel);

static esp_err_t panel_gp1287_draw_bitmap(esp_lcd_panel_t *panel, int x_start, int y_start, int x_end, int y_end, const void *color_data);
static esp_err_t panel_gp1287_invert_color(esp_lcd_panel_t *panel, bool invert_color_data);
static esp_err_t panel_gp1287_mirror(esp_lcd_panel_t *panel, bool mirror_x, bool mirror_y);
static esp_err_t panel_gp1287_swap_xy(esp_lcd_panel_t *panel, bool swap_axes);
static esp_err_t panel_gp1287_set_gap(esp_lcd_panel_t *panel, int x_gap, int y_gap);
static esp_err_t panel_gp1287_disp_on_off(esp_lcd_panel_t *panel, bool off);
static esp_err_t panel_gp1287_disp_sleep(esp_lcd_panel_t *panel, bool sleep);
typedef struct {
    esp_lcd_panel_t base;
    esp_lcd_panel_io_handle_t io;
    int reset_gpio_num;
    int x_gap;
    int y_gap;
    unsigned int bits_per_pixel;
    bool reset_level;
    bool swap_axes;
    bool mirror;
    bool rotate;
    gpio_num_t filament_en_gpio_num;
} gp1287_panel_t;

esp_err_t configure_panel_gpio(gpio_num_t reset_pin_num, gpio_num_t filament_enable_pin_num)
{
    uint64_t pin_bitmask = 0ULL;

    if (reset_pin_num != GPIO_NUM_NC) {
        pin_bitmask |= (1ULL << reset_pin_num);
    }

    if (filament_enable_pin_num > GPIO_NUM_NC) {
        pin_bitmask |= 1ULL << filament_enable_pin_num;
    }

    if (pin_bitmask == 0)
    {
        return ESP_OK; 
    }
    
    gpio_config_t gpio_conf = {
        .pin_bit_mask = pin_bitmask,
        .mode = GPIO_MODE_OUTPUT,  
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    
    return gpio_config(&gpio_conf);
}

esp_err_t esp_lcd_new_panel_gp1287(const esp_lcd_panel_io_handle_t io,
    const esp_lcd_panel_dev_config_t *panel_dev_config,
    esp_lcd_panel_handle_t *ret_panel)
{
#if CONFIG_LCD_ENABLE_DEBUG_LOG
    esp_log_level_set(TAG, ESP_LOG_DEBUG);
#endif
    esp_err_t ret = ESP_OK;

    gp1287_dev_config_t *gp1287_config = (gp1287_dev_config_t *)(panel_dev_config->vendor_config);
    gp1287_panel_t *gp1287 = NULL;

    ESP_GOTO_ON_FALSE(io && panel_dev_config && ret_panel, ESP_ERR_INVALID_ARG, err, TAG, "invalid argument");
    ESP_GOTO_ON_FALSE(panel_dev_config->bits_per_pixel == 1, ESP_ERR_INVALID_ARG, err, TAG, "bpp must be 1");
    
    gp1287 = calloc(sizeof(gp1287_panel_t), 1);
    ESP_GOTO_ON_FALSE(gp1287, ESP_ERR_NO_MEM, err, TAG, "no mem for gp1287 panel");

    
    ESP_GOTO_ON_FALSE(configure_panel_gpio(panel_dev_config->reset_gpio_num, 
        gp1287_config->filament_en_io_num), ESP_ERR_INVALID_STATE, err, TAG, "gpio config failed");
    
    gp1287->io = io;
    gp1287->bits_per_pixel = panel_dev_config->bits_per_pixel;
    gp1287->reset_gpio_num = panel_dev_config->reset_gpio_num;
    gp1287->reset_level = panel_dev_config->flags.reset_active_high;
    gp1287->filament_en_gpio_num = gp1287_config->filament_en_io_num;
    gp1287->base.del = panel_gp1287_del;
    gp1287->base.reset = panel_gp1287_reset;
    gp1287->base.init = panel_gp1287_init;
    gp1287->base.draw_bitmap = panel_gp1287_draw_bitmap;
    gp1287->base.invert_color = panel_gp1287_invert_color;
    gp1287->base.set_gap = panel_gp1287_set_gap;
    gp1287->base.mirror = panel_gp1287_mirror;
    gp1287->base.swap_xy = panel_gp1287_swap_xy;
    gp1287->base.disp_on_off = panel_gp1287_disp_on_off;
    gp1287->base.disp_sleep = panel_gp1287_disp_sleep;
    
    *ret_panel = &(gp1287->base);
    ESP_LOGD(TAG, "new gp1287 panel @%p", gp1287);

    return ESP_OK;

err:
    if (gp1287) {
        if (panel_dev_config->reset_gpio_num >= 0) {
            gpio_reset_pin(panel_dev_config->reset_gpio_num);
        }
        if (gp1287_config->filament_en_io_num >= 0)
        {
            gpio_reset_pin(gp1287_config->filament_en_io_num);
        }
        free(gp1287);
    }
    return ret;
}


esp_err_t esp_lcd_panel_gp1287_set_offset(esp_lcd_panel_handle_t panel, int offset_x, int offset_y)
{
    gp1287_panel_t *gp1287 = __containerof(panel, gp1287_panel_t, base);
    esp_lcd_panel_io_handle_t io = gp1287->io;

    esp_lcd_panel_io_tx_param(io, 0xC0, (uint8_t []) { offset_x & 0xFF, offset_y & 0x7F }, 2);
    return ESP_OK;
}


static esp_err_t panel_gp1287_del(esp_lcd_panel_t *panel)
{
    gp1287_panel_t *gp1287 = __containerof(panel, gp1287_panel_t, base);
    
    if (gp1287->reset_gpio_num != GPIO_NUM_NC) {
        gpio_reset_pin(gp1287->reset_gpio_num);
    }
    if (gp1287->filament_en_gpio_num != GPIO_NUM_NC)
    {
        gpio_reset_pin(gp1287->filament_en_gpio_num);
    }
    ESP_LOGD(TAG, "del gp1287 panel @%p", gp1287);
    free(gp1287);
    return ESP_OK;
}

static esp_err_t panel_gp1287_reset(esp_lcd_panel_t *panel)
{
    gp1287_panel_t *gp1287 = __containerof(panel, gp1287_panel_t, base);

    // perform hardware reset
    if (gp1287->reset_gpio_num >= 0) {
        gpio_set_level(gp1287->reset_gpio_num, gp1287->reset_level);
        vTaskDelay(pdMS_TO_TICKS(5));
        gpio_set_level(gp1287->reset_gpio_num, !gp1287->reset_level);
        vTaskDelay(pdMS_TO_TICKS(5));
    }

    return ESP_OK;
}
static uint8_t *gbuf;

static esp_err_t panel_gp1287_init(esp_lcd_panel_t *panel)
{
    gp1287_panel_t *gp1287 = __containerof(panel, gp1287_panel_t, base);
    esp_lcd_panel_io_handle_t io = gp1287->io;
    
    /* Software reset */
    esp_lcd_panel_io_tx_param(io, GP1287_CMD_RESET, NULL, 0);
    vTaskDelay(pdMS_TO_TICKS(5));

    /* Oscillation Setting */
    esp_lcd_panel_io_tx_param(io, GP1287_CMD_SET_OSC, (uint8_t []) { 0x08 }, 2);

    /* VFD Mode Setting */
    esp_lcd_panel_io_tx_param(io, GP1287_CMD_SET_VFDMODE, (uint8_t []) { 0x02, 0x00 }, 2);
    
    /* Display Area Setting */
    // esp_lcd_panel_io_tx_param(io, 0xE0, (uint8_t []) { 0xFF, 0x31, 0x00, 0x20, 0x00, 0x00, 0x00 }, 7);
    esp_lcd_panel_io_tx_param(io, GP1287_CMD_SET_DISPAREA, (uint8_t []) { 0xFF, 0x31, 0x00, 0x20, 0x00, 0x00, 0x80 }, 7);
        
    /* Internal Speed Setting */
    esp_lcd_panel_io_tx_param(io, GP1287_CMD_SET_SPEED, (uint8_t []) {0x20, 0x3F, 0x00, 0x01 }, 4);
    
    /* Dimming level Setting (1024 level, 0x3FF max) */
    esp_lcd_panel_io_tx_param(io, GP1287_CMD_SET_DIMMING, (uint8_t []) { 0x00, 0x30 }, 2);
    
    /* Memory Map Clear */
    esp_lcd_panel_io_tx_param(io, GP1287_CMD_MEMCLR, NULL, 0);
    vTaskDelay(pdMS_TO_TICKS(20));

    /* DW1 position setting (Set Display Area offset) */
    esp_lcd_panel_io_tx_param(io, 0xC0, (uint8_t []) { 0x00, 0x00 }, 2);
    
    /* DW2 position setting ? */
    // esp_lcd_panel_io_tx_param(io, 0xD0, (uint8_t []) { 0x00, 0x3C }, 2);
    // esp_lcd_panel_io_tx_color(io, -1, (uint8_t []) { 0xD0, 0x00, 0x3C }, 3);
    
    /* Internal Command ? */
    // esp_lcd_panel_io_tx_param(io, 0x90, (uint8_t []) { 0x00 }, 1);
    // esp_lcd_panel_io_tx_color(io, -1, (uint8_t []) { 0x90, 0x00 }, 2);
    
    /* T1 INT Setting */
    esp_lcd_panel_io_tx_param(io, GP1287_CMD_SET_INT, (uint8_t []) { 0x00 }, 1);
    // esp_lcd_panel_io_tx_color(io, -1, (uint8_t []) { 0x08, 0x00 }, 2);
    
    /* Display Mode Setting */
    /** param bits:
     *  0  0  *  SC HS LS *  NP
     * -----------------------------------------------------------
     *  0  0  *  1  *  *  *  * - stop scan
     *  0  0  *  0  1  0  *  * - all pixels on
     *  0  0  *  0  *  1  *  * - all pixels off
     *  0  0  *  0  0  0  *  0 - normal operation, positive scan
     *  0  0  *  0  0  0  *  1 - invert scan
    **/
    // esp_lcd_panel_io_tx_param(io, 0x80, (uint8_t []) { 0b00000000 }, 1);
    esp_lcd_panel_io_tx_param(io, GP1287_CMD_SET_DISPMODE, (uint8_t []) { 0b00000000 }, 1);
    
    /* Exit standby Mode */
    esp_lcd_panel_io_tx_param(io, 0x6D, NULL, 0);

    gbuf = (uint8_t *)heap_caps_calloc(256 * (128 >> 3), 1, MALLOC_CAP_DMA | MALLOC_CAP_8BIT);

    return ESP_OK;
}


static esp_err_t panel_gp1287_draw_bitmap(esp_lcd_panel_t *panel, int x_start, int y_start, int x_end, int y_end, const void *color_data)
{
    gp1287_panel_t *gp1287 = __containerof(panel, gp1287_panel_t, base);
    esp_lcd_panel_io_handle_t io = gp1287->io;
    ESP_RETURN_ON_FALSE(y_start < y_end && x_start < x_end, ESP_ERR_INVALID_ARG, TAG, "draw bitmap: x0(%d) >= x1(%d) or y0(%d) >= y1(%d)", x_start, x_end, y_start, y_end);

    uint16_t height = y_end - y_start;
    
    
    uint16_t width = x_end - x_start;
    uint16_t return_length = (((height) >> 3) << 3) - 1;
    
    gbuf[0] = x_start & 0xFF;
    gbuf[1] = y_start & 0x7F;
    gbuf[2] = return_length;
    // adding extra gap ??
    // x0 += gp1287->x_gap;
    // uint16_t x1 = x0 + (x_end - x_start);
    // y0 += gp1287->y_gap;
    // uint16_t y1 = y0 + (y_end - y_start);
    // y0 &= 0x78; 
    
    uint8_t data_height = height >> 3; // "height" of data buf in BYTES (pixel_height / 8)
    size_t data_len = width * data_height; 

    // convert color_data to GP1287 data format
    for (int x = 0, xh = 0; x < width; x++)
    {
        xh = x * data_height;
        for (int y = 0; y < data_height; y++)
        {
            uint32_t src_idx = y * width + x;
            uint32_t dest_idx = xh + y + 3;
            gbuf[dest_idx] = ((uint8_t *)color_data)[src_idx];
        }
    }

//    ESP_LOGI(TAG, "x0:%d, y0: %d, x1: %d, y1: %d", x_start, y_start, x_end, y_end);
//    ESP_LOGI(TAG, "width:%d, height: %d, data-height: %d\nreturn_length: %d, data_length: %d", width, height, height >> 3, return_length, data_len);

    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_color(io, GP1287_CMD_WRITE_GRAM, gbuf, data_len + 3),
        TAG, "io tx param GP1287_CMD_WRITE_GRAM");

    return ESP_OK;
}
/*
static esp_err_t gp1287_set_display_area(esp_lcd_panel_t *panel)
{
    gp1287_panel_t *gp1287 = __containerof(panel, gp1287_panel_t, base);
    esp_lcd_panel_io_handle_t io = gp1287->io;

    

    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, 0x80, (uint8_t []) { command_param }, 1),
        TAG, "io tx param GP1287_CMD_INVERT_ON/OFF failed");
        
    return ESP_OK;
}
*/

static esp_err_t panel_gp1287_invert_color(esp_lcd_panel_t *panel, bool invert_color_data)
{
    gp1287_panel_t *gp1287 = __containerof(panel, gp1287_panel_t, base);
    esp_lcd_panel_io_handle_t io = gp1287->io;
    int command_param = 0;
    if (invert_color_data) {
        command_param = 0b00000001;
    } else {
        command_param = 0b00000000;
    }
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, 0x80, (uint8_t []) { command_param }, 1),
        TAG, "io tx param GP1287_CMD_INVERT_ON/OFF failed");
    return ESP_OK;
}

static esp_err_t panel_gp1287_mirror(esp_lcd_panel_t *panel, bool mirror_x, bool mirror_y)
{
    return ESP_OK;
}

static esp_err_t panel_gp1287_swap_xy(esp_lcd_panel_t *panel, bool swap_axes)
{
    gp1287_panel_t *gp1287 = __containerof(panel, gp1287_panel_t, base);
    gp1287->swap_axes = swap_axes;

    return ESP_OK;
}

static esp_err_t panel_gp1287_set_gap(esp_lcd_panel_t *panel, int x_gap, int y_gap)
{
    gp1287_panel_t *gp1287 = __containerof(panel, gp1287_panel_t, base);
    gp1287->x_gap = x_gap;
    gp1287->y_gap = y_gap;
    return ESP_OK;
}

static esp_err_t panel_gp1287_disp_on_off(esp_lcd_panel_t *panel, bool on_off)
{
    gp1287_panel_t *gp1287 = __containerof(panel, gp1287_panel_t, base);
    esp_lcd_panel_io_handle_t io = gp1287->io;
    /* Display Mode Setting */
    /** param bits:
     *  0  0  *  SC HS LS *  NP
     *  0  0  *  1  *  *  *  * - stop scan
     *  0  0  *  0  1  0  *  * - all pixels on
     *  0  0  *  0  *  1  *  * - all pixels off
     *  0  0  *  0  0  0  *  0 - normal operation, positive scan
     *  0  0  *  0  0  0  *  1 - invert scan
    **/
    uint8_t command;
    if (on_off) {
        command = 0b00000000;
    } else {
        command = 0b00010000;
    }
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, GP1287_CMD_SET_DISPMODE, (uint8_t []) { command }, 1),
        TAG, "io tx param GP1287_CMD_DISP_ON_OFF failed");
    return ESP_OK;
}

static esp_err_t panel_gp1287_disp_sleep(esp_lcd_panel_t *panel, bool sleep)
{
    gp1287_panel_t *gp1287 = __containerof(panel, gp1287_panel_t, base);
    esp_lcd_panel_io_handle_t io = gp1287->io;
    
    if (sleep) {
        ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, GP1287_CMD_STANDBY, NULL, 0), TAG, "Display sleep failed");
        gpio_set_level(gp1287->filament_en_gpio_num, 0);
    } else {
        ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, GP1287_CMD_WAKEUP, NULL, 0), TAG, "Display wake-up failed");
        ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, GP1287_CMD_SET_DISPMODE, (uint8_t []) { 0x00 }, 1), TAG, "Display set mode failed");
        gpio_set_level(gp1287->filament_en_gpio_num, 1);
        vTaskDelay(pdMS_TO_TICKS(5)); // filament warmup delay
    }
     
    
    return ESP_OK;
}