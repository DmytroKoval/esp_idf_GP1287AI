/*
 * SPDX-FileCopyrightText: 2021-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "esp_err.h"
#include "esp_heap_caps.h"
#include "hal/gpio_types.h"
#include "soc/gpio_num.h"
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <sys/cdefs.h>
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
#include "esp_lcd_panel_gp1287.h"
#include "esp_lcd_panel_ops.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_check.h"
#include "sdkconfig.h"
#include "esp_lcd_panel_gp1287.h"
#include "gp1287_commands.h"

static const char *TAG = "lcd_panel.gp1287";

#define LCD_CHECK(a, str, ret)  if(!(a)) {                           \
        ESP_LOGE(TAG,"%s:%d (%s):%s", __FILE__, __LINE__, __FUNCTION__, str);   \
        return (ret);                                                           \
    }


#define GP1287_CMD_RESET            (0xAA)
#define GP1287_CMD_SET_BRIGHTNESS   (0xA0)
#define GP1287_CMD_SET_OSC          (0x78)
#define GP1287_CMD_SET_VFDMODE      (0xCC)
#define GP1287_CMD_SET_DISPAREA     (0xE0)
#define GP1287_CMD_WRITE_GRAM       (0xF0)

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
    // uint16_t brightness;
} gp1287_panel_t;

esp_err_t configure_panel_gpio(gpio_num_t reset_pin_num, gpio_num_t filament_enable_pin_num)
{
    uint64_t pin_bitmask = 0ULL;

    if (reset_pin_num != GPIO_NUM_NC) {
        pin_bitmask |= (1 << reset_pin_num);
    }

    if (filament_enable_pin_num > GPIO_NUM_NC) {
        pin_bitmask |= (1 << filament_enable_pin_num);
    }

    if (pin_bitmask == 0)
    {
        return ESP_OK; 
    }
    
    gpio_config_t gpio_conf = {
        .mode = GPIO_MODE_OUTPUT,  
        .intr_type = GPIO_INTR_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pin_bit_mask = pin_bitmask
    };
    //gpio_conf
    //;
    
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
    
    gp1287 = calloc(1, sizeof(gp1287_panel_t)); 
    ESP_GOTO_ON_FALSE(gp1287, ESP_ERR_NO_MEM, err, TAG, "no mem for gp1287 panel");

    
    configure_panel_gpio(panel_dev_config->reset_gpio_num, 
        gp1287_config->filament_en_io_num);
    
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
    ESP_LOGD(TAG, "New GP1287 panel @%p", gp1287);

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

esp_err_t esp_lcd_panel_set_brightness(const esp_lcd_panel_handle_t panel, uint16_t brightness)
{
    gp1287_panel_t *gp1287 = __containerof(panel, gp1287_panel_t, base);
     esp_lcd_panel_io_handle_t io = gp1287->io;
    //gp1287->brightness = brightness & 0x3FF;
    esp_lcd_panel_io_tx_param(io, 0xA0, (uint8_t []) { (uint8_t)((brightness & 0x0300) >> 8), (uint8_t)(brightness & 0xFF) }, 2);
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
    ESP_LOGD(TAG, "Delete GP1287 panel @%p", gp1287);
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
    if (gp1287->filament_en_gpio_num != GPIO_NUM_NC)
    {
        gpio_set_level(gp1287->filament_en_gpio_num, 1);
        // filament warmup time
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    
    /* Software reset */
    esp_lcd_panel_io_tx_param(io, GP1287_CMD_RESET, NULL, 0);
    vTaskDelay(pdMS_TO_TICKS(5));

    /* Oscillation Setting */
    // esp_lcd_panel_io_tx_param(io, 0x78, (uint8_t []) { 0x08 }, 1);
    esp_lcd_panel_io_tx_color(io, -1,
        (uint8_t []) {GP1287_CMD_SET_OSC, 0x08 }, 2);

    /* VFD Mode Setting */
    // esp_lcd_panel_io_tx_param(io, GP1287_CMD_SET_VFDMODE, (uint8_t []) { 0x02, 0x00 }, 2);
    esp_lcd_panel_io_tx_color(io, -1,
        (uint8_t []) {GP1287_CMD_SET_VFDMODE, 0x02, 0x00 }, 3);
    
    /* Display Area Setting */
    // esp_lcd_panel_io_tx_param(io, 0xE0, (uint8_t []) { 0xFF, 0x31, 0x00, 0x20, 0x00, 0x00, 0x00 }, 7);
    esp_lcd_panel_io_tx_color(io, -1, (uint8_t []) { GP1287_CMD_SET_DISPAREA,
        0xFF, 0x31, 0x00, 0x20, 0x00, 0x00, 0x80 }, 8);
        
    /* Internal Speed Setting */
    // esp_lcd_panel_io_tx_param(io, 0xB1,
    //     (uint8_t []) {0x20, 0x3F, 0x00, 0x01 }, 4);
    esp_lcd_panel_io_tx_color(io, -1,
        (uint8_t []) { 0xB1, 0x20, 0x3F, 0x00, 0x01 }, 5);
    
    /* Dimming level Setting (1024 level, 0x3FF max) */
    esp_lcd_panel_io_tx_param(io, GP1287_CMD_SET_BRIGHTNESS, (uint8_t []) { 0x00, 0x10 }, 2);
    // esp_lcd_panel_io_tx_color(io, -1, (uint8_t []) { 0xA0, 0x00, 0x60 }, 3);
    
    /* Memory Map Clear */
    esp_lcd_panel_io_tx_param(io, 0x55, NULL, 0);
    vTaskDelay(pdMS_TO_TICKS(20));

    /* DW1 position setting (Set Display Area offset) */
    esp_lcd_panel_io_tx_param(io, 0xC0, (uint8_t []) { 0x00, 0x00 }, 2);
    // esp_lcd_panel_io_tx_color(io, -1, (uint8_t []) { 0xC0, 0x00, 0x04 }, 3);
    
    /* DW2 position setting */
    // esp_lcd_panel_io_tx_param(io, 0xD0, (uint8_t []) { 0x00, 0x3C }, 2);
    // esp_lcd_panel_io_tx_color(io, -1, (uint8_t []) { 0xD0, 0x00, 0x3C }, 3);
    
    /* Internal Command */
    // esp_lcd_panel_io_tx_param(io, 0x90, (uint8_t []) { 0x00 }, 1);
    // esp_lcd_panel_io_tx_color(io, -1, (uint8_t []) { 0x90, 0x00 }, 2);
    
    /* T1 INT Setting */
    esp_lcd_panel_io_tx_param(io, 0x08, (uint8_t []) { 0x00 }, 1);
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
    esp_lcd_panel_io_tx_color(io, -1, (uint8_t []) { 0x80, 0b00000000 }, 2);
    
    /* Exit standby Mode */
    esp_lcd_panel_io_tx_param(io, 0x6D, NULL, 0);
    
    gbuf = (uint8_t *)calloc(3 + 256 * 16, 1);
    
    return ESP_OK;
}

static esp_err_t panel_gp1287_draw_bitmap(esp_lcd_panel_t *panel, int x_start, int y_start, int x_end, int y_end, const void *color_data)
{
    gp1287_panel_t *gp1287 = __containerof(panel, gp1287_panel_t, base);
    esp_lcd_panel_io_handle_t io = gp1287->io;

    // ESP_RETURN_ON_FALSE(y_start < y_end && x_start < x_end, ESP_ERR_INVALID_ARG, TAG, "draw bitmap: x0(%d) > x1(%d) or y0(%d) > y1(%d)", x_start, x_end, y_start, y_end);

    esp_err_t ret = ESP_OK;

    uint16_t height = y_end - y_start;
    uint16_t width = x_end - x_start;
    uint16_t return_length = (height & 0xF8) - 1;

    size_t data_len = width * (height >> 3);
    
    gbuf[0] = x_start & 0xFF;
    gbuf[1] = y_start & 0x7F;
    gbuf[2] = return_length;

    // memcpy(gbuf + 3, color_data, data_len);

    // lvgl bitmap transform
    for (int x = x_start; x < x_end; x++)
    {
        for (int y = (y_start >> 3); y < (y_end >> 3); y++)
        {
            int src_idx = y * 256 + x;
            int dest_idx = x * 16 + y;
            gbuf[3 + dest_idx] = ((uint8_t *)color_data)[src_idx];
        }
    }
    ESP_GOTO_ON_ERROR(esp_lcd_panel_io_tx_color(io, GP1287_CMD_WRITE_GRAM, gbuf, data_len + 3),
        cleanup, TAG, "io tx param GP1287_CMD_WRITE_GRAM");

cleanup:
    // heap_caps_free(gbuf);
    return ret;
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
    /*
    gp1287_panel_t *gp1287 = __containerof(panel, gp1287_panel_t, base);
    esp_lcd_panel_io_handle_t io = gp1287->io;

    int command = 0;
    if (mirror_x) {
        command = GP1287_CMD_set_m;
    } else {
        command = GP1287_CMD_MIRROR_X_OFF;
    }
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, command, NULL, 0), TAG,
                        "io tx param GP1287_CMD_MIRROR_X_ON/OFF failed");
    if (mirror_y) {
        command = GP1287_CMD_MIRROR_Y_ON;
    } else {
        command = GP1287_CMD_MIRROR_Y_OFF;
    }
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, command, NULL, 0), TAG,
                        "io tx param GP1287_CMD_MIRROR_Y_ON/OFF failed");
    */
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
        command = 0b00000001;
    }
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_color(io, -1, (uint8_t []) { 0x80, command }, 2), TAG,
                        "io tx param GP1287_CMD_DISP_ON_OFF failed");
    return ESP_OK;
}

static esp_err_t panel_gp1287_disp_sleep(esp_lcd_panel_t *panel, bool sleep)
{
    gp1287_panel_t *gp1287 = __containerof(panel, gp1287_panel_t, base);
    esp_lcd_panel_io_handle_t io = gp1287->io;
    if (sleep) {
        ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, 0x61, NULL, 0), TAG, "Display sleep failed");
        // filament_level = 0;
    } else {
        ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, 0x6D, NULL, 0), TAG, "Display wake-up failed");
        ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, 0x80, (uint8_t []) { 0x00 }, 1), TAG, "Display set mode failed");
        // filament_level = 0;
    }
        // 
    
    return ESP_OK;
}