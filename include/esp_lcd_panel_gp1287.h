/*
 * gp1287_commands.h
 *
 *  Created on: 2024-12-30
 *      Author: koshm
 *
 * SPDX-License-Identifier: GPL-3.0-only
 */
#pragma once

#include <stdbool.h>
#include "esp_err.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_lcd_panel_dev.h"
#include "soc/gpio_num.h"

#ifdef __cplusplus
extern "C" {
#endif

#define GP1287_DISPLAY_WIDTH        (256)
#define GP1287_DISPLAY_HEIGHT       (128)
#define GP1287_DISPLAY_H_RES        (256)
#define GP1287_DISPLAY_V_RES        (50)

typedef struct 
{
    spi_host_device_t host_id;
    
    gpio_num_t filament_en_io_num;     /* filament enable pin */
    gpio_num_t sclk_io_num;            /* serial clock pin */
    gpio_num_t mosi_io_num;            /* data pin */
    gpio_num_t cs_io_num;              /* chip select */
    gpio_num_t rst_io_num;             /* reset pin (reset - LOW, active - HIGH) */
    gpio_num_t int_io_num;             /* reserved (T1 int pin) */
    uint32_t pclk_hz;
    
} gp1287_dev_config_t;

/**
 * @brief Create LCD panel for model GP1287
 *
 * @param[in] io LCD panel IO handle
 * @param[in] panel_dev_config general panel device configuration
 * @param[out] ret_panel Returned LCD panel handle
 * @return
 *          - ESP_ERR_INVALID_ARG   if parameter is invalid
 *          - ESP_ERR_NO_MEM        if out of memory
 *          - ESP_OK                on success
 *
 * esp_lcd_panel_handle_t panel_handle = NULL;
 * esp_lcd_new_panel_gp1287(io_handle, &panel_config, &panel_handle);
 * @endcode
 */
esp_err_t esp_lcd_new_panel_gp1287(const esp_lcd_panel_io_handle_t io,
    const esp_lcd_panel_dev_config_t *panel_dev_config,
    esp_lcd_panel_handle_t *ret_panel);

esp_err_t esp_lcd_panel_gp1287_set_offset(const esp_lcd_panel_handle_t panel, int offset_x, int offset_y);

#ifdef __cplusplus
}
#endif
