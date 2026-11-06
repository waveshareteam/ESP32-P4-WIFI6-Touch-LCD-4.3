/*
 * SPDX-FileCopyrightText: 2015-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>
#include "esp_check.h"
#include "esp_log.h"
#include "driver/ledc.h"
#include "esp_lv_adapter.h"
#include "esp_board_manager_includes.h"
#include "board_display.h"

static const char *TAG = "board_display";

static lv_display_t *s_display = NULL;
static lv_indev_t *s_touch_indev = NULL;
static esp_lcd_panel_handle_t s_panel = NULL;
static esp_lcd_touch_handle_t s_touch = NULL;
static periph_ledc_handle_t *s_backlight = NULL;
static void *s_i2c_master = NULL;
static uint32_t s_backlight_duty_max = 0;
static int s_brightness = 0;

static esp_err_t board_backlight_init(void)
{
    ESP_RETURN_ON_ERROR(esp_board_manager_init_device_by_name(ESP_BOARD_DEVICE_NAME_LCD_BRIGHTNESS), TAG,
                        "Initialize lcd_brightness failed");
    ESP_RETURN_ON_ERROR(esp_board_manager_get_device_handle(ESP_BOARD_DEVICE_NAME_LCD_BRIGHTNESS, (void **)&s_backlight),
                        TAG, "Get lcd_brightness handle failed");

    dev_ledc_ctrl_config_t *dev_cfg = NULL;
    ESP_RETURN_ON_ERROR(esp_board_manager_get_device_config(ESP_BOARD_DEVICE_NAME_LCD_BRIGHTNESS, (void **)&dev_cfg),
                        TAG, "Get lcd_brightness config failed");
    periph_ledc_config_t *ledc_cfg = NULL;
    ESP_RETURN_ON_ERROR(esp_board_periph_get_config(dev_cfg->ledc_name, (void **)&ledc_cfg), TAG,
                        "Get LEDC config failed");

    // The LEDC peripheral applies output_invert, so a larger duty is always brighter here.
    s_backlight_duty_max = (1U << (uint32_t)ledc_cfg->duty_resolution) - 1;
    s_brightness = (int)dev_cfg->default_percent;
    return ESP_OK;
}

static lv_display_t *board_display_lvgl_init(void)
{
    dev_display_lcd_handles_t *lcd_handles = NULL;
    dev_display_lcd_config_t *lcd_cfg = NULL;
    ESP_RETURN_ON_FALSE(esp_board_manager_get_device_handle(ESP_BOARD_DEVICE_NAME_DISPLAY_LCD, (void **)&lcd_handles) == ESP_OK
                        && lcd_handles, NULL, TAG, "Get display_lcd handle failed");
    ESP_RETURN_ON_FALSE(esp_board_manager_get_device_config(ESP_BOARD_DEVICE_NAME_DISPLAY_LCD, (void **)&lcd_cfg) == ESP_OK
                        && lcd_cfg, NULL, TAG, "Get display_lcd config failed");
    ESP_RETURN_ON_FALSE(strcmp(lcd_cfg->sub_type, ESP_BOARD_DEVICE_LCD_SUB_TYPE_DSI) == 0, NULL, TAG,
                        "Unsupported LCD sub_type: %s", lcd_cfg->sub_type);

    esp_lv_adapter_config_t adapter_cfg = ESP_LV_ADAPTER_DEFAULT_CONFIG();
    // Keep LVGL on core 1 with a larger PSRAM-backed stack; high-resolution UI and
    // app switching can otherwise starve the GUI task or overflow the default stack.
    adapter_cfg.task_stack_size = 16 * 1024;
    adapter_cfg.task_priority = 10;
    adapter_cfg.task_core_id = 1;
    adapter_cfg.stack_in_psram = true;
    ESP_RETURN_ON_FALSE(esp_lv_adapter_init(&adapter_cfg) == ESP_OK, NULL, TAG, "Initialize LVGL adapter failed");

    esp_lv_adapter_display_config_t disp_cfg = ESP_LV_ADAPTER_DISPLAY_MIPI_DEFAULT_CONFIG(
                lcd_handles->panel_handle, lcd_handles->io_handle, lcd_cfg->lcd_width, lcd_cfg->lcd_height,
                ESP_LV_ADAPTER_ROTATE_0);
    // board_devices.yaml allocates three DPI frame buffers for the triple-buffered partial mode.
    disp_cfg.tear_avoid_mode = ESP_LV_ADAPTER_TEAR_AVOID_MODE_TRIPLE_PARTIAL;
    lv_display_t *disp = esp_lv_adapter_register_display(&disp_cfg);
    if (!disp) {
        ESP_LOGE(TAG, "Register LVGL display failed");
        goto err_adapter;
    }

    dev_lcd_touch_handles_t *touch_handles = NULL;
    if (esp_board_manager_get_device_handle(ESP_BOARD_DEVICE_NAME_LCD_TOUCH, (void **)&touch_handles) == ESP_OK
            && touch_handles && touch_handles->touch_handle) {
        const esp_lv_adapter_touch_config_t touch_cfg = ESP_LV_ADAPTER_TOUCH_DEFAULT_CONFIG(disp, touch_handles->touch_handle);
        s_touch_indev = esp_lv_adapter_register_touch(&touch_cfg);
        if (s_touch_indev) {
            s_touch = touch_handles->touch_handle;
        } else {
            ESP_LOGW(TAG, "Register LVGL touch failed, continue without touch");
        }
    }

    if (esp_lv_adapter_start() != ESP_OK) {
        ESP_LOGE(TAG, "Start LVGL adapter failed");
        goto err_disp;
    }

    s_panel = lcd_handles->panel_handle;
    ESP_LOGI(TAG, "LVGL display started: %dx%d, touch=%s", lcd_cfg->lcd_width, lcd_cfg->lcd_height,
             s_touch ? "yes" : "no");
    return disp;

err_disp:
    if (s_touch_indev) {
        esp_lv_adapter_unregister_touch(s_touch_indev);
        s_touch_indev = NULL;
        s_touch = NULL;
    }
    esp_lv_adapter_unregister_display(disp);
err_adapter:
    esp_lv_adapter_deinit();
    return NULL;
}

lv_display_t *board_display_start(void)
{
    if (s_display) {
        return s_display;
    }

    /*
     * The waveshare/esp_lcd_jd9365 driver writes the panel power IC (0x45) during panel creation
     * through i2c_bus_create(I2C_NUM_1) and never deletes that bus. i2c_bus reuses a bus that already
     * exists on the port, so create the board manager i2c_master first; otherwise the driver claims
     * I2C1 and touch and the audio codecs can no longer acquire it. The reference is held for good.
     */
    if (!s_i2c_master) {
        ESP_RETURN_ON_FALSE(esp_board_periph_ref_handle(ESP_BOARD_PERIPH_NAME_I2C_MASTER, &s_i2c_master) == ESP_OK,
                            NULL, TAG, "Initialize i2c_master failed");
    }

    ESP_RETURN_ON_FALSE(esp_board_manager_init_device_by_name(ESP_BOARD_DEVICE_NAME_DISPLAY_LCD) == ESP_OK, NULL, TAG,
                        "Initialize display_lcd failed");
    if (esp_board_manager_init_device_by_name(ESP_BOARD_DEVICE_NAME_LCD_TOUCH) != ESP_OK) {
        ESP_LOGW(TAG, "Initialize lcd_touch failed, continue without touch");
    }
    if (board_backlight_init() != ESP_OK) {
        ESP_LOGW(TAG, "Initialize backlight failed, brightness control unavailable");
        s_backlight = NULL;
    }

    s_display = board_display_lvgl_init();
    if (!s_display) {
        esp_board_manager_deinit_device_by_name(ESP_BOARD_DEVICE_NAME_LCD_BRIGHTNESS);
        esp_board_manager_deinit_device_by_name(ESP_BOARD_DEVICE_NAME_LCD_TOUCH);
        esp_board_manager_deinit_device_by_name(ESP_BOARD_DEVICE_NAME_DISPLAY_LCD);
        s_backlight = NULL;
    }
    return s_display;
}

esp_lcd_panel_handle_t board_display_get_panel_handle(void)
{
    return s_panel;
}

esp_lcd_touch_handle_t board_display_get_touch_handle(void)
{
    return s_touch;
}

esp_err_t board_display_get_info(board_display_info_t *info)
{
    ESP_RETURN_ON_FALSE(info, ESP_ERR_INVALID_ARG, TAG, "Invalid display info pointer");

    dev_display_lcd_config_t *lcd_cfg = NULL;
    ESP_RETURN_ON_ERROR(esp_board_manager_get_device_config(ESP_BOARD_DEVICE_NAME_DISPLAY_LCD, (void **)&lcd_cfg), TAG,
                        "Get display_lcd config failed");
    ESP_RETURN_ON_FALSE(lcd_cfg && strcmp(lcd_cfg->sub_type, ESP_BOARD_DEVICE_LCD_SUB_TYPE_DSI) == 0,
                        ESP_ERR_NOT_SUPPORTED, TAG, "Only MIPI DSI panels are supported");

    uint8_t num_fbs = lcd_cfg->sub_cfg.dsi.dpi_config.num_fbs;
    if (num_fbs == 0) {
        num_fbs = 1;  // esp_lcd allocates one frame buffer when num_fbs is left at 0
    }
    *info = (board_display_info_t) {
        .width = lcd_cfg->lcd_width,
        .height = lcd_cfg->lcd_height,
        .bits_per_pixel = lcd_cfg->bits_per_pixel,
        .frame_buffer_num = num_fbs > BOARD_DISPLAY_MAX_FRAME_BUFFERS ? BOARD_DISPLAY_MAX_FRAME_BUFFERS : num_fbs,
    };
    return ESP_OK;
}

esp_err_t board_display_brightness_set(int brightness)
{
    ESP_RETURN_ON_FALSE(s_backlight, ESP_ERR_INVALID_STATE, TAG, "Backlight is not initialized");

    if (brightness > 100) {
        brightness = 100;
    } else if (brightness < 0) {
        brightness = 0;
    }

    const uint32_t duty = (uint32_t)brightness * s_backlight_duty_max / 100;
    ESP_RETURN_ON_ERROR(ledc_set_duty(s_backlight->speed_mode, s_backlight->channel, duty), TAG, "Set LEDC duty failed");
    ESP_RETURN_ON_ERROR(ledc_update_duty(s_backlight->speed_mode, s_backlight->channel), TAG, "Update LEDC duty failed");
    s_brightness = brightness;
    return ESP_OK;
}

int board_display_brightness_get(void)
{
    return s_brightness;
}

esp_err_t board_display_backlight_on(void)
{
    return board_display_brightness_set(100);
}

esp_err_t board_display_backlight_off(void)
{
    return board_display_brightness_set(0);
}

bool board_display_lock(int32_t timeout_ms)
{
    return esp_lv_adapter_lock(timeout_ms) == ESP_OK;
}

void board_display_unlock(void)
{
    esp_lv_adapter_unlock();
}
