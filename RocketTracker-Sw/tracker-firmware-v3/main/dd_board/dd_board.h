#ifndef DDBOARD_H
#define DDBOARD_H

#include "stdint.h"
#include "stdbool.h"
#include "led_strip.h"
#include "esp_log.h"
#include "driver/gptimer.h"
#include "esp_timer.h"
#include "configuration.h"
#include "logging.h"

#include "tca6408.h"
#include "tla2024.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define TCA_PIN_PYRO_1_CTL 0
#define TCA_PIN_PYRO_1_STAT 1
#define TCA_PIN_PYRO_2_CTL 2
#define TCA_PIN_PYRO_2_STAT 3
#define TCA_PIN_PYRO_3_CTL 4
#define TCA_PIN_PYRO_3_STAT 5
#define TCA_PIN_PYRO_4_CTL 6
#define TCA_PIN_PYRO_4_STAT 7

typedef struct dd_board_t {
    led_strip_handle_t leds;
    int32_t led_brightness;

    i2c_master_bus_handle_t i2c;
    TCA6408_t pyro_expander;
    TLA2024_t adc;

    uint16_t adc_channel_reading[4]; // read by adc_reader_task at a configurable rate up to 1100Hz???? wow that fast.
    // TODO: How to allow multicore processing of sensor fresh... this is a BAD situation
    EventGroupHandle_t adc_reading_fresh;

    TaskHandle_t adc_reader_task;
    float adc_hz;
    bool adc_channel_enabled[4];
    int adc_num_channels_enabled;


    esp_timer_handle_t adc_timer;

    TaskHandle_t task_handle;

    logger_t* logger;
} dd_board_t;

typedef union led_color_t {
    uint32_t c;
    struct {
        uint8_t r, g, b;
    };
} led_color_t;

#define LED_COLOR_RED ((led_color_t){.r=255,.g=0,.b=0})
#define LED_COLOR_GREEN ((led_color_t){.r=0,.g=255,.b=0})
#define LED_COLOR_BLUE ((led_color_t){.r=0,.g=0,.b=255})
#define LED_COLOR_WHITE ((led_color_t){.r=255,.g=255,.b=255})
#define LED_COLOR_NONE ((led_color_t){.r=0,.g=0,.b=0})

/// Returns true on success
esp_err_t dd_board_init(dd_board_t* board, gpio_num_t pin_leds, gpio_num_t pin_serial_rx, gpio_num_t pin_sda, gpio_num_t pin_scl, logger_t* logger);

void dd_board_set_indicator_rgb(dd_board_t* board, int indicator, uint8_t red, uint8_t green, uint8_t blue);
void dd_board_set_indicator_color(dd_board_t* board, int indicator, led_color_t color);

/// Call this from the global logging function
void dd_board_log_now(dd_board_t* board);

#endif