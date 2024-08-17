#ifndef DDBOARD_H
#define DDBOARD_H

#include "stdint.h"
#include "stdbool.h"
#include "led_strip.h"
#include "esp_log.h"
#include "tca6408.h"
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

    i2c_master_bus_handle_t i2c;
    TCA6408_t pyro_expander;

    TaskHandle_t task_handle;
} dd_board_t;

/// Returns true on success
esp_err_t dd_board_init(dd_board_t* board, gpio_num_t pin_leds, gpio_num_t pin_serial_rx, gpio_num_t pin_sda, gpio_num_t pin_scl);

void dd_board_set_indicator(dd_board_t* board, int indicator, uint8_t red, uint8_t green, uint8_t blue);

#endif