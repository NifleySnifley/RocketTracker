#ifndef TCA6408_H
#define TCA6408_H

#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "stdbool.h"
#include "stdint.h"

#define TCA6408_REG_INPUT 0
#define TCA6408_REG_OUTPUT 1
#define TCA6408_REG_POLARITY_INVERSION 2
#define TCA6408_REG_CONFIGURATION 3

#define TCA6408_ADDR 0x20

typedef struct TCA6408_t {
    uint8_t output_state;
    uint8_t output_mask;

    i2c_master_dev_handle_t device;
} TCA6408_t;

esp_err_t tca6408_init(TCA6408_t* exp, i2c_master_bus_handle_t bus);
void tca6408_write(TCA6408_t* exp, uint8_t addr, uint8_t value);
uint8_t tca6408_read(TCA6408_t* exp, uint8_t addr);


void tca6408_set_pindir(TCA6408_t* exp, int pin, bool is_output);
void tca6408_set_level(TCA6408_t* exp, int pin, bool output);
bool tca6408_get_level(TCA6408_t* exp, int pin);

#endif