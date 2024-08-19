#ifndef TLA2024_H
#define TLA2024_H

#define TLA2024_ADDR 0b1001000
#define TLA2024_REG_CONVERSION 0x00
#define TLA2024_REG_CONFIGURATION 0x01

#include "driver/i2c_master.h"

enum TLA2024_DataRate {
    TLA2024_DR_128Hz,
    TLA2024_DR_250Hz,
    TLA2024_DR_490Hz,
    TLA2024_DR_920Hz,
    TLA2024_DR_1600Hz,
    TLA2024_DR_2400Hz,
    TLA2024_DR_3300Hz,
};

enum TLA2024_PGAConfig {
    TLA2024_PGA_6v144,
    TLA2024_PGA_4v096,
    TLA2024_PGA_2v048,
    TLA2024_PGA_1v024,
    TLA2024_PGA_0v512,
    TLA2024_PGA_0v256,
};

enum TLA2024_MUXConfig {
    TLA2024_MUX_1N_0P,
    TLA2024_MUX_3N_0P,
    TLA2024_MUX_3N_1P,
    TLA2024_MUX_3N_2P,
    TLA2024_MUX_0P,
    TLA2024_MUX_1P,
    TLA2024_MUX_2P,
    TLA2024_MUX_3P,
};

typedef union TLA2024_Configuration_t {
    struct {
        uint16_t rsvd_0x03 : 5;
        uint16_t datarate : 3;
        uint16_t opmode : 1;
        uint16_t pga_gain : 3;
        uint16_t mux_sel : 3;
        uint16_t status : 1;
    };
    uint16_t _reg;
} TLA2024_Configuration_t;

typedef struct TLA2024_t {
    i2c_master_dev_handle_t device;
    TLA2024_Configuration_t config;
} TLA2024_t;

esp_err_t tla2024_init(TLA2024_t* adc, i2c_master_bus_handle_t bus);
void tla2024_write(TLA2024_t* adc, uint8_t addr, uint16_t value);
uint16_t tla2024_read(TLA2024_t* adc, uint8_t addr);

int16_t tla2024_read_active_channel_raw(TLA2024_t* adc);
float tla2024_read_active_channel_mv(TLA2024_t* adc);
float tla2024_lsb2mv(TLA2024_t* adc, int16_t lsb);
void tla2024_set_active_channel(TLA2024_t* adc, int channel);

#endif