#include "tla2024.h"
#include "esp_log.h"

esp_err_t tla2024_init(TLA2024_t* adc, i2c_master_bus_handle_t bus) {
    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = TLA2024_ADDR,
        .scl_speed_hz = 400000
    };

    esp_err_t e = i2c_master_bus_add_device(bus, &dev_config, &adc->device);
    if (e != ESP_OK) {
        return e;
    }

    adc->config = (TLA2024_Configuration_t){
        .datarate = TLA2024_DR_3300Hz,
        .pga_gain = TLA2024_PGA_4v096,
        .rsvd_0x03 = 0x03,
        .mux_sel = TLA2024_MUX_0P,
        .opmode = 0, // Continuous conversion
    };
    tla2024_write(adc, TLA2024_REG_CONFIGURATION, adc->config._reg);

    return ESP_OK;
}

void tla2024_write(TLA2024_t* adc, uint8_t addr, uint16_t value) {
    uint8_t buffer[3] = {
            addr,
            (value >> 8) & 0xFF,
            (value) & 0xFF
    };
    esp_err_t e = i2c_master_transmit(adc->device, &buffer, sizeof(buffer), 10);
    if (e != ESP_OK) {
        ESP_LOGE("TLA2024", "Error writing I2C: %s", esp_err_to_name(e));
    }
}

uint16_t tla2024_read(TLA2024_t* adc, uint8_t addr) {
    uint8_t rxbuf[2];
    esp_err_t e = i2c_master_transmit(adc->device, &addr, 1, 10);
    e |= i2c_master_receive(adc->device, rxbuf, sizeof(rxbuf), 10);
    // esp_err_t e = i2c_master_transmit_receive(adc->device, &addr, 1, &rx, 2, 10);
    if (e != ESP_OK) {
        ESP_LOGE("TLA2024", "Error reading I2C: %s", esp_err_to_name(e));
    }
    return ((uint16_t)rxbuf[0]) << 8 | ((uint16_t)rxbuf[1]);
}

int16_t tla2024_read_active_channel_raw(TLA2024_t* adc) {
    return ((int16_t)tla2024_read(adc, TLA2024_REG_CONVERSION)) >> 4;
}

float tla2024_lsb2mv(TLA2024_t* adc, int16_t lsb) {
    // LSB Size = FSR / 4096
    float convfac = 0.0;
    switch (adc->config.pga_gain) {
        case TLA2024_PGA_0v256:
            convfac = 0.125f;
            break;
        case TLA2024_PGA_0v512:
            convfac = 0.25f;
            break;
        case TLA2024_PGA_1v024:
            convfac = 0.5f;
            break;
        case TLA2024_PGA_2v048:
            convfac = 1.0f;
            break;
        case TLA2024_PGA_4v096:
            convfac = 2.0f;
            break;
        case TLA2024_PGA_6v144:
            convfac = 3.0f;
            break;
        default:
            convfac = 1.0f;
            break;
    }
    return convfac * (float)lsb;
}

float tla2024_read_active_channel_mv(TLA2024_t* adc) {
    return tla2024_lsb2mv(adc, tla2024_read_active_channel_raw(adc));
}

void tla2024_set_active_channel(TLA2024_t* adc, int channel) {
    switch (channel) {
        case 0:
            adc->config.mux_sel = TLA2024_MUX_0P;
            break;
        case 1:
            adc->config.mux_sel = TLA2024_MUX_1P;
            break;
        case 2:
            adc->config.mux_sel = TLA2024_MUX_2P;
            break;
        case 3:
            adc->config.mux_sel = TLA2024_MUX_3P;
            break;
        default:
            return;
    }
    tla2024_write(adc, TLA2024_REG_CONFIGURATION, adc->config._reg);
}