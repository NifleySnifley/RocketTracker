#include "lps25.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "math.h"

void LPS25::reg_write(uint8_t reg, uint8_t v) {
	uint8_t cmd[2] = { reg, v };
	i2c_master_write_to_device(this->port, this->addr, cmd, 2, 1000 / portTICK_PERIOD_MS);
}

uint8_t LPS25::reg_read(uint8_t reg) {
	uint8_t data = 0;
	i2c_master_write_read_device(this->port, this->addr, &reg, 1, &data, 1, 1000 / portTICK_PERIOD_MS);
	return data;
}

LPS25::LPS25(i2c_port_t port) : port(port), addr(0x5D) {}

bool LPS25::init() {
	if (reg_read(LPS25_REG_WHO_AM_I) != LPS25_WHOAMI) {
		ESP_LOGE(TAG_LPS25, "Error identifying sensor");
		return false;
	}

	// BDU, 25Hz, Power On
	reg_write(LPS25_REG_CTRL_REG1, 0b11000100);

	// 8 sample moving average
	reg_write(LPS25_REG_FIFO_CTRL, 0b11000111);

	return true;
}

uint32_t LPS25::pressure_raw() {
	return (reg_read(LPS25_REG_PRESS_OUT_H) << 16) | (reg_read(LPS25_REG_PRESS_OUT_L) << 8) | reg_read(LPS25_REG_PRESS_OUT_XL);
}

// hPa
float LPS25::get_pressure_hpa() {
	return (float)pressure_raw() / 4096.0f;
}

float LPS25::get_temperature_c() {
	int16_t raw = (reg_read(LPS25_REG_TEMP_OUT_H) << 8) | reg_read(LPS25_REG_TEMP_OUT_L);
	return (((float)raw) / 480.f) + 42.5f;
}

float LPS25::get_altitude_m() {
	return 44330.f * (1 - powf(get_pressure_hpa() / 1013.25f, 0.190284f));
}