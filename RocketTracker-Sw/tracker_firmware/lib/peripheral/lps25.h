#ifndef LPS25_H
#define LPS25_H

#define TAG_LPS25 "LPS25"
#define LPS25_REG_REF_P_XL  0x08
#define LPS25_REG_REF_P_L  0x09
#define LPS25_REG_REF_P_H  0x0A
#define LPS25_REG_WHO_AM_I  0x0F
#define LPS25_WHOAMI 0b10111101
#define LPS25_REG_RES_CONF  0x10
#define LPS25_REG_CTRL_REG1  0x20
#define LPS25_REG_CTRL_REG2  0x21
#define LPS25_REG_CTRL_REG3  0x22
#define LPS25_REG_CTRL_REG4  0x23
#define LPS25_REG_INTERRUPT_CFG  0x24
#define LPS25_REG_INT_SOURCE  0x25
#define LPS25_REG_STATUS_REG  0x27
#define LPS25_REG_PRESS_OUT_XL  0x28
#define LPS25_REG_PRESS_OUT_L  0x29
#define LPS25_REG_PRESS_OUT_H  0x2A
#define LPS25_REG_TEMP_OUT_L  0x2B
#define LPS25_REG_TEMP_OUT_H  0x2C
#define LPS25_REG_FIFO_CTRL  0x2E
#define LPS25_REG_FIFO_STATUS  0x2F
#define LPS25_REG_THS_P_L  0x30
#define LPS25_REG_THS_P_H  0x31
#define LPS25_REG_RPDS_L  0x39
#define LPS25_REG_RPDS_H  0x3A

#include "stdint.h"
#include "driver/i2c.h"

class LPS25 {
private:
	i2c_port_t port;
	uint8_t addr;
	uint8_t rw_dummy;

	void reg_write(uint8_t reg, uint8_t val);

	uint8_t reg_read(uint8_t reg);

public:
	LPS25(i2c_port_t port);

	bool init();

	uint32_t pressure_raw();

	float get_pressure_hpa();

	float get_temperature_c();

	float get_altitude_m();
};

#endif