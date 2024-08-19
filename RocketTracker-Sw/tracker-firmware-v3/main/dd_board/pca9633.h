#ifndef PCA9633_H
#define PCA9633_H

#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "stdbool.h"
#include "stdint.h"

#define PCA9633_ADDR 0b1100010

#define PCA9633_REG_MODE1 0x00
#define PCA9633_REG_MODE2 0x01
#define PCA9633_REG_PWM0 0x02
#define PCA9633_REG_PWM1 0x03
#define PCA9633_REG_PWM2 0x04
#define PCA9633_REG_PWM3 0x05
#define PCA9633_REG_GRPPWM 0x06
#define PCA9633_REG_GRPFREQ 0x07
#define PCA9633_REG_LEDOUT 0x08
#define PCA9633_REG_SUBADR1 0x09
#define PCA9633_REG_SUBADR2 0x0A
#define PCA9633_REG_SUBADR3 0x0B
#define PCA9633_REG_ALLCALLADR 0x0C

#define PCA9633_AI_NONE 0b000
#define PCA9633_AI_ALL 0b100
#define PCA9633_AI_PWM 0b101
#define PCA9633_AI_GLBCTL 0b110
#define PCA9633_AI_LOCCTL 0b111

typedef union PCA9633_Addr_t {
    uint8_t reg;
    struct {
        uint8_t regaddr : 5;
        uint8_t AI : 3;
    };
} PCA9633_Addr_t;

typedef union PCA9633_MODE1_t {
    uint8_t reg;
    struct {
        uint8_t allcall : 1;
        uint8_t sub3 : 1;
        uint8_t sub2 : 1;
        uint8_t sub1 : 1;
        uint8_t sleep : 1;
        uint8_t ro_AI : 3;
    };
} PCA9633_MODE1_t;

typedef union PCA9633_MODE2_t {
    uint8_t reg;
    struct {
        uint8_t : 2;
        uint8_t outdrv : 1;
        uint8_t och : 1;
        uint8_t invrt : 1;
        uint8_t dmblnk : 1;
        uint8_t : 2;
    };
} PCA9633_MODE2_t;


#define PCA9633_LDR_OFF 0b00
#define PCA9633_LDR_ON 0b01
#define PCA9633_LDR_INDIVIDUAL 0b10
#define PCA9633_LDR_GROUP 0b11
typedef union PCA9633_LDR_t {
    uint8_t reg;
    struct {
        uint8_t ldr0 : 2;
        uint8_t ldr1 : 2;
        uint8_t ldr2 : 2;
        uint8_t ldr3 : 2;
    };
} PCA9633_LDR_t;

typedef struct PCA9633_t {
    i2c_master_dev_handle_t device;
    PCA9633_LDR_t ldr;
} PCA9633_t;

esp_err_t pca9633_init(PCA9633_t* exp, i2c_master_bus_handle_t bus);
void pca9633_write(PCA9633_t* exp, uint8_t addr, uint8_t input);
uint8_t pca9633_read(PCA9633_t* exp, uint8_t addr);

void pca9633_set_channel_level(PCA9633_t* exp, int channel, bool level);

#endif