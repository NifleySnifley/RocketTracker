#ifndef SENSORS_H
#define SENSORS_H

#include "lps22hh_reg.h"
#include "lis3mdl_reg.h"
#include "lsm6dsm_reg.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "stdint.h"
#include "lis3mdl_reg.h"
#include "lps22hh_reg.h"
#include "lsm6dsm_reg.h"
#include "adxl375.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>
#include "esp_log.h"
#include "logging.h"
#include "defs.h"
#include "memory.h"
#include "Fusion.h"

extern stmdev_ctx_t lps22;
extern spi_device_handle_t lps22_device;

extern stmdev_ctx_t lis3mdl;
extern spi_device_handle_t lis3mdl_device;

extern stmdev_ctx_t lsm6dsm;
extern spi_device_handle_t lsm6dsm_device;

extern adxl375_handle_t adxl;
extern spi_device_handle_t adxl_device;
extern TaskHandle_t adxl_int_handler;

extern TaskHandle_t sensors_worker_task;
extern esp_timer_handle_t sensor_timer;

extern volatile float pressure_hPa;
extern volatile float magnetic_mG[3];
extern volatile float acceleration_g[3];
extern volatile float angular_rate_dps[3];
extern volatile float temperature_degC;
extern volatile float adxl_acceleration_g[3];

// No calibration!
extern volatile float pressure_hPa_raw;
extern volatile float magnetic_mG_raw[3];
extern volatile float acceleration_g_raw[3];
extern volatile float angular_rate_dps_raw[3];
extern volatile float temperature_degC_raw;
extern volatile float adxl_acceleration_g_raw[3];
extern volatile float altitude_m_raw;

extern bool LIS3MDL_DISABLE;
extern bool LSM6DSM_DISABLE;
extern bool ADXL_DISABLE;
extern bool LPS22_DISABLE;

extern FusionAhrs ahrs;
extern volatile float altitude_m;
extern volatile float v_speed_m_s;

typedef struct calibration_2pt_t {
    float offset;
    float scale;
} calibration_2pt_t;

float apply_2pt_calibration(float value, calibration_2pt_t calibration);
void apply_2pt_calibrations_inplace(float* value, calibration_2pt_t* calibrations, int n);

void init_sensor_spi();
void init_lps22();
void init_lis3mdl();
void init_lsm6dsm();
void init_adxl375(TaskHandle_t* interrupt_task);

void sensors_timer_cb(void* arg);
void sensors_worker(void* arg);
void sensors_routine(void* arg);

void init_sensors(SemaphoreHandle_t sem);

#endif