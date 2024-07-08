#ifndef LOGGING_H
#define LOGGING_H

#include "esp_partition.h"
#include "esp_flash.h"
#include "nvs.h"

#define NVS_LOG_START "log_start"
#define NVS_LOG_END "log_end"

// Actual: 32000000 bytes (256Mbit)
#define LOG_MEMORY_SIZE_B 32000000
#define LOG_FLASH_SECTOR_SIZE 4096

#define LOGGER_COBS_ESC 0x0
#define LOGGER_COBS_ESC_00 0x01
#define LOGGER_COBS_ESC_FF 0x02

typedef struct logger_t {
    esp_flash_t* ext_flash;
    nvs_handle_t log_nvs;

    size_t log_start_address;
    size_t log_end_address;

    const esp_partition_t* log_partition;
    uint8_t* log_buffer;
} logger_t;

#define LOG_FLAG_PRESS_FRESH (1<<0)
#define LOG_FLAG_MAG_FRESH (1<<1)
#define LOG_FLAG_LSM_ACC_FRESH (1<<2)
#define LOG_FLAG_LSM_GYR_FRESH (1<<3)
#define LOG_FLAG_ADXL_ACC_FRESH (1<<4)
#define LOG_FLAG_GPS_FRESH (1<<5)
typedef struct log_data_t {
    // Sensor data
    uint32_t lps_press_raw;
    uint16_t lis_mag_raw[3];
    uint16_t lsm_acc_raw[3];
    uint16_t lsm_gyr_raw[3];
    float adxl_acc_raw[3]; // TYPE TBD!!!
    // Processed data
    float orientation_quat[4];
    float filtered_altitude_m;
    // From GPS
    float gps_lat;
    float gps_lon;
    float gps_alt;
    uint8_t flags;
} log_data_t;

// TODO: Implement COBS for log data aswell
// Instead of 0 being the delimiter, make 0xFF the delimiter!!
// That way, whenever 0xFF is seen, it is known that that 0xFF marks the start of clean erased flash memory
// As a result, NVS only needs to be written once per second or so, and then a simple search can be done to find the logical end of the log!
// If variable-length data can be written to the log, 0xFF can be used as a delimiter, and 0xFFFF will then mark the start of erased areas

// 1 <- DONE
esp_err_t logger_init(logger_t* logger, esp_flash_t* flash);
// 3.75 <- DONE
esp_err_t logger_refresh(logger_t* logger);

// TODO:
// LATER (handle ring buffer/wrapping)
esp_err_t logger_log_data_now(logger_t* logger, uint8_t* data, size_t data_length);

// DONE
esp_err_t logger_flush(logger_t* logger);

// TODO:
// LATER
// Pass starting state of first address through state
// Leave the log in COBS format! that way log data delimiters are saved
esp_err_t logger_iter(logger_t* logger, size_t* state, uint8_t* chunk_buffer, size_t chunk_size);

// 4 <- DONE
esp_err_t logger_write_bytes_raw(logger_t* logger, size_t start_address, size_t data_length, uint8_t* data);

// 2 <- DONE
esp_err_t logger_read_bytes_raw(logger_t* logger, size_t start_address, size_t size, uint8_t* buffer);

// 3.5 <- DONE
esp_err_t logger_erase_log(logger_t* logger);

esp_err_t logger_clean_log(logger_t* logger);

// 3 <- DONE
size_t logger_get_current_log_size(logger_t* logger);

#endif