#include <stdbool.h>
#include "configuration.h"

// Battery monitoring frequency (hz)
// Frequency at which the battery monitor IC is polled for SOC.
#define BATT_MON_HZ 0.1f

// Frequency at which sensor reads & filter updates are performed at
#define SENSOR_HZ 256

// Frequency at which data is output over USB for debugging
// #define DEBUG_MON_HZ 1.0f
// #define DEBUG_MON_ENABLED false
// #define DEBUG_MON_LPS22 true
// #define DEBUG_MON_LSM6DSM true
// #define DEBUG_MON_LIS3MDL true
// #define DEBUG_MON_ADXL true


// Noise is about +-0.5g
#define LIFTOFF_ACC_THRESHOLD_G 4.0
#define LIFTOFF_DURATION 10.0
#define LAND_THRESHOLD_G 2
#define LAND_TIME 10.0

#define BARO_CAL_OFFSET 63.0527133333f