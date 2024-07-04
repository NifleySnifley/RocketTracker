#include <stdbool.h>

// Battery monitoring frequency (hz)
// Frequency at which the battery monitor IC is polled for SOC.
#define BATT_MON_HZ 0.1f

// Frequency at which sensor reads & filter updates are performed at
#define SENSOR_HZ 256

// Frequency at which data is output over USB for debugging
#define DEBUG_MON_HZ 100.0f
#define DEBUG_MON_ENABLED true