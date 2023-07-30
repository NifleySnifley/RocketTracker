#ifndef PINDEFS_H
#define PINDEFS_H

#include "driver/gpio.h"

#define TRACKER_LED_RED (gpio_num_t)32
#define TRACKER_LED_GRN (gpio_num_t)33 

// GPS (NMEA) on UART2
#define TRACKER_GPS_RX (gpio_num_t)16
#define TRACKER_GPS_TX (gpio_num_t)17
#define TRACKER_GPS_FIX (gpio_num_t)26

// I2C (Qwiic)
#define TRACKER_SDA (gpio_num_t)21
#define TRACKER_SCL (gpio_num_t)22

// SPI (RFM97)
#define TRACKER_CIPO (gpio_num_t)19
#define TRACKER_SCK (gpio_num_t)18
#define TRACKER_COPI (gpio_num_t)23
#define TRACKER_RFM_CS (gpio_num_t)5
#define TRACKER_RFM_RST (gpio_num_t)27

#endif