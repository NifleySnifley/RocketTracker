// Pinout for U12 (ESP32-S3-WROOM-1)
#define PIN_EXP1 (gpio_num_t) 4
#define PIN_EXP2 (gpio_num_t) 5
#define PIN_EXP3 (gpio_num_t) 6
#define PIN_EXP4 (gpio_num_t) 7
#define PIN_LED_G (gpio_num_t) 15
#define PIN_LED_R (gpio_num_t) 16
#define PIN_GPS_SDI (gpio_num_t) 17
#define PIN_GPS_SDO (gpio_num_t) 18
#define PIN_USB_N (gpio_num_t) 19
#define PIN_USB_P (gpio_num_t) 20
#define PIN_ADXL_INT1 (gpio_num_t) 3
#define PIN_ADXL_CS (gpio_num_t) 9
#define PIN_LPS_CS (gpio_num_t) 10
#define PIN_MOSI (gpio_num_t) 11
#define PIN_SCK (gpio_num_t) 12
#define PIN_MISO (gpio_num_t) 13
#define PIN_LIS3MDL_CS (gpio_num_t) 14
#define PIN_LSM6DSM_CS (gpio_num_t) 21
#define PIN_ESP_BOOT (gpio_num_t) 0
#define PIN_RFM_RST (gpio_num_t) 35
#define PIN_RFM_D0 (gpio_num_t) 36
#define PIN_RFM_CS (gpio_num_t) 37
#define PIN_FLASH_CS (gpio_num_t) 38
#define PIN_TL_SCK (gpio_num_t) 39
#define PIN_TL_MOSI (gpio_num_t) 40
#define PIN_TL_MISO (gpio_num_t) 41
// #define PIN_NETNU12NRXD0 RXD0
// #define PIN_NETNU12NTXD0 TXD0
#define PIN_SCL (gpio_num_t) 2
#define PIN_SDA (gpio_num_t) 1

#define I2C_MAIN_PORT I2C_NUM_0

#define SENSORS_SPI SPI2_HOST
#define SENSORS_SPI_FREQ 5 * 1000 * 1000 // 2 Mhz

#define TL_SPI SPI3_HOST
#define TL_SPI_FREQ 10E6
#define LOG_PARTITION_LABEL "log"
#define ESP_PARTITION_TYPE_DATA_LOG (esp_partition_type_t)0x40
#define ESP_PARTITION_SUBTYPE_DATA_LOG (esp_partition_subtype_t)0x00

#define UART_GPS UART_NUM_1

#define USB_SERIAL_BUF_SIZE 1024*5
#define USB_SER_ESC 0xFF
#define USB_SER_ESC_ESC 0x01
#define USB_SER_ESC_NULL 0x02
#define USB_TIMEOUT_MS 500

// Switches the pins of the debug UART (UART0) to be pins on the expansion header
#define DEBUG_ON_EXP true

#if (DEBUG_ON_EXP)
#define DEBUG_UART_RX PIN_EXP1
#define DEBUG_UART_TX PIN_EXP2
#endif