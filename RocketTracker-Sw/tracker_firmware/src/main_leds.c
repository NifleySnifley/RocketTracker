#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/gpio.h"

#include "pindefs.h"

#define DBG_TAG "DEBUG"
#define GATTS_TAG "GATT"

// Bluetooth configuration
#define NUM_BLE_APPS 1
#define CONFIG_APP_ID 0


// void led_task(void* args) {
// 	while (1) {
// 		gpio_set_level(TRACKER_LED_RED, 1);
// 		gpio_set_level(TRACKER_LED_GRN, 1);
// 		vTaskDelay(500);
// 		gpio_set_level(TRACKER_LED_RED, 0);
// 		gpio_set_level(TRACKER_LED_GRN, 1);
// 		vTaskDelay(500);
// 		gpio_set_level(TRACKER_LED_RED, 0);
// 		gpio_set_level(TRACKER_LED_GRN, 0);
// 		vTaskDelay(500);
// 		gpio_set_level(TRACKER_LED_RED, 1);
// 		gpio_set_level(TRACKER_LED_GRN, 0);
// 		vTaskDelay(500);

// 		ESP_LOGI(DBG_TAG, "Cycle!");
// 	}
// }

// void app_main() {
// 	gpio_reset_pin(TRACKER_LED_RED);
// 	gpio_reset_pin(TRACKER_LED_GRN);
// 	gpio_set_direction(TRACKER_LED_RED, GPIO_MODE_OUTPUT);
// 	gpio_set_direction(TRACKER_LED_GRN, GPIO_MODE_OUTPUT);

// 	esp_log_level_set("*", ESP_LOG_VERBOSE);

// 	xTaskCreate(led_task, "led_blinky_task", 1024 * 2, NULL, configMAX_PRIORITIES, NULL);

// 	while (1) {
// 		vTaskDelay(1000);
// 	}
// }
