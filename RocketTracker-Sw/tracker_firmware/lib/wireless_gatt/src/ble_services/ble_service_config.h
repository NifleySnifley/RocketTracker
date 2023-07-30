
#ifndef CONFIG_SERVICE_H_
#define CONFIG_SERVICE_H_
#include <stdint.h>

#include "esp_gatt_defs.h"
#include "esp_gatts_api.h"

enum {
	// Service index
	CONFIG_SERV,

	// LED Declaration
	CONFIG_LED_CHAR,
	// LED Value
	CONFIG_LED_VAL,

	CONFIG_SERV_NUM_ATTR,
};

uint16_t CONFIG_handle_table[CONFIG_SERV_NUM_ATTR];

extern const uint16_t uuid_CONFIG_SERV;
extern const esp_gatts_attr_db_t CONFIG_serv_gatt_db[CONFIG_SERV_NUM_ATTR];

#endif /* CONFIG_SERVICE_H_ */
