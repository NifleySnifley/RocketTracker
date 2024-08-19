#ifndef CONFIGURATION_LOCAL_H
#define CONFIGURATION_LOCAL_H

#include "nvs.h"
#include "esp_log.h"
#include "config_generated.h"
#include "fmgr.h"
#include "protocol.pb.h"

int config_convert_bandwidth(LoRa_Bandwidth_t bw);
Config config_handle_request(Config* req, DatumTypeID* typeout);
Config config_value_as_message(config_entry_t entry);

bool config_get_bool_inline(char* key_hash);
int32_t config_get_int_inline(char* key_hash);
float config_get_float_inline(char* key_hash);

#endif