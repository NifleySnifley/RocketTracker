#include "configuration.h"
#include "configutils.h"
#include "esp_log.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "esp_assert.h"

#include "sx127x.h"
int config_convert_bandwidth(LoRa_Bandwidth_t bw) {
    switch (bw) {

        case _7_8KHZ:
            return SX127x_BW_7800;
        case _10_4KHZ:
            return SX127x_BW_10400;
        case _15_6KHZ:
            return SX127x_BW_15600;
        case _20_8KHZ:
            return SX127x_BW_20800;
        case _31_25KHZ:
            return SX127x_BW_31250;
        case _41_7KHZ:
            return SX127x_BW_41700;
        case _62_5KHZ:
            return SX127x_BW_62500;
        case _125KHZ:
            return SX127x_BW_125000;
        case _250KHZ:
            return SX127x_BW_250000;
        case _500KHZ:
            return SX127x_BW_500000;
        default:
            break;
    }
    return SX127x_BW_125000;
}

Config config_handle_request(Config* cmd, DatumTypeID* typeout) {
    Config response = {
        .has_error = false
    };

    if (cmd->mode == ConfigMode_ConfigErase) {
        // Set config value to default by erasing it
        response.mode = ConfigMode_ConfigSetResponse;


        int idx = config_get_manifest_idx_from_hashed(cmd->key_hashed);
        if (idx < 0) {
            response.has_error = true;
            response.error = 1;
        } else {
            config_reset_to_default(idx);
        }

        // link_send_datum(DatumTypeID_RESP_ConfigSet, Resp_BasicError_fields, &resp);
        *typeout = DatumTypeID_RESP_ConfigSet;
    } else if (cmd->mode == ConfigMode_ConfigGet) {
        // Return current value of a config parameter
        response.mode = ConfigMode_ConfigGetResponse;

        memcpy(response.key_hashed, cmd->key_hashed, sizeof(response.key_hashed));

        int idx = config_get_manifest_idx_from_hashed(cmd->key_hashed);

        if (idx < 0) {
            // Error, return no data
            ESP_LOGE("LINK", "Error getting config value with key hash '%s'", cmd->key_hashed);
            response.has_error = true;
            response.error = ESP_ERR_NOT_FOUND;
        } else {
            esp_err_t e = ESP_OK;
            config_type_t type = CONFIG_MANIFEST[idx].type;
            switch (type) {
                case FloatType:
                    response.has_float_value = true;
                    e = config_get_float(cmd->key_hashed, &response.float_value);
                    break;
                case IntType:
                    response.has_int_value = true;
                    e = config_get_int(cmd->key_hashed, &response.int_value);
                    break;
                case EnumType:
                    response.has_enum_value = true;
                    e = config_get_enum(cmd->key_hashed, &response.enum_value);
                    break;
                case StringType:
                    response.has_string_value = true;
                    e = config_get_string(cmd->key_hashed, response.string_value);
                    break;
                case BoolType:
                    response.has_bool_value = true;
                    e = config_get_bool(cmd->key_hashed, &response.bool_value);
                    break;
            }
            if (e != ESP_OK) {
                response.has_error = true;
                response.error = e;
            }
        }
        // link_send_datum(DatumTypeID_RESP_ConfigValue, Config_fields, &response);
        *typeout = DatumTypeID_RESP_ConfigValue;
    } else if (cmd->mode == ConfigMode_ConfigSet) {
        response.mode = ConfigMode_ConfigSetResponse;
        esp_err_t e = ESP_OK;

        // Actually set
        if (cmd->has_bool_value) {
            e = config_set_bool(cmd->key_hashed, cmd->bool_value);
        } else if (cmd->has_enum_value) {
            e = config_set_enum(cmd->key_hashed, cmd->enum_value);
        } else if (cmd->has_float_value) {
            e = config_set_float(cmd->key_hashed, cmd->float_value);
        } else if (cmd->has_int_value) {
            e = config_set_int(cmd->key_hashed, cmd->int_value);
        } else if (cmd->has_string_value) {
            e = config_set_string(cmd->key_hashed, cmd->string_value);
        }

        if (e != ESP_OK) {
            ESP_LOGE("CONFIG", "Error setting config value '%s'", esp_err_to_name(e));
            response.has_error = true;
            response.error = e;
        }

        *typeout = DatumTypeID_RESP_ConfigSet;
    } else if (cmd->mode == ConfigMode_ConfigGetResponse || cmd->mode == ConfigMode_ConfigSetResponse) {
        ESP_LOGW("LINK", "Error, received config response! tracker is not supposed to be responded to!!!");
        response.has_error = true;
        response.error = 0;
    } else if (cmd->mode == ConfigMode_ConfigClear) {
        // Erase all values in the NVS namespace
        esp_err_t e = nvs_erase_all(GLOBAL_CONFIG.nvs);
        if (e != ESP_OK) {
            response.has_error = true;
            response.error = e;
        } else {
            nvs_commit(GLOBAL_CONFIG.nvs);
            ESP_LOGI("CONFIG", "Successfully erased config");
            response.has_error = false;
        }
        *typeout = DatumTypeID_RESP_ConfigSet;
    }

    return response;
}

Config config_value_as_message(config_entry_t entry) {
    Config message = {
        .mode = ConfigMode_ConfigEnumerateResponse
    };
    memcpy(message.key_hashed, entry.hashed_key, strlen(entry.hashed_key) + 1);

    switch (entry.type) {
        case IntType:
            message.has_int_value = true;
            config_get_int((char*)entry.hashed_key, &message.int_value);
            break;
        case FloatType:
            message.has_float_value = true;
            config_get_float((char*)entry.hashed_key, &message.float_value);
            break;
        case EnumType:
            message.has_enum_value = true;
            config_get_enum((char*)entry.hashed_key, &message.enum_value);
            break;
        case StringType:
            message.has_string_value = true;
            config_get_string((char*)entry.hashed_key, message.string_value);
            break;
        case BoolType:
            message.has_bool_value = true;
            config_get_bool((char*)entry.hashed_key, &message.bool_value);
            break;
    }

    return message;
}

bool config_get_bool_inline(char* key_hash) {
    bool val = false;
    config_get_bool(key_hash, &val);
    return val;
}

int32_t config_get_int_inline(char* key_hash) {
    int32_t val = 0;
    config_get_int(key_hash, &val);
    return val;
}

float config_get_float_inline(char* key_hash) {
    float val = 0.0f;
    config_get_float(key_hash, &val);
    return val;
}