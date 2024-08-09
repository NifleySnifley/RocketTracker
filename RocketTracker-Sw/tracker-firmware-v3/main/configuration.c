#include "configuration.h"
#include "esp_log.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "esp_assert.h"

#define CONFIG_GENERATED_IMPL
#include "config_generated.h"
#undef CONFIG_GENERATED_IMPL

void config_print() {
    char string_array[64];
    for (config_iterator_t it = config_iterator_first(); !config_iterator_done(it); it = config_iterator_next(it)) {
        config_entry_t entry = config_iterator_get_value(it);
        config_value_t cur_value;
        cur_value.string_value = string_array;
        switch (entry.type) {
            case EnumType:
                config_get_enum((char*)entry.hashed_key, &cur_value.enum_value);
                ESP_LOGI("CONFIG", "%s (%s), type=enum, default=%" PRIi32 ", value=%" PRIu32, entry.key, entry.hashed_key, entry.default_value.enum_value, cur_value.enum_value);
                break;
            case IntType:
                config_get_int((char*)entry.hashed_key, &cur_value.int_value);
                ESP_LOGI("CONFIG", "%s (%s), type=int, default=%" PRIi32 ", value=%" PRIi32, entry.key, entry.hashed_key, entry.default_value.int_value, cur_value.int_value);
                break;

            case StringType:
                config_get_string((char*)entry.hashed_key, cur_value.string_value);
                ESP_LOGI("CONFIG", "%s (%s), type=string, default=%s, value=%s", entry.key, entry.hashed_key, entry.default_value.string_value, cur_value.string_value);
                break;

            case FloatType:
                config_get_float((char*)entry.hashed_key, &cur_value.float_value);
                ESP_LOGI("CONFIG", "%s (%s), type=float, default=%f, value=%f", entry.key, entry.hashed_key, entry.default_value.float_value, cur_value.float_value);
                break;

            case BoolType:
                config_get_bool((char*)entry.hashed_key, &cur_value.bool_value);
                ESP_LOGI("CONFIG", "%s (%s), type=bool, default=%d, value=%d", entry.key, entry.hashed_key, (int)entry.default_value.bool_value, cur_value.bool_value);
                break;

            default:
                break;
        }
    }

}

void init_config(const char* nvs_namespace) {
    esp_err_t e = nvs_open(nvs_namespace, NVS_READWRITE, &GLOBAL_CONFIG.nvs);
    if (e != ESP_OK) {
        ESP_LOGI("CONFIG", "Error (%s) opening config NVS handle!\n", esp_err_to_name(e));
    }

#if DEBUG_PRINT_CONFIG
    config_print();
#endif
}

bool config_is_set(char* key_hash) {
    nvs_type_t out_type;
    esp_err_t e = nvs_find_key(GLOBAL_CONFIG.nvs, key_hash, &out_type);
    if (e == ESP_OK) {
        return true;
    } else if (e == ESP_ERR_NVS_NOT_FOUND) {
        return false;
    } else {
        ESP_LOGW("CONFIG", "Error %s checking existance of config value", esp_err_to_name(e));
        return false;
    }
}

int config_get_manifest_idx_from_hashed(const char* key_hashed) {
    int i = 0;
    for (config_iterator_t it = config_iterator_first(); !config_iterator_done(it); it = config_iterator_next(it)) {
        config_entry_t entry = config_iterator_get_value(it);
        if (strcmp(key_hashed, entry.hashed_key) == 0) {
            return i;
        }
        ++i;
    }

    return -1;
}

int config_get_manifest_idx_from_key(const char* key) {
    int i = 0;
    for (config_iterator_t it = config_iterator_first(); !config_iterator_done(it); it = config_iterator_next(it)) {
        config_entry_t entry = config_iterator_get_value(it);
        if (strcmp(key, entry.key) == 0) {
            return i;
        }
        ++i;
    }

    return -1;
}

const char* config_get_key_hash(const char* key) {
    int idx = config_get_manifest_idx_from_key(key);
    if (idx == -1) {
        return NULL;
    } else {
        return CONFIG_MANIFEST[idx].hashed_key;
    }
}

// Stuff for enumerating through the CONFIG_MANIFEST
config_iterator_t config_iterator_first() {
    return 0;
}
config_iterator_t config_iterator_next(config_iterator_t iterator) {
    return iterator + 1;
}
bool config_iterator_done(config_iterator_t iterator) {
    return iterator >= (sizeof(CONFIG_MANIFEST) / sizeof(CONFIG_MANIFEST[0]));
}
config_entry_t config_iterator_get_value(config_iterator_t iterator) {
    return CONFIG_MANIFEST[(int)iterator];
}

esp_err_t config_get_bool(char* key_hash, bool* out) {
    int idx = config_get_manifest_idx_from_hashed(key_hash);
    if (idx < 0) {
        ESP_LOGE("CONFIG", "Could not find configuration value with hashed key '%s'", key_hash);
        return ESP_ERR_NOT_FOUND;
    }

    config_type_t type = CONFIG_MANIFEST[idx].type;
    if (type != BoolType) {
        ESP_LOGE("CONFIG", "Error retrieving value with hashed key '%s', type mismatch (wanted %d, got %d)", key_hash, (int)BoolType, (int)type);
        return ESP_ERR_NVS_TYPE_MISMATCH;
    }


    uint64_t v;
    esp_err_t e = nvs_get_u64(GLOBAL_CONFIG.nvs, key_hash, &v);
    if (e != ESP_OK && (e != ESP_ERR_NVS_NOT_FOUND)) {
        ESP_LOGE("CONFIG", "NVS Error: %s", esp_err_to_name(e));
        return e;
    } else if (e == ESP_ERR_NVS_NOT_FOUND) {
        *out = CONFIG_MANIFEST[idx].default_value.bool_value; // Set default
    } else {
        *out = v;
    }

    return ESP_OK;
}
esp_err_t config_get_string(char* key_hash, char* out_buf) {
    int idx = config_get_manifest_idx_from_hashed(key_hash);
    if (idx < 0) {
        ESP_LOGE("CONFIG", "Could not find configuration value with hashed key '%s'", key_hash);
        return ESP_ERR_NOT_FOUND;
    }

    config_type_t type = CONFIG_MANIFEST[idx].type;
    if (type != StringType) {
        ESP_LOGE("CONFIG", "Error retrieving value with hashed key '%s', type mismatch (wanted %d, got %d)", key_hash, (int)StringType, (int)type);
        return ESP_ERR_NVS_TYPE_MISMATCH;
    }

    // Set default value
    size_t len;
    esp_err_t e = nvs_get_str(GLOBAL_CONFIG.nvs, key_hash, out_buf, &len);
    if (e != ESP_OK && (e != ESP_ERR_NVS_NOT_FOUND)) {
        ESP_LOGE("CONFIG", "NVS Error: %s", esp_err_to_name(e));
        return e;
    } else if (e == ESP_ERR_NVS_NOT_FOUND) {
        memcpy(out_buf, CONFIG_MANIFEST[idx].default_value.string_value, strlen(CONFIG_MANIFEST[idx].default_value.string_value) + 1);
    }

    if (len <= 63) {
        out_buf[len] = '\0';
    }

    return ESP_OK;
}
esp_err_t config_get_float(char* key_hash, float* out) {
    int idx = config_get_manifest_idx_from_hashed(key_hash);
    if (idx < 0) {
        ESP_LOGE("CONFIG", "Could not find configuration value with hashed key '%s'", key_hash);
        return ESP_ERR_NOT_FOUND;
    }

    config_type_t type = CONFIG_MANIFEST[idx].type;
    if (type != FloatType) {
        ESP_LOGE("CONFIG", "Error retrieving value with hashed key '%s', type mismatch (wanted %d, got %d)", key_hash, (int)FloatType, (int)type);
        return ESP_ERR_NVS_TYPE_MISMATCH;
    }


    union {
        float value;
        int32_t storage;
    } format;

    ESP_STATIC_ASSERT(sizeof(format) == 4);

    esp_err_t e = nvs_get_i32(GLOBAL_CONFIG.nvs, key_hash, &format.storage);
    if (e != ESP_OK && (e != ESP_ERR_NVS_NOT_FOUND)) {
        ESP_LOGE("CONFIG", "NVS Error: %s", esp_err_to_name(e));
        return e;
    } else if (e == ESP_ERR_NVS_NOT_FOUND) {
        *out = CONFIG_MANIFEST[idx].default_value.float_value; // Set default
    } else {
        *out = format.value;
    }


    return ESP_OK;
}
esp_err_t config_get_int(char* key_hash, int32_t* out) {
    int idx = config_get_manifest_idx_from_hashed(key_hash);
    if (idx < 0) {
        ESP_LOGE("CONFIG", "Could not find configuration value with hashed key '%s'", key_hash);
        return ESP_ERR_NOT_FOUND;
    }

    config_type_t type = CONFIG_MANIFEST[idx].type;
    if (type != IntType) {
        ESP_LOGE("CONFIG", "Error retrieving value with hashed key '%s', type mismatch (wanted %d, got %d)", key_hash, (int)IntType, (int)type);
        return ESP_ERR_NVS_TYPE_MISMATCH;
    }


    esp_err_t e = nvs_get_i32(GLOBAL_CONFIG.nvs, key_hash, out);
    if (e != ESP_OK && (e != ESP_ERR_NVS_NOT_FOUND)) {
        ESP_LOGE("CONFIG", "NVS Error: %s", esp_err_to_name(e));
        return e;
    } else if (e == ESP_ERR_NVS_NOT_FOUND) {
        *out = CONFIG_MANIFEST[idx].default_value.int_value; // Set default
    }

    return ESP_OK;
}

esp_err_t config_get_enum(char* key_hash, int32_t* out) {
    int idx = config_get_manifest_idx_from_hashed(key_hash);
    if (idx < 0) {
        ESP_LOGE("CONFIG", "Could not find configuration value with hashed key '%s'", key_hash);
        return ESP_ERR_NOT_FOUND;
    }

    config_type_t type = CONFIG_MANIFEST[idx].type;
    if (type != EnumType) {
        ESP_LOGE("CONFIG", "Error retrieving value with hashed key '%s', type mismatch (wanted %d, got %d)", key_hash, (int)EnumType, (int)type);
        return ESP_ERR_NVS_TYPE_MISMATCH;
    }


    esp_err_t e = nvs_get_i32(GLOBAL_CONFIG.nvs, key_hash, out);
    if (e != ESP_OK && (e != ESP_ERR_NVS_NOT_FOUND)) {
        ESP_LOGE("CONFIG", "NVS Error: %s", esp_err_to_name(e));
        return e;
    } else if (e == ESP_ERR_NVS_NOT_FOUND) {
        *out = CONFIG_MANIFEST[idx].default_value.enum_value; // Set default
    }

    return ESP_OK;
}

esp_err_t config_set_bool(char* key_hash, bool val) {
    int idx = config_get_manifest_idx_from_hashed(key_hash);
    if (idx < 0) {
        ESP_LOGE("CONFIG", "Could not find configuration value with hashed key '%s'", key_hash);
        return ESP_ERR_NOT_FOUND;
    }

    config_type_t type = CONFIG_MANIFEST[idx].type;
    if (type != BoolType) {
        ESP_LOGE("CONFIG", "Error setting value with hashed key '%s', type mismatch (wanted %d, got %d)", key_hash, (int)BoolType, (int)type);
        return ESP_ERR_NVS_TYPE_MISMATCH;
    }


    uint64_t v = val;

    esp_err_t e = nvs_set_u64(GLOBAL_CONFIG.nvs, key_hash, v);
    if (e != ESP_OK) {
        ESP_LOGE("CONFIG", "NVS Error: %s", esp_err_to_name(e));
        return e;
    }

    nvs_commit(GLOBAL_CONFIG.nvs);
    return ESP_OK;
}
esp_err_t config_set_string(char* key_hash, char* val) {
    if (strlen(val) + 1 > 64) {
        return ESP_ERR_NOT_ALLOWED;
    }

    int idx = config_get_manifest_idx_from_hashed(key_hash);
    if (idx < 0) {
        ESP_LOGE("CONFIG", "Could not find configuration value with hashed key '%s'", key_hash);
        return ESP_ERR_NOT_FOUND;
    }

    config_type_t type = CONFIG_MANIFEST[idx].type;
    if (type != StringType) {
        ESP_LOGE("CONFIG", "Error setting value with hashed key '%s', type mismatch (wanted %d, got %d)", key_hash, (int)StringType, (int)type);
        return ESP_ERR_NVS_TYPE_MISMATCH;
    }

    esp_err_t e = nvs_set_str(GLOBAL_CONFIG.nvs, key_hash, val);
    if (e != ESP_OK && (e != ESP_ERR_NVS_NOT_FOUND)) {
        ESP_LOGE("CONFIG", "NVS Error: %s", esp_err_to_name(e));
        return e;
    }

    nvs_commit(GLOBAL_CONFIG.nvs);
    return ESP_OK;
}
esp_err_t config_set_float(char* key_hash, float val) {
    int idx = config_get_manifest_idx_from_hashed(key_hash);
    if (idx < 0) {
        ESP_LOGE("CONFIG", "Could not find configuration value with hashed key '%s'", key_hash);
        return ESP_ERR_NOT_FOUND;
    }

    config_type_t type = CONFIG_MANIFEST[idx].type;
    if (type != FloatType) {
        ESP_LOGE("CONFIG", "Error setting value with hashed key '%s', type mismatch (wanted %d, got %d)", key_hash, (int)FloatType, (int)type);
        return ESP_ERR_NVS_TYPE_MISMATCH;
    }

    union {
        float value;
        int32_t storage;
    } format;

    ESP_STATIC_ASSERT(sizeof(format) == 4);

    format.value = val;

    esp_err_t e = nvs_set_i32(GLOBAL_CONFIG.nvs, key_hash, format.storage);
    if (e != ESP_OK && (e != ESP_ERR_NVS_NOT_FOUND)) {
        ESP_LOGE("CONFIG", "NVS Error: %s", esp_err_to_name(e));
        return e;
    }

    nvs_commit(GLOBAL_CONFIG.nvs);
    return ESP_OK;
}

esp_err_t config_set_int(char* key_hash, int32_t val) {
    int idx = config_get_manifest_idx_from_hashed(key_hash);
    if (idx < 0) {
        ESP_LOGE("CONFIG", "Could not find configuration value with hashed key '%s'", key_hash);
        return ESP_ERR_NOT_FOUND;
    }

    config_type_t type = CONFIG_MANIFEST[idx].type;
    if (type != IntType) {
        ESP_LOGE("CONFIG", "Error setting value with hashed key '%s', type mismatch (wanted %d, got %d)", key_hash, (int)IntType, (int)type);
        return ESP_ERR_NVS_TYPE_MISMATCH;
    }


    esp_err_t e = nvs_set_i32(GLOBAL_CONFIG.nvs, key_hash, val);
    if (e != ESP_OK && (e != ESP_ERR_NVS_NOT_FOUND)) {
        ESP_LOGE("CONFIG", "NVS Error: %s", esp_err_to_name(e));
        return e;
    }

    nvs_commit(GLOBAL_CONFIG.nvs);
    return ESP_OK;
}

esp_err_t config_set_enum(char* key_hash, int32_t val) {
    int idx = config_get_manifest_idx_from_hashed(key_hash);
    if (idx < 0) {
        ESP_LOGE("CONFIG", "Could not find configuration value with hashed key '%s'", key_hash);
        return ESP_ERR_NOT_FOUND;
    }

    config_type_t type = CONFIG_MANIFEST[idx].type;
    if (type != EnumType) {
        ESP_LOGE("CONFIG", "Error setting value with hashed key '%s', type mismatch (wanted %d, got %d)", key_hash, (int)EnumType, (int)type);
        return ESP_ERR_NVS_TYPE_MISMATCH;
    }


    esp_err_t e = nvs_set_i32(GLOBAL_CONFIG.nvs, key_hash, val);
    if (e != ESP_OK && (e != ESP_ERR_NVS_NOT_FOUND)) {
        ESP_LOGE("CONFIG", "NVS Error: %s", esp_err_to_name(e));
        return e;
    }

    nvs_commit(GLOBAL_CONFIG.nvs);
    return ESP_OK;
}

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

void config_reset_to_default(int manifest_idx) {
    if (manifest_idx > 0) {
        config_entry_t cfg = CONFIG_MANIFEST[manifest_idx];
        esp_err_t e = nvs_erase_key(GLOBAL_CONFIG.nvs, cfg.hashed_key);
        if (e != ESP_OK && (e != ESP_ERR_NVS_NOT_FOUND)) {
            ESP_LOGE("CONFIG", "Error erasing config value!");
        }
    }
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
        *typeout = DatumTypeID_RESP_ConfigSet;

        ESP_LOGI("CONFIG", "Erasing configuration memory!");
        esp_err_t e = nvs_erase_all(GLOBAL_CONFIG.nvs);
        if (e != ESP_OK) {
            response.has_error = true;
            response.error = e;
        } else {
            response.has_error = false;
            ESP_LOGI("CONFIG", "Successfully erased configuration memory");
            nvs_commit(GLOBAL_CONFIG.nvs);
        }
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