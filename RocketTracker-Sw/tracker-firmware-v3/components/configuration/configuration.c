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
void config_reset_to_default(int manifest_idx) {
    if (manifest_idx > 0) {
        config_entry_t cfg = CONFIG_MANIFEST[manifest_idx];
        esp_err_t e = nvs_erase_key(GLOBAL_CONFIG.nvs, cfg.hashed_key);
        if (e != ESP_OK && (e != ESP_ERR_NVS_NOT_FOUND)) {
            ESP_LOGE("CONFIG", "Error erasing config value!");
        }
    }
}