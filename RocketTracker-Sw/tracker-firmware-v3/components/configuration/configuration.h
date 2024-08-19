#ifndef CONFIGURATION_H
#define CONFIGURATION_H

#include "stdint.h"
#include "stdbool.h"
#include "nvs.h"
#include "esp_log.h"
#include "config_generated.h"

typedef struct config_provider_t {
    nvs_handle_t nvs;
} config_provider_t;

extern config_provider_t GLOBAL_CONFIG;

typedef int config_iterator_t;

void init_config(const char* nvs_namespace);

// Stuff for enumerating through the CONFIG_MANIFEST
config_iterator_t config_iterator_first();
config_iterator_t config_iterator_next(config_iterator_t iterator);
bool config_iterator_done(config_iterator_t iterator);
config_entry_t config_iterator_get_value(config_iterator_t iterator);

esp_err_t config_get_bool(char* key_hash, bool* out);
// Must be 64 long!!!!!
esp_err_t config_get_string(char* key_hash, char* out_buf);
esp_err_t config_get_float(char* key_hash, float* out);
esp_err_t config_get_int(char* key_hash, int32_t* out);
esp_err_t config_get_enum(char* key_hash, int32_t* out);

// Always returns static pointer
int config_get_manifest_idx_from_hashed(const char* key_hashed);
int config_get_manifest_idx_from_key(const char* key);
const char* config_get_key_hash(const char* key);

esp_err_t config_set_bool(char* key_hash, bool val);
esp_err_t config_set_string(char* key_hash, char* val);
esp_err_t config_set_float(char* key_hash, float val);
esp_err_t config_set_int(char* key_hash, int32_t val);
esp_err_t config_set_enum(char* key_hash, int32_t val);

void config_reset_to_default(int manifest_idx);
bool config_is_set(char* key_hash);

#endif