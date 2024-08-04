#include "prelog.h"
#include "memory.h"
#include "esp_log.h"

void prelog_init(prelog_t* prelog, logger_t* logger, int capacity) {
    prelog->logger = logger;
    prelog->state = PRELOG_STANDBY;

    prelog->prelog_ringbuf_fill = 0;
    prelog->prelog_ringbuf_idx = 0;
    prelog->prelog_ringbuf_capacity = capacity;
    prelog->prelog_ringbuf = (prelog_data_wrapper_t*)malloc(sizeof(prelog_data_wrapper_t) * capacity);
}

// Call this from the task where data is being generated for logging (high speed)
void prelog_give_data(prelog_t* prelog, log_data_default_t logdata) {
    if (prelog->state == PRELOG_RECORDING) {
        // Write to ringbuf;
        prelog_data_wrapper_t data = {
            .timestamp = util_time_us(),
            .data = logdata
        };
        prelog->prelog_ringbuf[prelog->prelog_ringbuf_idx++] = data;
        prelog->prelog_ringbuf_idx %= prelog->prelog_ringbuf_capacity;

        if (prelog->prelog_ringbuf_fill < prelog->prelog_ringbuf_capacity) {
            prelog->prelog_ringbuf_fill++;
        }
    }
}
// Call this from the task that does normal logging, won't flush if not set to 
void prelog_flush_some(prelog_t* prelog) {
    if (prelog->state == PRELOG_FLUSHING) {
        // Flush
        if (prelog->prelog_ringbuf_fill > 0) {

            // Walk the index back one
            prelog->prelog_ringbuf_idx = (prelog->prelog_ringbuf_idx - 1) % prelog->prelog_ringbuf_capacity;
            if (prelog->prelog_ringbuf_idx < 0)
                prelog->prelog_ringbuf_idx = prelog->prelog_ringbuf_capacity - 1;
            // ESP_LOGI("PRELOG", "Flushing at idx=%d", prelog->prelog_ringbuf_idx);
            // Grab the data and flush it
            prelog_data_wrapper_t data_to_flush = prelog->prelog_ringbuf[prelog->prelog_ringbuf_idx];
            logger_log_data(prelog->logger, LOG_DTYPE_DATA_DEFAULT, (uint8_t*)&data_to_flush.data, sizeof(log_data_default_t), data_to_flush.timestamp);
            // Unfill
            prelog->prelog_ringbuf_fill--;
        } else {
            // Done flushing
            prelog->state = PRELOG_STANDBY;
        }
    }
}


void prelog_reset_buffer(prelog_t* prelog) {
    prelog->prelog_ringbuf_fill = 0;
    prelog->prelog_ringbuf_idx = 0;
    // No need to actually erase the memory!
}

void prelog_start_recording(prelog_t* prelog) {
    ESP_LOGI("PRELOG", "Starting recording");
    if (prelog->state == PRELOG_FLUSHING) {
        // Stop
        prelog_reset_buffer(prelog);
    }
    prelog->state = PRELOG_RECORDING;
}
void prelog_start_flushing(prelog_t* prelog) {
    ESP_LOGI("PRELOG", "Starting flush");
    prelog->state = PRELOG_FLUSHING;
}