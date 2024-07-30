#ifndef PRELOG_H
#define PRELOG_H
#include "logging.h"

typedef enum prelog_state_t {
    PRELOG_STANDBY,
    PRELOG_RECORDING,
    PRELOG_FLUSHING,
} prelog_state_t;

typedef struct prelog_data_wrapper_t {
    int64_t timestamp;
    log_data_default_t data;
} prelog_data_wrapper_t;

typedef struct prelog_t {
    logger_t* logger;
    int prelog_ringbuf_capacity;
    prelog_state_t state;

    prelog_data_wrapper_t* prelog_ringbuf;
    int prelog_ringbuf_idx;
    int prelog_ringbuf_fill;
} prelog_t;

void prelog_init(prelog_t* prelog, logger_t* logger, int capacity);

// Call this from the task where data is being generated for logging (high speed)
void prelog_give_data(prelog_t* prelog, log_data_default_t logdata);
// Call this from the task that does normal logging, won't flush if not set to 
void prelog_flush_some(prelog_t* prelog);
void prelog_reset_buffer(prelog_t* prelog);

void prelog_start_recording(prelog_t* prelog);
void prelog_start_flushing(prelog_t* prelog);


#endif