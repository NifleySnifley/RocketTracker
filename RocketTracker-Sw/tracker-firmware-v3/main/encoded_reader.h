#ifndef ENCODED_READER_H
#define ENCODED_READER_H

#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>

typedef enum encoded_reader_state_t {
    READER_STATE_READ_LEN_MSB,
    READER_STATE_READ_LEN_LSB,
    READER_STATE_READ_DATA,
    READER_STATE_DONE,
    READER_STATE_ESC
} encoded_reader_state_t;

typedef struct encoded_reader_t {
    encoded_reader_state_t state;
    encoded_reader_state_t state_ret;
    uint16_t frame_size;
    uint8_t val_unescaped;
    size_t frame_idx;

    uint8_t* buffer;
    size_t buffer_len;
} encoded_reader_t;

void encoded_reader_init(encoded_reader_t* reader, uint8_t* buffer, size_t buffer_size);
void encoded_reader_give_byte(encoded_reader_t* reader, uint8_t byte);
void encoded_reader_reset(encoded_reader_t* reader);
bool encoded_reader_has_frame(encoded_reader_t* reader);
uint8_t* encoded_reader_get_frame(encoded_reader_t* reader, size_t* len_out);

#endif