#include <stdint.h>

typedef struct RadioMessage {
    uint8_t message_type;
    uint8_t message_category;
    uint32_t message_idx;
    uint8_t RESERVED[9];
    uint8_t data[240];
} RadioMessage;