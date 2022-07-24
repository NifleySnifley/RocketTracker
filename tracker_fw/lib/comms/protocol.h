#include <stdint.h>

enum MessageTypeIden {
    MSGTYPE_BROADCAST,
    MSGTYPE_DATA_DOWNLINK,
    MSGTYPE_TRANSACTION_INIT,
    MSGTYPE_TRANSACTION_END,
    MSGTYPE_ENTRY_SET,
    MSGTYPE_ENTRY_GET,
    MSGTYPE_PING,
    MSGTYPE_CONNECTION_CONFIG
};

enum BroadcastMessageCategory {
    BM_POSITION_BROADCAST,
    BM_INFLIGHT_UPDATE,
    BM_IDLE,
    BM_STATUS_UPDATE
};

enum ConnectionCfgMessageCategory {
    CC_CONNECT,
    CC_DISCONNECT
};

typedef struct RadioMessage {
    uint8_t message_type;
    uint8_t message_category;
    uint8_t RESERVED_1;
    uint8_t RESERVED_2;
    uint8_t RESERVED_3;
    uint8_t RESERVED_4;
    uint8_t RESERVED_5;
    uint8_t RESERVED_6;
    uint32_t message_idx;
    uint8_t data[240];
} RadioMessage;