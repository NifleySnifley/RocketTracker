#include <stdint.h>

enum MessageTypeIden {
    MSGTYPE_BROADCAST,
    MSGTYPE_DATA_DOWNLINK,
    MSGTYPE_TRANSACTION_INIT,
    MSGTYPE_TRANSACTION_END,
    MSGTYPE_TRANSACTION_DATA,
    MSGTYPE_CONFIG_SET,
    MSGTYPE_CONFIG_GET,
    MSGTYPE_PING,
    MSGTYPE_CONNECTION_CONFIG,
    MSGTYPE_GET_INFO,
};

enum BroadcastMessageCategory {
    BM_POSITION_BROADCAST,
    BM_INFLIGHT_UPDATE,
    BM_IDLE,
    BM_STATUS_UPDATE,
};

enum ConnectionCfgMessageCategory {
    CC_CONNECT,
    CC_DISCONNECT
};

enum ConfigMessageCategory {
    CM_ASK,
    CM_REPLY,
};

typedef struct RadioMessage {
    uint8_t message_type;       // 0
    uint8_t message_category;   // 1
    uint8_t sender;             // 2
    uint8_t RESERVED;           // 3
    uint32_t tracker_id;        // 4 5 6 7
    uint32_t seeker_id;         // 8 9 10 11
    uint32_t message_idx;       // 12 13 14 15
    uint8_t data[239];          // 15-254
} RadioMessage;