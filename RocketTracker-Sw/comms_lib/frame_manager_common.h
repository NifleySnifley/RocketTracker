#ifndef FRAME_MANAGER_COMMON_H
#define FRAME_MANAGER_COMMON_H

#define CRC_POLY 0x8d95
extern uint16_t CRC_TABLE[256];

typedef struct Frame {
	uint16_t id;
	uint16_t crc;
	// uint8_t data[256 - sizeof(uint16_t) * 2];
	uint8_t* data;
} Frame;

const size_t FRAME_METADATA_SIZE = 2 * sizeof(uint16_t);

#pragma pack(push,1)
typedef struct Datum_Info {
	uint8_t type;
	uint16_t length;
} Datum_Info;
#pragma pack(pop)

#endif