#ifndef FRAME_MANAGER_H
#define FRAME_MANAGER_H

#include <stdint.h>
#include <protocol.pb.h>
#include <pb_encode.h>
#include <pb_decode.h>

#define CRC_POLY 0x8d95

typedef struct Frame {
	uint16_t id;
	uint16_t crc;
	uint8_t data[256 - sizeof(uint16_t) * 2];
} Frame;

typedef struct Datum_Info {
	uint8_t type;
	uint8_t length;
} Datum_Info;

typedef void (*datum_decoded_callback)(int i, MessageTypeID type, int length, uint8_t* data);

class FrameManager {
private:
	pb_ostream_t buf_serializer;
	pb_istream_t buf_deserializer;
	uint8_t tmp_buf[sizeof(Frame::data) - sizeof(Datum_Info)];

public:
	Frame frame;
	int cur_frame_len;
	int n_datum_encoded;

	FrameManager();

	// Serialize a message and add it to the current frame
	bool encode_datum(MessageTypeID type, const pb_msgdesc_t* fields, const void* src_struct);

	// Clear the buffer and reset state
	void reset();

	uint16_t calculate_crc();

	bool check_crc();

	// Returns a pointer to the current serialized data buffer of the frame and writes its length to `buflen`
	uint8_t* get_frame(int* buflen);

	// First step in decoding a frame, load it into the buffer (NOTE: this will overwrite any existing frame data)
	bool load_frame(uint8_t* data, int length);

	// Decode the current frame datum by datum using a callback to process each datum segment
	bool decode_frame(datum_decoded_callback callback);
};

bool FrameManager_ser_callback(pb_ostream_t* stream, const pb_byte_t* buf, size_t count);

#endif