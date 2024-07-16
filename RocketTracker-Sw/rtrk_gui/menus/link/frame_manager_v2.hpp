#ifndef FRAME_MANAGER_V2_H
#define FRAME_MANAGER_V2_H

#include "stdint.h"
#include <memory>
#include <vector>
#include <iostream>
#include "frame_manager_common.h"
#include "protocol.pb.h"

class FrameManager2 {
private:
	Frame frame;
	uint8_t* frame_buffer; // Alloc to maxsize
	size_t buffer_size = 0;
	bool owns_buffer = false;
	size_t num_datums = 0;
	size_t cur_frame_len;

public:
	FrameManager2(int maxsize);
	FrameManager2(uint8_t* buffer, size_t buffer_length);

	std::vector<std::tuple<DatumTypeID, std::shared_ptr<std::vector<uint8_t>>>> get_datums();

	bool encode_datum(DatumTypeID id, google::protobuf::Message* datum);

	size_t get_num_datums();

	uint16_t calculate_crc();

	bool check_crc();

	void reset();

	uint8_t* get_data(size_t* length_out);

	~FrameManager2();
};

#endif