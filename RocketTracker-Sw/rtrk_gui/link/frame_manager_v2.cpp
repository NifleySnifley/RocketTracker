#include "frame_manager_v2.hpp"
#include <memory>
#include "protocol.pb.h"

uint16_t CRC_TABLE[256] = { 0x0,0x8d95,0x96bf,0x1b2a,0xa0eb,0x2d7e,0x3654,0xbbc1,0xcc43,0x41d6,0x5afc,0xd769,0x6ca8,0xe13d,0xfa17,0x7782,0x1513,0x9886,0x83ac,0xe39,0xb5f8,0x386d,0x2347,0xaed2,0xd950,0x54c5,0x4fef,0xc27a,0x79bb,0xf42e,0xef04,0x6291,0x2a26,0xa7b3,0xbc99,0x310c,0x8acd,0x758,0x1c72,0x91e7,0xe665,0x6bf0,0x70da,0xfd4f,0x468e,0xcb1b,0xd031,0x5da4,0x3f35,0xb2a0,0xa98a,0x241f,0x9fde,0x124b,0x961,0x84f4,0xf376,0x7ee3,0x65c9,0xe85c,0x539d,0xde08,0xc522,0x48b7,0x544c,0xd9d9,0xc2f3,0x4f66,0xf4a7,0x7932,0x6218,0xef8d,0x980f,0x159a,0xeb0,0x8325,0x38e4,0xb571,0xae5b,0x23ce,0x415f,0xccca,0xd7e0,0x5a75,0xe1b4,0x6c21,0x770b,0xfa9e,0x8d1c,0x89,0x1ba3,0x9636,0x2df7,0xa062,0xbb48,0x36dd,0x7e6a,0xf3ff,0xe8d5,0x6540,0xde81,0x5314,0x483e,0xc5ab,0xb229,0x3fbc,0x2496,0xa903,0x12c2,0x9f57,0x847d,0x9e8,0x6b79,0xe6ec,0xfdc6,0x7053,0xcb92,0x4607,0x5d2d,0xd0b8,0xa73a,0x2aaf,0x3185,0xbc10,0x7d1,0x8a44,0x916e,0x1cfb,0xa898,0x250d,0x3e27,0xb3b2,0x873,0x85e6,0x9ecc,0x1359,0x64db,0xe94e,0xf264,0x7ff1,0xc430,0x49a5,0x528f,0xdf1a,0xbd8b,0x301e,0x2b34,0xa6a1,0x1d60,0x90f5,0x8bdf,0x64a,0x71c8,0xfc5d,0xe777,0x6ae2,0xd123,0x5cb6,0x479c,0xca09,0x82be,0xf2b,0x1401,0x9994,0x2255,0xafc0,0xb4ea,0x397f,0x4efd,0xc368,0xd842,0x55d7,0xee16,0x6383,0x78a9,0xf53c,0x97ad,0x1a38,0x112,0x8c87,0x3746,0xbad3,0xa1f9,0x2c6c,0x5bee,0xd67b,0xcd51,0x40c4,0xfb05,0x7690,0x6dba,0xe02f,0xfcd4,0x7141,0x6a6b,0xe7fe,0x5c3f,0xd1aa,0xca80,0x4715,0x3097,0xbd02,0xa628,0x2bbd,0x907c,0x1de9,0x6c3,0x8b56,0xe9c7,0x6452,0x7f78,0xf2ed,0x492c,0xc4b9,0xdf93,0x5206,0x2584,0xa811,0xb33b,0x3eae,0x856f,0x8fa,0x13d0,0x9e45,0xd6f2,0x5b67,0x404d,0xcdd8,0x7619,0xfb8c,0xe0a6,0x6d33,0x1ab1,0x9724,0x8c0e,0x19b,0xba5a,0x37cf,0x2ce5,0xa170,0xc3e1,0x4e74,0x555e,0xd8cb,0x630a,0xee9f,0xf5b5,0x7820,0xfa2,0x8237,0x991d,0x1488,0xaf49,0x22dc,0x39f6,0xb463 };


FrameManager2::FrameManager2(int maxsize) {
	this->frame_buffer = (uint8_t*)malloc(maxsize);
	this->owns_buffer = true;
	this->cur_frame_len = 0;
	this->buffer_size = maxsize;

	this->frame.data = &this->frame_buffer[FRAME_METADATA_SIZE];
}

FrameManager2::FrameManager2(uint8_t* buffer, size_t buffer_length) {
	this->owns_buffer = true;
	this->frame_buffer = (uint8_t*)malloc(buffer_length);
	std::memcpy(this->frame_buffer, buffer, buffer_length);

	this->frame.data = &this->frame_buffer[FRAME_METADATA_SIZE];
	this->owns_buffer = false;
	this->buffer_size = buffer_length;
	this->cur_frame_len = buffer_length - FRAME_METADATA_SIZE;

	std::memcpy(&this->frame, this->frame_buffer, FRAME_METADATA_SIZE);
}

// TODO: manage the ownership of this
std::vector<std::tuple<DatumTypeID, std::shared_ptr<std::vector<uint8_t>>>> FrameManager2::get_datums() {
	std::vector<std::tuple<DatumTypeID, std::shared_ptr<std::vector<uint8_t>>>> datums = {};

	int datum_i = 0;
	int data_ptr = 0;

	// Seek datum info
	while ((data_ptr + sizeof(Datum_Info)) <= this->cur_frame_len) {
		Datum_Info* info = (Datum_Info*)&this->frame.data[data_ptr];
		data_ptr += sizeof(Datum_Info);
		if (info->length + data_ptr > this->cur_frame_len) {
			std::cerr << "Error, malformed frame: not enough data for datum body" << std::endl;
			return datums; // Frame is malformed, not enough data for the datum's body
		}

		auto v = std::make_shared<std::vector<uint8_t>>();
		v->resize(info->length);
		std::copy(&this->frame.data[data_ptr], &this->frame.data[data_ptr + info->length], v->begin());
		/*
		&this->frame.data[data_ptr],
			info->length*/
		std::tuple<DatumTypeID, std::shared_ptr<std::vector<uint8_t>>> datum = {
			(DatumTypeID)info->type,
			v
		};

		datums.push_back(datum);

		data_ptr += info->length;
		++datum_i;
	}

	return datums;
}

bool FrameManager2::encode_datum(DatumTypeID id, google::protobuf::Message* datum) {
	if (datum == nullptr) {
		return false;
	}
	else {
		// datum->SerializeToArray();
	}
}

size_t FrameManager2::get_num_datums() {
	return this->num_datums;
}


uint16_t FrameManager2::calculate_crc() {
	uint16_t crc = 0;
	for (int i = 0; i < this->cur_frame_len; ++i) {
		uint8_t prev = ((crc & 0xFF00) >> 8) ^ frame.data[i];
		crc = ((crc << 8) ^ CRC_TABLE[prev]) & 0xFFFF;
	}
	return crc;
}

bool FrameManager2::check_crc() {
	uint16_t cur_crc = calculate_crc();
	return this->frame.crc == cur_crc;
}

void FrameManager2::reset() {
	std::memset(this->frame_buffer, 0, this->buffer_size);
	this->frame.crc = 0;
	this->frame.id = TalkerID::Basestation;
	this->cur_frame_len = 0;
	this->num_datums = 0;
}

uint8_t* FrameManager2::get_data(size_t* length_out) {
	this->frame.id = TalkerID::Basestation;
	this->frame.crc = this->calculate_crc();
	std::memcpy(this->frame_buffer, &this->frame, FRAME_METADATA_SIZE);

	*length_out = this->cur_frame_len + FRAME_METADATA_SIZE;
	return this->frame_buffer;
}

FrameManager2::~FrameManager2() {
	if (this->owns_buffer) {
		free(this->frame_buffer);
	}
}

using namespace google::protobuf;

std::shared_ptr<Message> decode_datum(DatumTypeID id, std::shared_ptr<std::vector<uint8_t>> payload)
{
	std::shared_ptr<Message> decoded;
	switch (id)
	{
	case INFO_Blank:
		decoded = nullptr;
		break;
	case INFO_Raw:
		decoded = nullptr;
		break;
	case INFO_Battery:
	{
		auto msg = std::make_shared<Battery>();
		msg->ParseFromArray(payload->data(), payload->size());
		decoded = std::dynamic_pointer_cast<Message>(msg);
	}
	break;
	case INFO_GPS:
	{
		auto msg = std::make_shared<GPS>();
		msg->ParseFromArray(payload->data(), payload->size());
		decoded = std::dynamic_pointer_cast<Message>(msg);
	}
	break;
	case INFO_Altitude:
	{
		auto msg = std::make_shared<Altitude>();
		msg->ParseFromArray(payload->data(), payload->size());
		decoded = std::dynamic_pointer_cast<Message>(msg);
	}
	break;
	case INFO_Orientation:
	{
		auto msg = std::make_shared<Orientation>();
		msg->ParseFromArray(payload->data(), payload->size());
		decoded = std::dynamic_pointer_cast<Message>(msg);
	}
	break;
	case INFO_Alert:
	{
		auto msg = std::make_shared<Alert>();
		msg->ParseFromArray(payload->data(), payload->size());
		decoded = std::dynamic_pointer_cast<Message>(msg);
	}
	break;
	case INFO_LogStatus:
	{
		auto msg = std::make_shared<LogStatus>();
		msg->ParseFromArray(payload->data(), payload->size());
		decoded = std::dynamic_pointer_cast<Message>(msg);
	}
	break;
	case IMFO_SensorData:
	{
		auto msg = std::make_shared<SensorData>();
		msg->ParseFromArray(payload->data(), payload->size());
		decoded = std::dynamic_pointer_cast<Message>(msg);
	}
	break;
	case STATUS_RadioRxStatus:
	{
		auto msg = std::make_shared<RadioRxStatus>();
		msg->ParseFromArray(payload->data(), payload->size());
		decoded = std::dynamic_pointer_cast<Message>(msg);
	}
	break;
	case CMD_Ping:
	{
		auto msg = std::make_shared<Command_Ping>();
		msg->ParseFromArray(payload->data(), payload->size());
		decoded = std::dynamic_pointer_cast<Message>(msg);
	}
	break;
	case RESP_Ping:
	{
		auto msg = std::make_shared<Resp_Ping>();
		msg->ParseFromArray(payload->data(), payload->size());
		decoded = std::dynamic_pointer_cast<Message>(msg);
	}
	break;
	case CMD_ConfigureLogging:
	{
		auto msg = std::make_shared<Command_ConfigureLogging>();
		msg->ParseFromArray(payload->data(), payload->size());
		decoded = std::dynamic_pointer_cast<Message>(msg);
	}
	break;

	case CMD_EraseLog:
	{
		auto msg = std::make_shared<Command_EraseLog>();
		msg->ParseFromArray(payload->data(), payload->size());
		decoded = std::dynamic_pointer_cast<Message>(msg);
	}
	break;
	case CMD_DownloadLog:
		decoded = nullptr;
		break;
	case RESP_DownloadLog_Segment:
	{
		auto msg = std::make_shared<Resp_DownloadLog_Segment>();
		msg->ParseFromArray(payload->data(), payload->size());
		decoded = std::dynamic_pointer_cast<Message>(msg);
	}
		break;
	case ACK_Download_Complete:
	{
		auto msg = std::make_shared<Acknowledgement_Download_Complete>();
		msg->ParseFromArray(payload->data(), payload->size());
		decoded = std::dynamic_pointer_cast<Message>(msg);
	}
		break;
	case CMD_ConfigSensorOutput:
	{
		auto msg = std::make_shared<Command_ConfigSensorOutput>();
		msg->ParseFromArray(payload->data(), payload->size());
		decoded = std::dynamic_pointer_cast<Message>(msg);
	}
		break;
	case RESP_ConfigSensorOutput:
		decoded = nullptr;
		break;
	case CMD_LogStatus:
		decoded = nullptr;
		break;

	case RESP_DownloadLog:
	case RESP_EraseLog:
	case RESP_ConfigureLogging: // All basic responses here!
	{
		auto msg = std::make_shared<Resp_BasicError>();
		msg->ParseFromArray(payload->data(), payload->size());
		decoded = std::dynamic_pointer_cast<Message>(msg);
	}
	break;
	default:
		decoded = nullptr;
		break;
	}

	return decoded;
}
