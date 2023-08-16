#include <frame_manager.h>

FrameManager::FrameManager() {
	reset();
}

void FrameManager::reset() {
	this->frame.id = 0;
	memset(this->tmp_buf, 0, sizeof(this->tmp_buf));
	n_datum_encoded = 0;
	this->buf_serializer = pb_ostream_from_buffer(this->tmp_buf, sizeof(this->tmp_buf));
}

uint8_t* FrameManager::get_frame(int* len) {
	*len = this->cur_frame_len + sizeof(Frame::id);
	return (uint8_t*)&this->frame;
}

bool FrameManager::encode_datum(MessageTypeID type, const pb_msgdesc_t* fields, const void* src_struct) {
	this->buf_serializer = pb_ostream_from_buffer(this->tmp_buf, sizeof(this->tmp_buf));
	bool success = pb_encode(&this->buf_serializer, fields, src_struct);
	if (!success)
		return false;

	int total_size = sizeof(Datum_Info) + this->buf_serializer.bytes_written;
	if ((total_size + this->cur_frame_len) > sizeof(Frame::data))
		return false; // Over message size limit

	Datum_Info info;
	info.length = this->buf_serializer.bytes_written;
	info.type = (uint8_t)type;

	// Write datum info
	memcpy(&this->frame.data[this->cur_frame_len], &info, sizeof(info));
	this->cur_frame_len += sizeof(info);

	memcpy(&this->frame.data[this->cur_frame_len], this->tmp_buf, this->buf_serializer.bytes_written);
	this->cur_frame_len += this->buf_serializer.bytes_written;

	++n_datum_encoded;
	return true;
}

bool FrameManager::load_frame(uint8_t* data, int length) {
	reset();
	// Invalid frame or too long
	if (length < sizeof(Frame::id) || length > sizeof(Frame))
		return false;

	this->cur_frame_len = length;

	memcpy(&this->frame, data, length);
	return true;
}

bool FrameManager::decode_frame(datum_decoded_callback callback) {
	int datum_i = 0;
	int data_ptr = 0;

	// Seek datum info
	while ((data_ptr + sizeof(Datum_Info)) < this->cur_frame_len) {
		Datum_Info* info = (Datum_Info*)&this->frame.data[data_ptr];
		data_ptr += sizeof(Datum_Info);
		if (info->length + data_ptr > this->cur_frame_len)
			return false; // Frame is malformed, not enough data for the datum's body
		callback(datum_i, (MessageTypeID)info->type, info->length, &this->frame.data[data_ptr]);
		data_ptr += info->length;
		++datum_i;
	}

	return true;
}