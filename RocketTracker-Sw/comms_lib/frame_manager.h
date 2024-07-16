#ifndef FRAME_MANAGER_H
#define FRAME_MANAGER_H

#include <stdint.h>
#include <stdlib.h>
#include <memory.h>
#include "protocol.pb.h"
#include <pb_encode.h>
#include <pb_decode.h>
#include "frame_manager_common.h"

typedef void (*datum_decoded_callback)(int i, DatumTypeID type, int length, uint8_t* data);

class FrameManager {
private:
    pb_ostream_t buf_serializer;
    pb_istream_t buf_deserializer;
    // uint8_t tmp_buf[sizeof(Frame::data) - sizeof(Datum_Info)];
    // uint8_t* serialization_buffer; // length = get_max_datum_data_size

    size_t frame_maxsize;
    bool bufs_allocd = false;

public:
    Frame frame; // frame.data points to &this->frame_buf[FRAME_METADATA_SIZE] (start of the serialized frame's data)
    int cur_frame_len;
    int n_datum_encoded;
    uint8_t* frame_buf; // length = frame_maxsize


    FrameManager(uint8_t* buf_frame, size_t frame_maxsize);


    // Initialize the frame manager with a frame buffer
    FrameManager(size_t frame_maxsize);

    // Return the data capacity of the frame buffer
    size_t get_frame_max_datalen();

    size_t get_max_datum_data_size();

    size_t get_buffer_size();

    // Serialize a message and add it to the current frame
    bool encode_datum(DatumTypeID type, const pb_msgdesc_t* fields, const void* src_struct);

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

    // TODO: Implement this, so when switching from USB -> LoRa, all queued frames can be sent!
    // Minimize loss.

    // Successful when -1 is returned, pass 0 for starting_offset, and then pass the previous call's return value until -1 is returned
    // size_t transfer_from_framemanager(FrameManager* other, size_t starting_offset);

    ~FrameManager();
};

bool FrameManager_ser_callback(pb_ostream_t* stream, const pb_byte_t* buf, size_t count);

#endif