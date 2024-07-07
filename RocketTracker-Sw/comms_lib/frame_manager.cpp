#include "frame_manager.h"
#include <stdio.h>

// frame_databuf will have enough room to store all of a serialized frame, including ID, etc

FrameManager::FrameManager(uint8_t* buf_frame, size_t buflens) {
    this->frame_maxsize = buflens;
    this->frame_buf = buf_frame;
    bufs_allocd = true;

    this->frame.data = &this->frame_buf[FRAME_METADATA_SIZE];
    reset();
}

FrameManager::FrameManager(size_t frame_maxsize) {
    this->frame_maxsize = frame_maxsize;
    this->frame_buf = (uint8_t*)malloc(frame_maxsize);
    bufs_allocd = true;

    this->frame.data = &this->frame_buf[FRAME_METADATA_SIZE];
    reset();
}

size_t FrameManager::get_frame_max_datalen() {
    return this->frame_maxsize - FRAME_METADATA_SIZE;
}


size_t FrameManager::get_buffer_size() {
    return this->frame_maxsize;
}

size_t FrameManager::get_max_datum_data_size() {
    return this->frame_maxsize - FRAME_METADATA_SIZE - sizeof(Datum_Info);
}

void FrameManager::reset() {
    this->cur_frame_len = 0;
    this->frame.id = 0;
    memset(this->frame_buf, 0, this->frame_maxsize);
    n_datum_encoded = 0;
}

uint8_t* FrameManager::get_frame(int* len) {
    this->frame.crc = calculate_crc();

    // Move frame metadata to databuf
    *((uint16_t*)&this->frame_buf[0]) = this->frame.id;
    *((uint16_t*)&this->frame_buf[sizeof(Frame::id)]) = this->frame.crc;

    *len = this->cur_frame_len + FRAME_METADATA_SIZE;
    return this->frame_buf;
}

bool FrameManager::encode_datum(DatumTypeID type, const pb_msgdesc_t* fields, const void* src_struct) {
    int datum_start_offset = this->cur_frame_len;

    if ((datum_start_offset + sizeof(Datum_Info)) > this->get_frame_max_datalen()) {
        return false;
    } else {
        this->cur_frame_len += sizeof(Datum_Info);
    }

    // printf("datum_start_offset = %d, cur_frame_len: %d, frame_max_datalen = %d, remaining = %d\n", datum_start_offset, this->cur_frame_len, this->get_frame_max_datalen(), (this->get_frame_max_datalen() - this->cur_frame_len));
    this->buf_serializer = pb_ostream_from_buffer(&this->frame.data[this->cur_frame_len], (this->get_frame_max_datalen() - this->cur_frame_len));
    bool success = pb_encode(&this->buf_serializer, fields, src_struct);
    if (!success) {
        this->cur_frame_len -= sizeof(Datum_Info);
        return false;
    }

    if ((this->buf_serializer.bytes_written + this->cur_frame_len) > this->get_frame_max_datalen()) {
        this->cur_frame_len -= sizeof(Datum_Info);
        return false;
    } // Over message size limit
    else
        this->cur_frame_len += this->buf_serializer.bytes_written;

    Datum_Info info;
    info.length = this->buf_serializer.bytes_written;
    info.type = (uint8_t)type;

    // Write datum info to start of datum written
    memcpy(&this->frame.data[datum_start_offset], &info, sizeof(info));

    ++n_datum_encoded;
    return true;
}

bool FrameManager::load_frame(uint8_t* data, int length) {
    // Invalid frame or too long
    if (length < FRAME_METADATA_SIZE || length > frame_maxsize)
        return false;


    // Copy data into local buffer
    if (this->frame_buf != data) {
        reset();
        memcpy(this->frame_buf, data, length);
    }

    this->cur_frame_len = length - FRAME_METADATA_SIZE;

    // Retreive metadata and store in `frame`
    this->frame.id = *((uint16_t*)&this->frame_buf[0]);
    this->frame.crc = *((uint16_t*)&this->frame_buf[sizeof(Frame::id)]);

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
        callback(datum_i, (DatumTypeID)info->type, info->length, &this->frame.data[data_ptr]);
        data_ptr += info->length;
        ++datum_i;
    }

    return true;
}

uint16_t CRC_TABLE[256] = { 0x0,0x8d95,0x96bf,0x1b2a,0xa0eb,0x2d7e,0x3654,0xbbc1,0xcc43,0x41d6,0x5afc,0xd769,0x6ca8,0xe13d,0xfa17,0x7782,0x1513,0x9886,0x83ac,0xe39,0xb5f8,0x386d,0x2347,0xaed2,0xd950,0x54c5,0x4fef,0xc27a,0x79bb,0xf42e,0xef04,0x6291,0x2a26,0xa7b3,0xbc99,0x310c,0x8acd,0x758,0x1c72,0x91e7,0xe665,0x6bf0,0x70da,0xfd4f,0x468e,0xcb1b,0xd031,0x5da4,0x3f35,0xb2a0,0xa98a,0x241f,0x9fde,0x124b,0x961,0x84f4,0xf376,0x7ee3,0x65c9,0xe85c,0x539d,0xde08,0xc522,0x48b7,0x544c,0xd9d9,0xc2f3,0x4f66,0xf4a7,0x7932,0x6218,0xef8d,0x980f,0x159a,0xeb0,0x8325,0x38e4,0xb571,0xae5b,0x23ce,0x415f,0xccca,0xd7e0,0x5a75,0xe1b4,0x6c21,0x770b,0xfa9e,0x8d1c,0x89,0x1ba3,0x9636,0x2df7,0xa062,0xbb48,0x36dd,0x7e6a,0xf3ff,0xe8d5,0x6540,0xde81,0x5314,0x483e,0xc5ab,0xb229,0x3fbc,0x2496,0xa903,0x12c2,0x9f57,0x847d,0x9e8,0x6b79,0xe6ec,0xfdc6,0x7053,0xcb92,0x4607,0x5d2d,0xd0b8,0xa73a,0x2aaf,0x3185,0xbc10,0x7d1,0x8a44,0x916e,0x1cfb,0xa898,0x250d,0x3e27,0xb3b2,0x873,0x85e6,0x9ecc,0x1359,0x64db,0xe94e,0xf264,0x7ff1,0xc430,0x49a5,0x528f,0xdf1a,0xbd8b,0x301e,0x2b34,0xa6a1,0x1d60,0x90f5,0x8bdf,0x64a,0x71c8,0xfc5d,0xe777,0x6ae2,0xd123,0x5cb6,0x479c,0xca09,0x82be,0xf2b,0x1401,0x9994,0x2255,0xafc0,0xb4ea,0x397f,0x4efd,0xc368,0xd842,0x55d7,0xee16,0x6383,0x78a9,0xf53c,0x97ad,0x1a38,0x112,0x8c87,0x3746,0xbad3,0xa1f9,0x2c6c,0x5bee,0xd67b,0xcd51,0x40c4,0xfb05,0x7690,0x6dba,0xe02f,0xfcd4,0x7141,0x6a6b,0xe7fe,0x5c3f,0xd1aa,0xca80,0x4715,0x3097,0xbd02,0xa628,0x2bbd,0x907c,0x1de9,0x6c3,0x8b56,0xe9c7,0x6452,0x7f78,0xf2ed,0x492c,0xc4b9,0xdf93,0x5206,0x2584,0xa811,0xb33b,0x3eae,0x856f,0x8fa,0x13d0,0x9e45,0xd6f2,0x5b67,0x404d,0xcdd8,0x7619,0xfb8c,0xe0a6,0x6d33,0x1ab1,0x9724,0x8c0e,0x19b,0xba5a,0x37cf,0x2ce5,0xa170,0xc3e1,0x4e74,0x555e,0xd8cb,0x630a,0xee9f,0xf5b5,0x7820,0xfa2,0x8237,0x991d,0x1488,0xaf49,0x22dc,0x39f6,0xb463 };
uint16_t FrameManager::calculate_crc() {
    uint16_t crc = 0;
    for (int i = 0; i < this->cur_frame_len; ++i) {
        uint8_t prev = ((crc & 0xFF00) >> 8) ^ frame.data[i];
        crc = ((crc << 8) ^ CRC_TABLE[prev]) & 0xFFFF;
    }
    return crc;
}

bool FrameManager::check_crc() {
    return frame.crc == calculate_crc();
}

FrameManager::~FrameManager() {
    this->frame.data = NULL;
    if (bufs_allocd) {
        free(this->frame_buf);
    }
}

// size_t transfer_from_framemanager(FrameManager* other, size_t starting_offset) {
// }