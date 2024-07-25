#ifndef FMGR_H
#define FMGR_H

#ifdef __cplusplus
#define EXTERNC extern "C"
#else
#define EXTERNC
#endif

#include <protocol.pb.h>
#include <stdint.h>

// Redef
typedef void (*datum_decoded_callback)(int i, DatumTypeID type, int length, uint8_t* data);


typedef struct fmgr_t {
    void* frame_manager; // FrameManager*
} fmgr_t;

EXTERNC void fmgr_init(fmgr_t* fmgr, size_t frame_maxsize);
EXTERNC void fmgr_deinit(fmgr_t* fmgr);
EXTERNC uint8_t* fmgr_get_frame(fmgr_t* fmgr, int* len_out);
EXTERNC uint8_t* fmgr_get_buffer(fmgr_t* fmgr, int* bufsize);
EXTERNC bool fmgr_encode_datum(fmgr_t* fmgr, DatumTypeID type, const pb_msgdesc_t* fields, const void* src_struct);
EXTERNC bool fmgr_load_frame(fmgr_t* fmgr, uint8_t* data, int length);
EXTERNC bool fmgr_decode_frame(fmgr_t* fmgr, datum_decoded_callback callback);
EXTERNC uint16_t fmgr_calculate_crc(fmgr_t* fmgr);
EXTERNC bool fmgr_check_crc(fmgr_t* fmgr);
EXTERNC void fmgr_reset(fmgr_t* fmgr);
EXTERNC int fmgr_get_n_datum_encoded(fmgr_t* fmgr);
EXTERNC int fmgr_get_cur_frame_len(fmgr_t* fmgr);
EXTERNC void fmgr_set_frame_id(fmgr_t* fmgr, uint16_t id);

EXTERNC uint16_t fmgr_get_cur_frame_crc(fmgr_t* fmgr);
EXTERNC uint16_t fmgr_get_cur_frame_id(fmgr_t* fmgr);
EXTERNC uint16_t fmgr_util_crc16(uint8_t* buffer, size_t length);

#undef EXTERNC

#endif