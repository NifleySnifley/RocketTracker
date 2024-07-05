#ifdef __cplusplus
#define EXTERNC extern "C"
#else
#define EXTERNC
#endif

#include <protocol.pb.h>
#include <stdint.h>

// Redef
typedef void (*datum_decoded_callback)(int i, MessageTypeID type, int length, uint8_t* data);


typedef struct fmgr_t {
    void* frame_manager; // FrameManager*
} fmgr_t;

EXTERNC void fmgr_init(fmgr_t* fmgr);
EXTERNC uint8_t* fmgr_get_frame(fmgr_t* fmgr, int* len_out);
EXTERNC bool fmgr_encode_datum(fmgr_t* fmgr, MessageTypeID type, const pb_msgdesc_t* fields, const void* src_struct);
EXTERNC bool fmgr_load_frame(fmgr_t* fmgr, uint8_t* data, int length);
EXTERNC bool fmgr_decode_frame(fmgr_t* fmgr, datum_decoded_callback callback);
EXTERNC uint16_t fmgr_calculate_crc(fmgr_t* fmgr);
EXTERNC bool fmgr_check_crc(fmgr_t* fmgr);
EXTERNC void fmgr_reset(fmgr_t* fmgr);
EXTERNC int fmgr_get_n_datum_encoded(fmgr_t* fmgr);
EXTERNC int fmgr_get_cur_frame_len(fmgr_t* fmgr);
EXTERNC void fmgr_set_frame_id(fmgr_t* fmgr, uint16_t id);

#undef EXTERNC