#include "fmgr.h"
#include "frame_manager.h"

void fmgr_init(fmgr_t* fmgr, size_t frame_maxsize) {
    fmgr->frame_manager = (void*)(new FrameManager(frame_maxsize));
}
void fmgr_init_with_static_buffers(fmgr_t* fmgr, uint8_t* buffer_1, size_t frame_maxsize) {
    fmgr->frame_manager = (void*)(new FrameManager(buffer_1, frame_maxsize));
}
void fmgr_deinit(fmgr_t* fmgr) {
    delete fmgr;
}

uint8_t* fmgr_get_buffer(fmgr_t* fmgr, int* bufsize) {
    FrameManager* mgr = (FrameManager*)fmgr->frame_manager;
    *bufsize = mgr->get_buffer_size();
    return mgr->frame_buf;
}
uint8_t* fmgr_get_frame(fmgr_t* fmgr, int* len_out) {
    FrameManager* mgr = (FrameManager*)fmgr->frame_manager;
    return mgr->get_frame(len_out);
}
bool fmgr_encode_datum(fmgr_t* fmgr, DatumTypeID type, const pb_msgdesc_t* fields, const void* src_struct) {
    FrameManager* mgr = (FrameManager*)fmgr->frame_manager;
    return mgr->encode_datum(type, fields, src_struct);
}
bool fmgr_load_frame(fmgr_t* fmgr, uint8_t* data, int length) {
    FrameManager* mgr = (FrameManager*)fmgr->frame_manager;
    return mgr->load_frame(data, length);
}
bool fmgr_decode_frame(fmgr_t* fmgr, datum_decoded_callback callback) {
    FrameManager* mgr = (FrameManager*)fmgr->frame_manager;
    return mgr->decode_frame(callback);
}
uint16_t fmgr_calculate_crc(fmgr_t* fmgr) {
    FrameManager* mgr = (FrameManager*)fmgr->frame_manager;
    return mgr->calculate_crc();
}
bool fmgr_check_crc(fmgr_t* fmgr) {
    FrameManager* mgr = (FrameManager*)fmgr->frame_manager;
    return mgr->check_crc();
}
void fmgr_reset(fmgr_t* fmgr) {
    FrameManager* mgr = (FrameManager*)fmgr->frame_manager;
    return mgr->reset();
}
int fmgr_get_n_datum_encoded(fmgr_t* fmgr) {
    FrameManager* mgr = (FrameManager*)fmgr->frame_manager;
    return mgr->n_datum_encoded;
}
int fmgr_get_cur_frame_len(fmgr_t* fmgr) {
    FrameManager* mgr = (FrameManager*)fmgr->frame_manager;
    return mgr->cur_frame_len;
}
void fmgr_set_frame_id(fmgr_t* fmgr, uint16_t id) {
    FrameManager* mgr = (FrameManager*)fmgr->frame_manager;
    mgr->frame.id = id;
}


uint16_t fmgr_get_cur_frame_crc(fmgr_t* fmgr) {
    FrameManager* mgr = (FrameManager*)fmgr->frame_manager;
    return mgr->frame.crc;
}
uint16_t fmgr_get_cur_frame_id(fmgr_t* fmgr) {
    FrameManager* mgr = (FrameManager*)fmgr->frame_manager;
    return mgr->frame.id;
}