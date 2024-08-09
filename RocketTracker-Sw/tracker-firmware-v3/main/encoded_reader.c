#include "encoded_reader.h"
#include "defs.h"
#include "esp_log.h"

void encoded_reader_init(encoded_reader_t* reader, uint8_t* buffer, size_t buffer_size) {
    reader->buffer = buffer;
    reader->buffer_len = buffer_size;
    encoded_reader_reset(reader);
}

void encoded_reader_give_byte(encoded_reader_t* reader, uint8_t byte) {
    // Reset FSM on zero byte!
    if (byte == 0) {
        encoded_reader_reset(reader);
        return;
    }

    // On esc, save state and go to escape state
    if (byte == USB_SER_ESC) {
        reader->state_ret = reader->state;
        reader->state = READER_STATE_ESC;
        return;
        // If in esc state, set val unesc to the unescaped!
    } else if (reader->state == READER_STATE_ESC) {
        reader->val_unescaped = (byte == USB_SER_ESC_ESC) ? USB_SER_ESC : 0;
        reader->state = reader->state_ret;
    } else {
        reader->val_unescaped = byte;
    }

    switch (reader->state) {
        // Receive MSB of frame length
        case READER_STATE_READ_LEN_MSB:
            reader->frame_size = reader->val_unescaped;
            reader->state = READER_STATE_READ_LEN_LSB;
            return;
        case READER_STATE_READ_LEN_LSB:
            reader->frame_size |= (uint16_t)reader->val_unescaped << 8;

            reader->state = READER_STATE_READ_DATA;
            reader->frame_idx = 0;
            break;

        case READER_STATE_READ_DATA:
            // ESP_LOGI("LINK_USB", "buffer[%d] = %d", usb_recv_frameidx, val_unesc);
            reader->buffer[reader->frame_idx++] = reader->val_unescaped;
            if (reader->frame_idx == reader->buffer_len) {
                // Abort to prevent overflow
                encoded_reader_reset(reader);
            }

            if (reader->frame_idx >= reader->frame_size) {
                reader->state = READER_STATE_DONE;

                ESP_LOGI("READER", "Frame received, %" PRIu16 " bytes.", reader->frame_size);

                // fmgr_load_frame(&usb_incoming_fmgr, buffer, frame_size);
                // bool ok = fmgr_decode_frame(&usb_incoming_fmgr, link_usb_decode_datum);
                // if (!ok)
                //     ESP_LOGW("READER", "Error decoding frame!");
                // fmgr_reset(&usb_incoming_fmgr);

                // For command/response, will need a set of semaphores to signal that the response has been received so that task can do this
                // link_send_datum(cmd)
                // <<wait for semaphore to signal response>>
                // handle_response(global_variable) (or even better use a queue/semaphore to send a pointer to the data that is "hot" on the buffer)

                // usb_recv_state = 0;
                // usb_recv_frameidx = 0;
            }
            break;
        case READER_STATE_DONE:
            // Nothing
            break;
        default:
            break;
    }
}

void encoded_reader_reset(encoded_reader_t* reader) {
    reader->frame_size = 0;
    reader->state = READER_STATE_READ_LEN_MSB;
    reader->state_ret = READER_STATE_READ_LEN_MSB;
    reader->val_unescaped = 0x00;
}

bool encoded_reader_has_frame(encoded_reader_t* reader) {
    return reader->state == READER_STATE_DONE;
}

uint8_t* encoded_reader_get_frame(encoded_reader_t* reader, size_t* len_out) {
    *len_out = reader->frame_size;
    return reader->buffer;
}

/*
static int usb_recv_frameidx = 0;
    // 0 read len MSB, 4 read len LSB, 1 reading data, 2 esc, 3 done UNUSED
    static int usb_recv_state = 0;
    static int usb_recv_state_ret = 0;
    static uint16_t frame_size;
    static uint8_t val_unesc;

    int buffer_size;
    uint8_t* buffer = fmgr_get_buffer(&usb_incoming_fmgr, &buffer_size);

    while (1) {
        // Blocking read 1 byte
        uint8_t byte;
        int nread = usb_serial_jtag_read_bytes(&byte, 1, pdMS_TO_TICKS(USB_TIMEOUT_MS));
        if (nread != 1) {
            usb_active = false;
            continue;
        } else {
            usb_active = true;
        }

        // Reset FSM on zero byte!
        if (byte == 0) {
            usb_recv_frameidx = 0;
            usb_recv_state = 0;
            continue;
        }

        // On esc, save state and go to escape state
        if (byte == USB_SER_ESC) {
            usb_recv_state_ret = usb_recv_state;
            usb_recv_state = 2;
            // printf("ESC\n");
            continue;
            // If in esc state, set val unesc to the unescaped!
        } else if (usb_recv_state == 2) {
            val_unesc = (byte == USB_SER_ESC_ESC) ? USB_SER_ESC : 0;
            usb_recv_state = usb_recv_state_ret; // Pop state
            // printf("UNESC: %d\n", val_unesc);
        } else {
            // Otherwise the value is just the byte
            val_unesc = byte;
        }

        switch (usb_recv_state) {
            // Receive MSB of frame length
            case 0:
                frame_size = val_unesc;

                usb_recv_state = 4;
                break;
            case 4:
                frame_size |= (uint16_t)val_unesc << 8;

                usb_recv_state = 1;
                usb_recv_frameidx = 0;
                // ESP_LOGI("LINK_USB", "Got frame size: %d", frame_size);
                break;

            case 1:
                // ESP_LOGI("LINK_USB", "buffer[%d] = %d", usb_recv_frameidx, val_unesc);
                buffer[usb_recv_frameidx++] = val_unesc;
                if (usb_recv_frameidx >= frame_size) {
                    usb_recv_state = 3;

                    ESP_LOGI("LINK_USB", "Frame received over usb, %d bytes.", frame_size);
                    fmgr_load_frame(&usb_incoming_fmgr, buffer, frame_size);
                    bool ok = fmgr_decode_frame(&usb_incoming_fmgr, link_usb_decode_datum);
                    if (!ok)
                        ESP_LOGW("LINK_USB", "Error decoding frame!");
                    fmgr_reset(&usb_incoming_fmgr);

                    // For command/response, will need a set of semaphores to signal that the response has been received so that task can do this
                    // link_send_datum(cmd)
                    // <<wait for semaphore to signal response>>
                    // handle_response(global_variable) (or even better use a queue/semaphore to send a pointer to the data that is "hot" on the buffer)

                    usb_recv_state = 0;
                    usb_recv_frameidx = 0;
                }
                break;
        }
    }*/