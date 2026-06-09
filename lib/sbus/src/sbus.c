#include "sbus.h"
#include <stddef.h>

static const uint8_t SBUS_HEADER = 0x0F;
static const uint8_t SBUS_FOOTER = 0x00;
static const uint8_t SBUS_FOOTER2 = 0x04;

bool sbus_decode_frame(const uint8_t *frame, uint16_t channels[SBUS_CHANNEL_COUNT], uint8_t *flags) {
    if (frame == NULL || channels == NULL) {
        return false;
    }
    if (frame[0] != SBUS_HEADER) {
        return false;
    }

    channels[0]  = (frame[1]    | frame[2] << 8)                         & 0x07FF;
    channels[1]  = (frame[2] >> 3 | frame[3] << 5)                         & 0x07FF;
    channels[2]  = (frame[3] >> 6 | frame[4] << 2 | frame[5] << 10)       & 0x07FF;
    channels[3]  = (frame[5] >> 1 | frame[6] << 7)                         & 0x07FF;
    channels[4]  = (frame[6] >> 4 | frame[7] << 4)                         & 0x07FF;
    channels[5]  = (frame[7] >> 7 | frame[8] << 1 | frame[9] << 9)         & 0x07FF;
    channels[6]  = (frame[9] >> 2 | frame[10] << 6)                        & 0x07FF;
    channels[7]  = (frame[10] >> 5 | frame[11] << 3)                       & 0x07FF;
    channels[8]  = (frame[12]   | frame[13] << 8)                          & 0x07FF;
    channels[9]  = (frame[13] >> 3 | frame[14] << 5)                       & 0x07FF;
    channels[10] = (frame[14] >> 6 | frame[15] << 2 | frame[16] << 10)      & 0x07FF;
    channels[11] = (frame[16] >> 1 | frame[17] << 7)                       & 0x07FF;
    channels[12] = (frame[17] >> 4 | frame[18] << 4)                       & 0x07FF;
    channels[13] = (frame[18] >> 7 | frame[19] << 1 | frame[20] << 9)      & 0x07FF;
    channels[14] = (frame[20] >> 2 | frame[21] << 6)                       & 0x07FF;
    channels[15] = (frame[21] >> 5 | frame[22] << 3)                       & 0x07FF;

    if (flags != NULL) {
        *flags = frame[23];
    }

    return true;
}

bool sbus_parse_frame(const uint8_t *frame, uint16_t channels[SBUS_CHANNEL_COUNT]) {
    return sbus_decode_frame(frame, channels, NULL);
}

void sbus_parser_init(sbus_parser_t *parser) {
    if (parser == NULL) {
        return;
    }
    parser->state = 0;
    parser->prev_byte = 0;
}

bool sbus_parser_consume(sbus_parser_t *parser, uint8_t byte, uint16_t channels[SBUS_CHANNEL_COUNT], uint8_t *flags) {
    if (parser == NULL || channels == NULL) {
        return false;
    }

    if (parser->state == 0) {
        if (byte == SBUS_HEADER) {
            parser->buf[0] = byte;
            parser->state = 1;
        }
    } else if (parser->state < SBUS_FRAME_SIZE - 1) {
        if (byte == SBUS_HEADER) {
            parser->buf[0] = byte;
            parser->state = 1;
        } else {
            parser->buf[parser->state++] = byte;
        }
    } else {
        parser->buf[parser->state] = byte;
        parser->state = 0;
        parser->prev_byte = byte;

        if (byte == SBUS_FOOTER || (byte & 0x0F) == SBUS_FOOTER2) {
            return sbus_decode_frame(parser->buf, channels, flags);
        }
    }

    parser->prev_byte = byte;
    return false;
}
