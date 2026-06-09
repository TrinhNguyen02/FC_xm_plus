#ifndef SBUS_H
#define SBUS_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SBUS_FRAME_SIZE 25
#define SBUS_CHANNEL_COUNT 16

#define SBUS_CH17_MASK 0x01
#define SBUS_CH18_MASK 0x02
#define SBUS_LOST_FRAME_MASK 0x04
#define SBUS_FAILSAFE_MASK 0x08

typedef struct {
    uint8_t buf[SBUS_FRAME_SIZE];
    uint8_t state;
    uint8_t prev_byte;
} sbus_parser_t;

/// Decode a full SBUS frame into 16 channels and flags.
/// Returns true if the frame header is valid.
bool sbus_decode_frame(const uint8_t *frame, uint16_t channels[SBUS_CHANNEL_COUNT], uint8_t *flags);

/// Decode only channel values from a valid SBUS frame.
bool sbus_parse_frame(const uint8_t *frame, uint16_t channels[SBUS_CHANNEL_COUNT]);

/// Initialize SBUS parser state.
void sbus_parser_init(sbus_parser_t *parser);

/// Consume one byte from the SBUS stream.
/// Returns true when a valid frame is completed.
bool sbus_parser_consume(sbus_parser_t *parser, uint8_t byte, uint16_t channels[SBUS_CHANNEL_COUNT], uint8_t *flags);

/// Return true if SBUS failsafe flag is set.
static inline bool sbus_failsafe(uint8_t flags) {
    return (flags & SBUS_FAILSAFE_MASK) != 0;
}

/// Return true if SBUS frame lost flag is set.
static inline bool sbus_frame_lost(uint8_t flags) {
    return (flags & SBUS_LOST_FRAME_MASK) != 0;
}

/// Return true if SBUS channel 17 is active.
static inline bool sbus_channel17(uint8_t flags) {
    return (flags & SBUS_CH17_MASK) != 0;
}

/// Return true if SBUS channel 18 is active.
static inline bool sbus_channel18(uint8_t flags) {
    return (flags & SBUS_CH18_MASK) != 0;
}

#ifdef __cplusplus
}
#endif

#endif // SBUS_H
