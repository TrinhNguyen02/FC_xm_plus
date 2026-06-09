/**
 * @file xm_plus.h
 * @brief XM+ SBUS Receiver Interface for ESP32-C3 Flight Controller
 * 
 * This module handles communication with the FrSky XM+ receiver via SBUS protocol.
 * SBUS uses inverted UART at 100000 baud, 8 data bits, even parity, 2 stop bits.
 * 
 * Features:
 * - Uses UART with hardware inversion support
 * - DMA-based reception for reliable data capture
 * - FreeRTOS task for continuous SBUS frame processing
 * - Thread-safe channel data access via mutex
 */

#ifndef XM_PLUS_H
#define XM_PLUS_H

#include <stdint.h>
#include <stdbool.h>
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief SBUS channel data structure
 * 
 * Contains all 16 channels plus status flags from the SBUS frame.
 * Channel values range from 172 to 1811 (FrSky SBUS standard).
 */
typedef struct {
    uint16_t channels[16];    ///< 16 SBUS channel values (172-1811)
    uint8_t  flags;           ///< SBUS flags (failsafe, frame lost, ch17, ch18)
    bool     data_valid;      ///< True if valid data has been received
    uint32_t last_update;     ///< Tick count of last valid frame
} xm_plus_data_t;

/**
 * @brief Initialize the XM+ SBUS receiver interface
 * 
 * Configures UART1 with inverted RX signal for SBUS communication.
 * Sets up DMA buffer for reliable reception and creates the SBUS
 * processing task.
 * 
 * @return true if initialization successful, false otherwise
 */
bool xm_plus_init(void);

/**
 * @brief Get the latest SBUS channel data (thread-safe)
 * 
 * Copies the most recent SBUS data to the provided structure.
 * This function is thread-safe and can be called from any task.
 * 
 * @param[out] data Pointer to structure to receive channel data
 */
void xm_plus_get_data(xm_plus_data_t *data);

/**
 * @brief Get a single channel value (thread-safe)
 * 
 * @param channel Channel index (0-15)
 * @param[out] value Pointer to receive channel value
 * @return true if channel is valid, false if no data received yet
 */
bool xm_plus_get_channel(uint8_t channel, uint16_t *value);

/**
 * @brief Check if SBUS signal is valid
 * 
 * @return true if valid SBUS frames are being received
 */
bool xm_plus_is_valid(void);

/**
 * @brief Check if receiver is in failsafe mode
 * 
 * @return true if failsafe is active (transmitter off or out of range)
 */
bool xm_plus_is_failsafe(void);

/**
 * @brief Get the SBUS flags byte
 * 
 * @param[out] flags Pointer to receive flags byte
 * @return true if data is valid
 */
bool xm_plus_get_flags(uint8_t *flags);

/**
 * @brief Deinitialize the XM+ interface
 * 
 * Stops the SBUS task and releases UART resources.
 */
void xm_plus_deinit(void);

#ifdef __cplusplus
}
#endif

#endif // XM_PLUS_H