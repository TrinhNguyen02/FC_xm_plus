/**
 * @file xm_plus.h
 * @brief FrSky XM+ SBUS receiver interface.
 *
 * This module implements the SBUS receiver driver for the ESP32-C3 Flight
 * Controller. It is responsible for:
 *
 *  - UART initialization
 *  - SBUS frame decoding
 *  - Failsafe detection
 *  - Channel value decoding
 *  - Flight mode selection
 *  - Thread-safe data access
 */

#ifndef XM_PLUS_H
#define XM_PLUS_H

#ifdef __cplusplus
extern "C" {
#endif

/*==============================================================================
 * Includes
 *============================================================================*/

#include <stdbool.h>
#include <stdint.h>

#include "driver/uart.h"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"


/*==============================================================================
 * SBUS Configuration
 *============================================================================*/

/**
 * @brief SBUS frame definitions.
 */
#define SBUS_FRAME_SIZE                 25

#define SBUS_HEADER_BYTE                0x0F
#define SBUS_FOOTER_BYTE                0x00
#define SBUS_FOOTER_2_BYTE              0x04

#define SBUS_CHANNEL_COUNT              16

/**
 * @brief Queue configuration.
 */
#define SBUS_QUEUE_LENGTH               1

/**
 * @brief UART receive timeout.
 */
#define SBUS_READ_TIMEOUT_MS            1

/**
 * @brief Signal monitoring configuration.
 */
#define SBUS_STATUS_INTERVAL_MS         2000
#define SBUS_SIGNAL_LOST_TIMEOUT_MS     500

/**
 * @brief Expected SBUS channel range.
 */
#define SBUS_CHANNEL_VALUE_MIN          100
#define SBUS_CHANNEL_VALUE_MAX          2048


/*==============================================================================
 * Enumerations
 *============================================================================*/

/**
 * @brief SBUS channel index.
 */
typedef enum
{
    SBUS_CH_1 = 0,      /**< Aileron */
    SBUS_CH_2,          /**< Elevator */
    SBUS_CH_3,          /**< Throttle */
    SBUS_CH_4,          /**< Rudder */

    SBUS_CH_5,          /**< AUX1 (ARM switch) */
    SBUS_CH_6,          /**< AUX2 */
    SBUS_CH_7,          /**< AUX3 */
    SBUS_CH_8,          /**< AUX4 */

    SBUS_CH_9,          /**< AUX5 */
    SBUS_CH_10,         /**< AUX6 */
    SBUS_CH_11,         /**< AUX7 */
    SBUS_CH_12,         /**< AUX8 */

    SBUS_CH_13,         /**< AUX9 */
    SBUS_CH_14,         /**< AUX10 */
    SBUS_CH_15,         /**< AUX11 */
    SBUS_CH_16          /**< AUX12 */

} sbus_channel_t;


/**
 * @brief SBUS parser state machine.
 */
typedef enum
{
    SBUS_SYNC,
    SBUS_DATA,
    SBUS_DONE

} sbus_state_t;


/**
 * @brief Flight control mode.
 */
typedef enum
{
    CONTROL_MODE_ANGLE = 0,
    CONTROL_MODE_HORIZON,
    CONTROL_MODE_ACRO,
    CONTROL_MODE_RTH,
    CONTROL_MODE_WAYPOINT

} flight_mode_t;


/*==============================================================================
 * Type Definitions
 *============================================================================*/

/**
 * @brief Latest decoded SBUS frame.
 */
typedef struct
{
    /**
     * @brief SBUS channel values.
     */
    uint16_t channels[SBUS_CHANNEL_COUNT];

    /**
     * @brief SBUS status flags.
     */
    uint8_t flags;

    /**
     * @brief Indicates whether valid data has been received.
     */
    bool data_valid;

    /**
     * @brief FreeRTOS tick count of the latest valid frame.
     */
    uint32_t last_update;

} xm_plus_data_t;


/*==============================================================================
 * Public API
 *============================================================================*/

/**
 * @brief Initialize the XM+ receiver.
 *
 * This function configures the UART peripheral, initializes the SBUS
 * decoder and starts the receiver task.
 *
 * @return true if initialization succeeds.
 * @return false otherwise.
 */
bool xm_plus_init(void);


/**
 * @brief Deinitialize the receiver.
 */
void xm_plus_deinit(void);


/**
 * @brief Get the latest decoded SBUS frame.
 *
 * Thread-safe.
 *
 * @param[out] data Destination structure.
 */
void xm_plus_get_data(xm_plus_data_t *data);


/**
 * @brief Get one SBUS channel value.
 *
 * @param channel Channel index.
 * @param[out] value Decoded channel value.
 *
 * @return true if valid data is available.
 */
bool xm_plus_get_channel(uint8_t channel,
                         uint16_t *value);


/**
 * @brief Check whether the receiver signal is valid.
 *
 * @return true if valid SBUS frames are being received.
 */
bool xm_plus_is_valid(void);


/**
 * @brief Check receiver failsafe status.
 *
 * @return true if failsafe is active.
 */
bool xm_plus_is_failsafe(void);


/**
 * @brief Get the SBUS status flags.
 *
 * @param[out] flags SBUS flags.
 *
 * @return true if valid data is available.
 */
bool xm_plus_get_flags(uint8_t *flags);


/**
 * @brief Get the current flight control mode.
 *
 * The mode is decoded from the configured AUX channel.
 *
 * @return Current flight mode.
 */
flight_mode_t xm_plus_get_control_mode(void);


/**
 * @brief Set the output queue for decoded SBUS frames.
 *
 * The XM+ task writes the latest decoded frame into this queue using
 * xQueueOverwrite().
 *
 * @param q Queue handle.
 */
void xm_plus_set_output_queue(QueueHandle_t q);


#ifdef __cplusplus
}
#endif

#endif /* XM_PLUS_H */