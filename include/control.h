/**
 * @file control.h
 * @brief Flight Control Output Interface for ESP32-C3 Flight Controller
 * 
 * This module handles PWM output control for:
 * - Main motor/throttle (ESC) via PWM
 * - Left servo (aileron/elevator) via PWM
 * - Right servo (aileron/elevator) via PWM
 * - LED outputs controlled by transmitter switches
 * 
 * Features:
 * - Uses ESP32-C3 LEDC peripheral for PWM generation
 * - Thread-safe control surface updates
 * - Failsafe handling (cuts motor, centers servos)
 * - Smooth control surface transitions
 */

#ifndef CONTROL_H
#define CONTROL_H

#include <stdint.h>
#include <stdbool.h>
#include "config.h"
#include "xm_plus.h"


#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Control surface data structure
 * 
 * Contains PWM values in microseconds for all controlled outputs.
 */
typedef struct {
    uint16_t throttle_us;  ///< Throttle PWM value (0-1000 us, center=500)
    uint16_t roll_us;      ///< Roll PWM value (0-1000 us, center=500)
    uint16_t pitch_us;     ///< Pitch PWM value (0-1000 us, center=500)
    uint16_t yaw_us;       ///< Yaw PWM value (0-1000 us, center=500)
    bool led1_state;       ///< LED 1 on/off state
    bool led2_state;       ///< LED 2 on/off state
} control_outputs_t;

/**
 * @brief Initialize control system
 * 
 * Configures LEDC peripheral for PWM generation on motor and servo pins.
 * Configures GPIO pins for LED outputs.
 * Sets all outputs to safe initial state (motor off, servos centered, LEDs off).
 * 
 * @return true if initialization successful
 */
bool control_init(void);

/**
 * @brief Update control outputs from SBUS channel data
 * 
 * Maps SBUS channel values (172-1811) to PWM outputs:
 * - CH1 (Aileron) -> Left/Right servos (differential for aileron control)
 * - CH2 (Elevator) -> Left/Right servos (mixed for elevator control)
 * - CH3 (Throttle) -> Motor ESC
 * - CH4 (Rudder) -> Reserved for future use
 * - CH5 (AUX1) -> LED1 control
 * - CH6 (AUX2) -> LED2 control
 * 
 * @param channels Pointer to array of 16 SBUS channel values
 */
// Legacy direct update (still present but main loop should use queue-based flow)
void control_update_from_sbus(const uint16_t channels[16]);

/**
 * @brief Set control outputs directly
 * 
 * Allows direct control of all outputs. Useful for autonomous flight modes.
 * 
 * @param outputs Pointer to control outputs structure
 */
void control_set_outputs(const control_outputs_t *outputs);

/**
 * @brief Get current control output values
 * 
 * @param outputs Pointer to structure to receive current output values
 */
void control_get_outputs(control_outputs_t *outputs);

/**
 * @brief Arm the flight controller
 * 
 * Enables motor output. Must be called before motor will respond to throttle.
 */
void control_arm(void);

/**
 * @brief Disarm the flight controller
 * 
 * Disables motor output immediately. Motor will not respond to throttle.
 */
void control_disarm(void);

/**
 * @brief Check if flight controller is armed
 * 
 * @return true if armed, false if disarmed
 */
bool control_is_armed(void);

/**
 * @brief Trigger failsafe behavior
 * 
 * Called when receiver loses signal or enters failsafe mode.
 * - Cuts motor to minimum
 * - Centers all servos
 * - Flashes LEDs as warning
 */
void control_failsafe(void);

/**
 * @brief Deinitialize control system
 * 
 * Stops PWM outputs and releases resources.
 */
void control_deinit(void);

/**
 * @brief Push latest decoded SBUS data into control queue.
 *
 * Safe to call from the RX/main side. Control task will consume
 * and apply outputs.
 *
 * @return true if enqueued successfully
 */


#ifdef __cplusplus
}
#endif

#endif // CONTROL_H