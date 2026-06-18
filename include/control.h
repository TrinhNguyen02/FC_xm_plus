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
#include "driver/gpio.h"
#include "freertos/queue.h"


#ifdef __cplusplus
extern "C" {
#endif

// ============================================================================
// SBUS Channel Mapping (FrSky/FLYSKY standard)
// ============================================================================
#define SBUS_CH_1              0   // Channel 1 (Aileron)
#define SBUS_CH_2              1   // Channel 2 (Elevator)
#define SBUS_CH_3              2   // Channel 3 (Throttle)
#define SBUS_CH_4              3   // Channel 4 (Yaw)
#define SBUS_CH_5              4   // Channel 5 (AUX1)
#define SBUS_CH_6              5   // Channel 6 (AUX2)
#define SBUS_CH_7              6   // Channel 7 (AUX3)
#define SBUS_CH_8              7   // Channel 8 (AUX4)
#define SBUS_CH_9              8   // Channel 9 (AUX5)
#define SBUS_CH_10             9   // Channel 10 (AUX6)
#define SBUS_CH_11             10  // Channel 11 (AUX7)
#define SBUS_CH_12             11  // Channel 12 (AUX8)
#define SBUS_CH_13             12  // Channel 13 (AUX9)
#define SBUS_CH_14             13  // Channel 14 (AUX10)
#define SBUS_CH_15             14  // Channel 15 (AUX11)
#define SBUS_CH_16             15  // Channel 16 (AUX12)

#define SBUS_VALUE_MIN            172
#define SBUS_VALUE_MAX            1811

// ============================================================================
// PWM Output Configuration for SG90 Servos
// ============================================================================
#define SG90_PWM_FREQ_HZ             50     // PWM frequency for ESC and servos
#define SG90_PWM_MIN_US              500   // Minimum pulse width in microseconds
#define SG90_PWM_MAX_US              2500   // Maximum pulse width in microseconds
#define SG90_PWM_CENTER_US           1500   // Center pulse width in microseconds
#define SG90_PWM_RESOLUTION_BITS     12     // PWM resolution in bits (for LED

// ============================================================================
// PWM Output Configuration for general ESC
// ============================================================================
#define ESC_PWM_FREQ_HZ             400    // PWM frequency for ESC (can be 50Hz for analog ESC or 300/600Hz for digital ESC)
#define ESC_PWM_MIN_US              1000   // Minimum pulse width in microseconds 
#define ESC_PWM_MAX_US              2000   // Maximum pulse width in microseconds

// ============================================================================
// PWM Output Configuration for DSHOT ESC (uses digital protocol, so timing is different)
// ============================================================================
#define DSHOT_PWM_FREQ_HZ           600    // Effective frequency for DSHOT signals (not actual PWM frequency)
#define DSHOT_PWM_MIN_US            125    // Minimum pulse width for DSHOT (represents DSHOT command 0)
#define DSHOT_PWM_MAX_US            250    // Maximum pulse width for DSHOT (represents DSHOT command 48)

#define CONTROL_MAX_OUTPUTS           8       // Max number of outputs (4 PWM + 2 digital)


// ============================================================================
// PWM timer channel mapping
// ============================================================================
#define TIM_CH_1 0
#define TIM_CH_2 1
#define TIM_CH_3 2
#define TIM_CH_4 3
#define TIM_CH_5 4
#define TIM_CH_6 5



// ============================================================================
// Configuration output structure
// ============================================================================
/**
 * @brief Output mode configuration
 */
typedef enum {
    CFG_OUTPUT_TYPE_NONE,
    CFG_OUTPUT_TYPE_DIGITAL,
    CFG_OUTPUT_TYPE_SERVO,
    CFG_OUTPUT_TYPE_DSHOT300,
    CFG_OUTPUT_TYPE_DSHOT600
} cfg_output_type_t;

/**
 * @brief Output configuration structure
 */
typedef struct {
    uint32_t frequency_hz;  
    uint32_t min_value;
    uint32_t max_value;
    uint32_t center_value;
    uint32_t resolution_bits;
} cfg_pwm_output_t;

/**
 * @brief Output hardware type configuration
 */
typedef struct {
    uint8_t timer_channel;
    gpio_num_t gpio_num;
    cfg_output_type_t type;
    cfg_pwm_output_t pwm_config;
} cfg_hw_output_t;

static const cfg_pwm_output_t PWM_CONFIG_SERVO = {
    .frequency_hz = SG90_PWM_FREQ_HZ,
    .min_value = SG90_PWM_MIN_US,
    .max_value = SG90_PWM_MAX_US,
    .center_value = SG90_PWM_CENTER_US,
    .resolution_bits = SG90_PWM_RESOLUTION_BITS
};

static const cfg_hw_output_t BOARD_OUTPUT_MAP[CONTROL_MAX_OUTPUTS] = {
    { .timer_channel = TIM_CH_1, .gpio_num = PIN_PWM_1,    .type = CFG_OUTPUT_TYPE_SERVO   , .pwm_config = PWM_CONFIG_SERVO },
    { .timer_channel = TIM_CH_2, .gpio_num = PIN_PWM_2,    .type = CFG_OUTPUT_TYPE_SERVO   , .pwm_config = PWM_CONFIG_SERVO },
    { .timer_channel = TIM_CH_3, .gpio_num = PIN_PWM_3,    .type = CFG_OUTPUT_TYPE_SERVO   , .pwm_config = PWM_CONFIG_SERVO },
    { .timer_channel = TIM_CH_4, .gpio_num = PIN_PWM_4,    .type = CFG_OUTPUT_TYPE_SERVO   , .pwm_config = PWM_CONFIG_SERVO },
    { .timer_channel = TIM_CH_5, .gpio_num = PIN_PWM_5,    .type = CFG_OUTPUT_TYPE_NONE     , .pwm_config = {0} },
    { .timer_channel = TIM_CH_6, .gpio_num = PIN_PWM_6,    .type = CFG_OUTPUT_TYPE_NONE     , .pwm_config = {0} },
    { .timer_channel = 0,        .gpio_num = PIN_OUTPUT_1, .type = CFG_OUTPUT_TYPE_DIGITAL  , .pwm_config = {0} },
    { .timer_channel = 0,        .gpio_num = PIN_OUTPUT_2, .type = CFG_OUTPUT_TYPE_DIGITAL  , .pwm_config = {0} }
};

// ============================================================================
// Control structure definitions
// ============================================================================
/**
 * @brief Control mode configuration
 */
typedef enum {
    CONTROL_MODE_MANUAL,
    CONTROL_MODE_STABILIZE,
    CONTROL_MODE_ACRO,
    CONTROL_MODE_ANGLE,
} FC_control_mode_t;


/**
 * @brief Control surface data structure
 */
typedef struct {
    uint16_t pwm_ch1_us;   // Throttle PWM value (0-1000 us)
    uint16_t pwm_ch2_us;   // Roll PWM value (0-1000 us, center=500)
    uint16_t pwm_ch3_us;   // Pitch PWM value (0-1000 us, center=500)
    uint16_t pwm_ch4_us;   // Yaw PWM value (0-1000 us, center=500)
    uint16_t pwm_ch5_us;   // Auxiliary channel 1 (e.g. LED dimmer)
    uint16_t pwm_ch6_us;   // Auxiliary channel 2 (e.g. LED dimmer)
    uint16_t dig_ch1_state;   // Digital output state for channel 1 (e.g. LED)
    uint16_t dig_ch2_state;   // Digital output state for channel 2 (e.g. LED)
    uint16_t dig_ch3_state;   // Digital output state for channel 3 (future use)
    uint16_t dig_ch4_state;   // Digital output state for channel 4 (future use)
    uint16_t dig_ch5_state;   // Digital output state for channel 5 (future use)
    uint16_t dig_ch6_state;   // Digital output state for channel 6 (future use)
    uint16_t dig_ch7_state;   // Digital output state for channel 7 (future use)
    uint16_t dig_ch8_state;   // Digital output state for channel 8 (future use)
    
} ctrl_value_outputs_t;


/**
 * @brief Set current output type (runtime)
 * 
 * This selects which output mapping/config is used inside control.c.
 * @return true if accepted
 */
bool control_set_output_type(cfg_output_type_t type);

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
void control_set_outputs(const ctrl_value_outputs_t *outputs);

/**
 * @brief Get current control output values
 * 
 * @param outputs Pointer to structure to receive current output values
 */
void control_get_outputs(ctrl_value_outputs_t *outputs);

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

// Provide a FreeRTOS queue handle that carries xm_plus_data_t from xm_plus_task.
// control_task will consume from this queue.
void control_set_sbus_queue(QueueHandle_t q);




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