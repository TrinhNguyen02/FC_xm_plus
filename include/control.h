/**
 * @file control.h
 * @brief Flight control output interface.
 *
 * This module manages all actuator outputs of the Flight Controller,
 * including:
 *
 *  - Servo PWM outputs
 *  - ESC outputs
 *  - Digital outputs
 *  - Arming / disarming
 *  - Failsafe handling
 *  - Output mixing
 */

#ifndef CONTROL_H
#define CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

/*==============================================================================
 * Includes
 *============================================================================*/

#include <stdbool.h>
#include <stdint.h>

#include "driver/gpio.h"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#include "config.h"
#include "xm_plus.h"


/*==============================================================================
 * SBUS Channel Mapping
 *============================================================================*/

/**
 * @brief SBUS input channel indices.
 */
#define SBUS_CH_1                 0      /**< Roll      */
#define SBUS_CH_2                 1      /**< Pitch     */
#define SBUS_CH_3                 2      /**< Throttle  */
#define SBUS_CH_4                 3      /**< Yaw       */
#define SBUS_CH_5                 4      /**< AUX1      */
#define SBUS_CH_6                 5      /**< AUX2      */
#define SBUS_CH_7                 6      /**< AUX3      */
#define SBUS_CH_8                 7      /**< AUX4      */
#define SBUS_CH_9                 8      /**< AUX5      */
#define SBUS_CH_10                9      /**< AUX6      */
#define SBUS_CH_11                10     /**< AUX7      */
#define SBUS_CH_12                11     /**< AUX8      */
#define SBUS_CH_13                12     /**< AUX9      */
#define SBUS_CH_14                13     /**< AUX10     */
#define SBUS_CH_15                14     /**< AUX11     */
#define SBUS_CH_16                15     /**< AUX12     */

#define CTRL_VALUE_MIN            172
#define CTRL_VALUE_MAX            1811


/*==============================================================================
 * Servo PWM Configuration
 *============================================================================*/

#define SG90_PWM_FREQ_HZ          50
#define SG90_PWM_MIN_US           500
#define SG90_PWM_MAX_US           2500
#define SG90_PWM_CENTER_US        1500
#define SG90_PWM_RESOLUTION_BITS  12


/*==============================================================================
 * ESC general PWM Configuration
 *============================================================================*/

#define ESC_PWM_FREQ_HZ           50
#define ESC_PWM_MIN_US            1000
#define ESC_PWM_MAX_US            2000
#define ESC_PWM_CENTER_US         0
#define ESC_PWM_RESOLUTION_BITS  12


/*==============================================================================
 * DSHOT Configuration
 *============================================================================*/

#define DSHOT_PWM_FREQ_HZ         600
#define DSHOT_PWM_MIN_US          125
#define DSHOT_PWM_MAX_US          250


/*==============================================================================
 * Output Configuration
 *============================================================================*/

#define CONTROL_MAX_OUTPUTS       8
#define CONTROL_QUEUE_LENGTH      1


/*==============================================================================
 * Timer Channel Mapping
 *============================================================================*/

#define TIM_CH_1                  0
#define TIM_CH_2                  1
#define TIM_CH_3                  2
#define TIM_CH_4                  3
#define TIM_CH_5                  4
#define TIM_CH_6                  5


/*==============================================================================
 * Type Definitions
 *============================================================================*/

/**
 * @brief Supported output protocols.
 */
typedef enum
{
    CFG_OUTPUT_TYPE_NONE = 0,
    CFG_OUTPUT_TYPE_DIGITAL,
    CFG_OUTPUT_TYPE_SERVO,
    CFG_OUTPUT_TYPE_ESC_PWM,
    CFG_OUTPUT_TYPE_DSHOT300,
    CFG_OUTPUT_TYPE_DSHOT600

} cfg_output_type_t;


/**
 * @brief PWM configuration.
 */
typedef struct
{
    uint32_t frequency_hz;
    uint32_t min_value;
    uint32_t max_value;
    uint32_t center_value;
    uint32_t resolution_bits;

} cfg_pwm_output_t;


/**
 * @brief Hardware output configuration.
 */
typedef struct
{
    uint8_t timer_channel;

    gpio_num_t gpio_num;

    cfg_output_type_t type;

    cfg_pwm_output_t pwm_config;

} cfg_hw_output_t;


/*==============================================================================
 * Default PWM Configurations
 *============================================================================*/

static const cfg_pwm_output_t PWM_CONFIG_SERVO =
{
    .frequency_hz   = SG90_PWM_FREQ_HZ,
    .min_value      = SG90_PWM_MIN_US,
    .max_value      = SG90_PWM_MAX_US,
    .center_value   = SG90_PWM_CENTER_US,
    .resolution_bits = SG90_PWM_RESOLUTION_BITS
};

static const cfg_pwm_output_t PWM_CONFIG_ESC_GEN =
{
    .frequency_hz   = SG90_PWM_FREQ_HZ,
    .min_value      = SG90_PWM_MIN_US,
    .max_value      = SG90_PWM_MAX_US,
    .center_value   = SG90_PWM_CENTER_US,
    .resolution_bits = SG90_PWM_RESOLUTION_BITS
};

/*==============================================================================
 * Board Output Mapping
 *============================================================================*/

/**
 * @brief Default servo PWM configuration.
 */
static const cfg_hw_output_t BOARD_OUTPUT_MAP[CONTROL_MAX_OUTPUTS] =
{
    {
        .timer_channel = TIM_CH_1,
        .gpio_num      = PIN_PWM_1,
        .type          = CFG_OUTPUT_TYPE_ESC_PWM,
        .pwm_config    = PWM_CONFIG_ESC_GEN
    },
    {
        .timer_channel = TIM_CH_2,
        .gpio_num      = PIN_PWM_2,
        .type          = CFG_OUTPUT_TYPE_SERVO,
        .pwm_config    = PWM_CONFIG_SERVO
    },
    {
        .timer_channel = TIM_CH_3,
        .gpio_num      = PIN_PWM_3,
        .type          = CFG_OUTPUT_TYPE_SERVO,
        .pwm_config    = PWM_CONFIG_SERVO
    },
    {
        .timer_channel = TIM_CH_4,
        .gpio_num      = PIN_PWM_4,
        .type          = CFG_OUTPUT_TYPE_SERVO,
        .pwm_config    = PWM_CONFIG_SERVO
    },
    {
        .timer_channel = TIM_CH_5,
        .gpio_num      = PIN_PWM_5,
        .type          = CFG_OUTPUT_TYPE_NONE,
        .pwm_config    = {0}
    },
    {
        .timer_channel = TIM_CH_6,
        .gpio_num      = PIN_PWM_6,
        .type          = CFG_OUTPUT_TYPE_NONE,
        .pwm_config    = {0}
    },
    {
        .timer_channel = 0,
        .gpio_num      = PIN_OUTPUT_1,
        .type          = CFG_OUTPUT_TYPE_DIGITAL,
        .pwm_config    = {0}
    },
    {
        .timer_channel = 0,
        .gpio_num      = PIN_OUTPUT_2,
        .type          = CFG_OUTPUT_TYPE_DIGITAL,
        .pwm_config    = {0}
    }
};


/*==============================================================================
 * Type Definitions
 *============================================================================*/

/**
 * @brief Actuator output values.
 *
 * This structure contains the commands generated by the flight control
 * algorithm before they are sent to the hardware outputs.
 *
 * PWM outputs use pulse width in microseconds.
 * Digital outputs use logical states (0 or 1).
 */
typedef struct
{
    /*---------------- PWM Outputs ----------------*/

    uint16_t pwm_ch1_us;      /**< PWM Output 1 */
    uint16_t pwm_ch2_us;      /**< PWM Output 2 */
    uint16_t pwm_ch3_us;      /**< PWM Output 3 */
    uint16_t pwm_ch4_us;      /**< PWM Output 4 */
    uint16_t pwm_ch5_us;      /**< PWM Output 5 */
    uint16_t pwm_ch6_us;      /**< PWM Output 6 */

    /*-------------- Digital Outputs --------------*/

    uint16_t dig_ch1_state;   /**< Digital Output 1 */
    uint16_t dig_ch2_state;   /**< Digital Output 2 */
    uint16_t dig_ch3_state;   /**< Reserved */
    uint16_t dig_ch4_state;   /**< Reserved */
    uint16_t dig_ch5_state;   /**< Reserved */
    uint16_t dig_ch6_state;   /**< Reserved */
    uint16_t dig_ch7_state;   /**< Reserved */
    uint16_t dig_ch8_state;   /**< Reserved */

} ctrl_value_output_t;


/**
 * @brief Control commands received from the flight algorithm.
 *
 * Values are normalized from receiver inputs before being converted
 * to actuator outputs.
 */
typedef struct
{
    uint16_t throttle;

    uint16_t roll;
    uint16_t pitch;
    uint16_t yaw;

    uint16_t aux1;
    uint16_t aux2;
    uint16_t aux3;
    uint16_t aux4;
    uint16_t aux5;
    uint16_t aux6;
    uint16_t aux7;
    uint16_t aux8;
    uint16_t aux9;
    uint16_t aux10;

} ctrl_value_input_t;

/*==============================================================================
 * Public API
 *============================================================================*/

/*------------------------------------------------------------------------------
 * Initialization
 *----------------------------------------------------------------------------*/

/**
 * @brief Initialize the control output module.
 *
 * This function performs:
 *  - GPIO initialization
 *  - PWM peripheral initialization
 *  - Output configuration
 *  - Creation of the control task
 *
 * @return true if initialization succeeds.
 * @return false otherwise.
 */
bool control_init(void);


/**
 * @brief Deinitialize the control module.
 *
 * All outputs are disabled and allocated resources are released.
 */
void control_deinit(void);


/*------------------------------------------------------------------------------
 * Queue Interface
 *----------------------------------------------------------------------------*/

/**
 * @brief Set the input queue for control commands.
 *
 * The algorithm module sends actuator commands through this queue.
 *
 * @param queue Queue handle.
 */
void control_set_input_queue(QueueHandle_t queue);


/*------------------------------------------------------------------------------
 * Output Control
 *----------------------------------------------------------------------------*/

/**
 * @brief Apply actuator outputs.
 *
 * This function writes the specified output values to the configured
 * PWM and digital output channels.
 *
 * @param output Pointer to the output structure.
 */
void control_update_from_alg(const ctrl_value_input_t *channel);


/**
 * @brief Stop all actuator outputs.
 *
 * Servo outputs return to their neutral position and all digital outputs
 * are cleared.
 */
void control_stop_all_outputs(void);


/*------------------------------------------------------------------------------
 * Output State
 *----------------------------------------------------------------------------*/

/**
 * @brief Enable actuator outputs.
 *
 * After enabling, PWM and digital outputs are allowed to update normally.
 */
void control_enable_output(void);


/**
 * @brief Disable actuator outputs.
 *
 * All outputs remain inactive until enabled again.
 */
void control_disable_output(void);


/**
 * @brief Check whether actuator outputs are enabled.
 *
 * @return true if outputs are enabled.
 * @return false if outputs are disabled.
 */
bool control_is_output_enabled(void);


/*------------------------------------------------------------------------------
 * Arming State
 *----------------------------------------------------------------------------*/

/**
 * @brief Arm the flight controller.
 *
 * When armed, throttle and control surface outputs are allowed
 * to respond to control commands.
 */
void control_arm(void);


/**
 * @brief Disarm the flight controller.
 *
 * Motor outputs are immediately disabled.
 */
void control_disarm(void);


/**
 * @brief Check the current arming state.
 *
 * @return true if the controller is armed.
 * @return false otherwise.
 */
bool control_is_armed(void);


/*------------------------------------------------------------------------------
 * Failsafe
 *----------------------------------------------------------------------------*/

/**
 * @brief Enter failsafe mode.
 *
 * The control module immediately switches all outputs to their
 * predefined failsafe values.
 */
void control_enter_failsafe(void);


/**
 * @brief Exit failsafe mode.
 */
void control_exit_failsafe(void);


/**
 * @brief Check whether failsafe mode is active.
 *
 * @return true if failsafe is active.
 */
bool control_is_failsafe(void);


#ifdef __cplusplus
}
#endif

#endif /* CONTROL_H */