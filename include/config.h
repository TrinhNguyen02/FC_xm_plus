/**
 * @file config.h
 * @brief Hardware configuration for the ESP32-C3 Flight Controller.
 *
 * This file contains all hardware-related definitions used by the firmware,
 * including:
 *  - UART configuration
 *  - I2C configuration
 *  - GPIO pin assignments
 *  - FreeRTOS task priorities
 *  - Task stack sizes
 *
 * Keeping all hardware configuration in a single location makes the firmware
 * easier to port to different flight controller boards.
 */

#ifndef CONFIG_H
#define CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

/*==============================================================================
 * UART Configuration
 *============================================================================*/

/**
 * @brief Debug console UART baud rate.
 */
#define UART_CONSOLE_BAUD           115200

/**
 * @brief SBUS receiver UART configuration.
 *
 * SBUS uses:
 *  - 100000 baud
 *  - 8 data bits
 *  - Even parity
 *  - 2 stop bits
 *  - Inverted RX signal
 */
#define PIN_XM_PLUS_UART_RX         20
#define XM_PLUS_UART_PORT           UART_NUM_1
#define XM_PLUS_BAUD_RATE           100000
#define XM_PLUS_UART_BUF_SIZE       512


/*==============================================================================
 * I2C Configuration
 *============================================================================*/

/**
 * @brief IMU I2C interface.
 */
#define PIN_IMU_SDA                 8
#define PIN_IMU_SCL                 9


/*==============================================================================
 * PWM Output Pins
 *============================================================================*/

/**
 * @brief PWM output channels.
 *
 * These outputs can be configured for:
 *  - Servo PWM
 *  - ESC PWM
 *  - DSHOT (future)
 */
#define PIN_PWM_1                   4
#define PIN_PWM_2                   5
#define PIN_PWM_3                   6
#define PIN_PWM_4                   7
#define PIN_PWM_5                   10
#define PIN_PWM_6                   21


/*==============================================================================
 * Digital Output Pins
 *============================================================================*/

/**
 * @brief General-purpose digital outputs.
 */
#define PIN_OUTPUT_1                2
#define PIN_OUTPUT_2                3


/*==============================================================================
 * Analog Input Pins
 *============================================================================*/

/**
 * @brief Reserved analog inputs.
 *
 * These pins are currently unused and may be assigned to
 * sensors or ADC functions in future revisions.
 */
#define PIN_INPUT_1                 0
#define PIN_INPUT_2                 1


/*==============================================================================
 * FreeRTOS Task Configuration
 *============================================================================*/

/**
 * @brief Task priority definitions.
 *
 * Higher numeric value means higher scheduling priority.
 *
 * Current scheduling order:
 *
 *   Control Task
 *        ↑
 *   IMU Task
 *        ↑
 *   Algorithm Inner Loop
 *        ↑
 *   SBUS Receiver
 *        ↑
 *   Algorithm Outer Loop
 */
#define PRIO_BASE                   configMAX_PRIORITIES

#define TASK_CONTROL_PRIORITY       (PRIO_BASE - 1)
#define TASK_IMU_PRIORITY           (PRIO_BASE - 2)
#define TASK_ALG_INNER_PRIORITY     (PRIO_BASE - 3)
#define TASK_XM_PLUS_PRIORITY       (PRIO_BASE - 4)
#define TASK_ALG_OUTER_PRIORITY     (PRIO_BASE - 5)


/*==============================================================================
 * FreeRTOS Task Stack Sizes
 *============================================================================*/

/**
 * @brief Task stack allocation (bytes).
 *
 * Increase these values if stack overflow is detected.
 */
#define STACK_CONTROL               4096
#define STACK_IMU                   4096
#define STACK_ALG_INNER             4096
#define STACK_ALG_OUTER             4096
#define STACK_XM_PLUS               4096


#ifdef __cplusplus
}
#endif

#endif /* CONFIG_H */