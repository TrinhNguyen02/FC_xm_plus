/**
 * @file config.h
 * @brief Pin configuration and hardware definitions for ESP32-C3 Flight Controller
 * 
 * Pin mapping based on connection diagram:
 * - XM+ Receiver (SBUS): GPIO 8 (UART RX)
 * - MPU-6050: SDA=8, SCL=9
 * - PWM Outputs: GPIO 4,5,6,7 for motors/servos; GPIO 10,20 for additional outputs
 * - Digital Outputs: GPIO 2,3 for LEDs 
 */

#ifndef CONFIG_H
#define CONFIG_H

// ============================================================================
// XM+ SBUS Receiver Configuration
// ============================================================================
// SBUS uses inverted UART signal at 100000 baud, 8E2
#define PIN_XM_PLUS_UART_RX     20   // SBUS signal input (inverted UART)
#define XM_PLUS_UART_PORT       UART_NUM_1
#define XM_PLUS_BAUD_RATE       100000
#define XM_PLUS_UART_BUF_SIZE   512

// ============================================================================
// UART Console/Telemetry Configuration
// ============================================================================
#define UART_CONSOLE_BAUD       115200

// ============================================================================
// MPU Module Configuration (I2C) - For future development
// ============================================================================
#define PIN_MPU_SDA             8  // I2C SDA for MPU (optional
#define PIN_MPU_SCL             9  // I2C SCL for MPU (optional)

// ============================================================================
// PWM Output Configuration
// ============================================================================
#define PIN_PWM_1               4
#define PIN_PWM_2               5
#define PIN_PWM_3               6
#define PIN_PWM_4               7
#define PIN_PWM_5               10
#define PIN_PWM_6               21

// ============================================================================
// Digital Output Configuration (LEDs, etc.)
// ============================================================================
#define PIN_OUTPUT_1            2
#define PIN_OUTPUT_2            3

// ============================================================================
// Analog Input Configuration (for future use)
// ============================================================================
#define PIN_INPUT_1             0
#define PIN_INPUT_2             1

// ============================================================================
// FreeRTOS Task Configuration
// ============================================================================
#define TASK_XM_PLUS_PRIORITY   (configMAX_PRIORITIES - 2)
#define TASK_XM_PLUS_STACK      4096
#define TASK_CONTROL_PRIORITY   (configMAX_PRIORITIES - 1)
#define TASK_CONTROL_STACK      3072
#define TASK_MONITOR_PRIORITY   (configMAX_PRIORITIES - 3)
#define TASK_MONITOR_STACK      2048

#endif // CONFIG_H