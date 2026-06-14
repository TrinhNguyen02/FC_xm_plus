/**
 * @file config.h
 * @brief Pin configuration and hardware definitions for ESP32-C3 Flight Controller
 * 
 * Pin mapping based on connection diagram:
 * - XM+ Receiver (SBUS): GPIO 8 (UART RX)
 * - LoRa Module (SPI): MOSI=4, MISO=5, SCK=6, NSS=7, DIO0=9
 * - UART Console: RX=2, TX=3
 * - PWM Outputs: Motor=10, Servo L=1, Servo R=0
 */

#ifndef CONFIG_H
#define CONFIG_H

// ============================================================================
// XM+ SBUS Receiver Configuration
// ============================================================================
// SBUS uses inverted UART signal at 100000 baud, 8E2
#define PIN_XM_PLUS_UART_RX     8   // SBUS signal input (inverted UART)
#define XM_PLUS_UART_PORT       UART_NUM_1
#define XM_PLUS_BAUD_RATE       100000
#define XM_PLUS_UART_BUF_SIZE   512

// ============================================================================
// UART Console/Telemetry Configuration
// ============================================================================
#define PIN_UART_RX             2   // UART RX for console output
#define PIN_UART_TX             3   // UART TX for console output
#define UART_CONSOLE_BAUD       115200

// ============================================================================
// LoRa Module Configuration (SPI) - For future development
// ============================================================================
#define PIN_LORA_MOSI           4   // SPI MOSI
#define PIN_LORA_MISO           5   // SPI MISO
#define PIN_LORA_SCK            6   // SPI Clock
#define PIN_LORA_NSS            7   // SPI Chip Select
#define PIN_LORA_DIO0           9   // DIO0 interrupt pin

// ============================================================================
// PWM Output Configuration
// ============================================================================
#define PIN_PWM_MOTOR           10  // Output currently driven by motor_us (Channel 3)

// Legacy servo pins are disabled for this wiring/mapping update.
// Channel outputs will be mapped as:
//   CH1 (SBUS ch0) -> GPIO1
//   CH2 (SBUS ch1) -> GPIO2
//   CH4 (SBUS ch3) -> GPIO0
#define PIN_PWM_SERVO_L         1
#define PIN_PWM_SERVO_R         0


// PWM settings for ESC and servos
#define PWM_FREQ_HZ             50  // 50Hz for standard ESC/servo
#define PWM_RES_BITS            LEDC_TIMER_14_BIT
#define PWM_DUTY_MAX            ((1 << PWM_RES_BITS) - 1)

// PWM pulse width range (microseconds)
#define PWM_MIN_US              1000
#define PWM_MAX_US              2000
#define PWM_CENTER_US           1500

// ESC specific settings
#define ESC_MIN_US              1000  // Minimum throttle
#define ESC_MAX_US              2000  // Maximum throttle
#define ESC_ARM_US              1000  // Armed minimum

// Servo specific settings
#define SERVO_MIN_US            1000  // Minimum servo angle
#define SERVO_MAX_US            2000  // Maximum servo angle
#define SERVO_CENTER_US         1500  // Center position

// ============================================================================
// LED Output Configuration
// ============================================================================
// Note: GPIO 11 and 12 are used for LEDs controlled by transmitter switches
// These are separate from the built-in LED on GPIO 8 of ESP32-C3
#define PIN_LED1                11  // LED 1 (controlled by AUX1/CH5)
#define PIN_LED2                12  // LED 2 (controlled by AUX2/CH6)

// ============================================================================
// FreeRTOS Task Configuration
// ============================================================================
#define TASK_XM_PLUS_PRIORITY   (configMAX_PRIORITIES - 2)
#define TASK_XM_PLUS_STACK      4096
#define TASK_CONTROL_PRIORITY   (configMAX_PRIORITIES - 3)
#define TASK_CONTROL_STACK      3072
#define TASK_MONITOR_PRIORITY   (configMAX_PRIORITIES - 4)
#define TASK_MONITOR_STACK      3072

// ============================================================================
// SBUS Channel Mapping (FrSky/FLYSKY standard)
// ============================================================================
// Channel indices for SBUS (0-15)
#define SBUS_CH_AILERON         0   // Roll (Channel 1)
#define SBUS_CH_ELEVATOR        1   // Pitch (Channel 2)
#define SBUS_CH_THROTTLE        2   // Throttle (Channel 3)
#define SBUS_CH_RUDDER          3   // Yaw (Channel 4)
#define SBUS_CH_AUX1            4   // AUX1 (Channel 5) - LED1
#define SBUS_CH_AUX2            5   // AUX2 (Channel 6) - LED2
#define SBUS_CH_AUX3            6   // AUX3 (Channel 7)
#define SBUS_CH_AUX4            7   // AUX4 (Channel 8)

// ============================================================================
// Control Mode Definitions
// ============================================================================
typedef enum {
    CONTROL_MODE_MANUAL = 0,
    CONTROL_MODE_STABILIZE,
    CONTROL_MODE_ACRO,
    CONTROL_MODE_ANGLE,
} control_mode_t;

// ============================================================================
// System Flags
// ============================================================================
#define SYSTEM_FLAG_ARMED       (1 << 0)
#define SYSTEM_FLAG_FAILSAFE    (1 << 1)
#define SYSTEM_FLAG_LOST_MODEL  (1 << 2)

#endif // CONFIG_H