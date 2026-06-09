/**
 * @file control.c
 * @brief Flight Control Output Implementation for ESP32-C3 Flight Controller
 * 
 * This module handles PWM output control using the ESP32-C3 LEDC peripheral.
 * Provides thread-safe control surface updates with failsafe handling.
 */

#include "control.h"
#include "config.h"

#include <string.h>
#include <stdio.h>
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

static const char *TAG = "control";

// Static state for control system
static control_outputs_t s_outputs = {0};
static SemaphoreHandle_t s_control_mutex = NULL;
static bool s_armed = false;
static bool s_initialized = false;
static bool s_failsafe_active = false;

// LEDC channel assignments
#define LEDC_CHANNEL_MOTOR   LEDC_CHANNEL_0
#define LEDC_CHANNEL_SERVO_L LEDC_CHANNEL_1
#define LEDC_CHANNEL_SERVO_R LEDC_CHANNEL_2

/**
 * @brief Convert pulse width in microseconds to LEDC duty cycle value
 * 
 * @param us Pulse width in microseconds (1000-2000)
 * @return LEDC duty cycle value
 */
static uint32_t us_to_duty(uint16_t us)
{
    // PWM period in microseconds (50Hz = 20000us)
    const uint32_t period_us = 1000000 / PWM_FREQ_HZ;
    
    // Calculate duty cycle
    uint32_t duty = ((uint32_t)us * PWM_DUTY_MAX) / period_us;
    
    // Clamp to valid range
    if (duty > PWM_DUTY_MAX) {
        duty = PWM_DUTY_MAX;
    }
    
    return duty;
}

/**
 * @brief Map SBUS channel value to pulse width in microseconds
 * 
 * SBUS values range from 172 to 1811 (FrSky standard)
 * Maps to 1000-2000 microseconds PWM pulse width
 * 
 * @param sbus_value SBUS channel value (172-1811)
 * @return Pulse width in microseconds (1000-2000)
 */
static uint16_t sbus_to_us(uint16_t sbus_value)
{
    // SBUS range: 172 (min) to 1811 (max)
    const uint16_t sbus_min = 172;
    const uint16_t sbus_max = 1811;
    
    // Clamp input
    if (sbus_value < sbus_min) sbus_value = sbus_min;
    if (sbus_value > sbus_max) sbus_value = sbus_max;
    
    // Map to 1000-2000 us
    uint16_t us = PWM_MIN_US + (uint32_t)(sbus_value - sbus_min) * 
                  (PWM_MAX_US - PWM_MIN_US) / (sbus_max - sbus_min);
    
    return us;
}

/**
 * @brief Set PWM output for a specific LEDC channel
 * 
 * @param channel LEDC channel number
 * @param us Pulse width in microseconds
 */
static void set_pwm_channel(ledc_channel_t channel, uint16_t us)
{
    uint32_t duty = us_to_duty(us);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, channel, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, channel);
}

/**
 * @brief Apply current control outputs to hardware
 * 
 * Updates all PWM channels and GPIO outputs based on s_outputs.
 * Must be called with mutex held.
 */
static void apply_outputs(void)
{
    // Apply PWM outputs
    if (s_armed) {
        set_pwm_channel(LEDC_CHANNEL_MOTOR, s_outputs.motor_us);
    } else {
        // When disarmed, motor output is minimum (ESC off)
        set_pwm_channel(LEDC_CHANNEL_MOTOR, ESC_MIN_US);
    }
    
    set_pwm_channel(LEDC_CHANNEL_SERVO_L, s_outputs.servo_l_us);
    set_pwm_channel(LEDC_CHANNEL_SERVO_R, s_outputs.servo_r_us);
    
    // Apply LED outputs
    gpio_set_level(PIN_LED1, s_outputs.led1_state ? 1 : 0);
    gpio_set_level(PIN_LED2, s_outputs.led2_state ? 1 : 0);
}

bool control_init(void)
{
    if (s_initialized) {
        ESP_LOGW(TAG, "Already initialized");
        return true;
    }
    
    ESP_LOGI(TAG, "Initializing control system");
    
    // Create mutex for thread-safe access
    s_control_mutex = xSemaphoreCreateMutex();
    if (s_control_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create mutex");
        return false;
    }
    
    // Initialize LEDC timer
    ledc_timer_config_t timer_conf = {
        .duty_resolution = PWM_RES_BITS,
        .freq_hz = PWM_FREQ_HZ,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
        .clk_cfg = LEDC_AUTO_CLK,
        .deconfigure = false,
    };
    
    esp_err_t err = ledc_timer_config(&timer_conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ledc_timer_config failed: %d", err);
        vSemaphoreDelete(s_control_mutex);
        s_control_mutex = NULL;
        return false;
    }
    
    // Configure LEDC channels for motor and servos
    ledc_channel_config_t channel_conf = {
        .channel = 0,
        .duty = 0,
        .gpio_num = PIN_PWM_MOTOR,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .hpoint = 0,
        .timer_sel = LEDC_TIMER_0,
        .flags.output_invert = 0,
    };
    
    // Motor channel
    channel_conf.channel = LEDC_CHANNEL_MOTOR;
    channel_conf.gpio_num = PIN_PWM_MOTOR;
    err = ledc_channel_config(&channel_conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ledc_channel_config (motor) failed: %d", err);
        vSemaphoreDelete(s_control_mutex);
        s_control_mutex = NULL;
        return false;
    }
    
    // Left servo channel
    channel_conf.channel = LEDC_CHANNEL_SERVO_L;
    channel_conf.gpio_num = PIN_PWM_SERVO_L;
    err = ledc_channel_config(&channel_conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ledc_channel_config (servo_l) failed: %d", err);
        vSemaphoreDelete(s_control_mutex);
        s_control_mutex = NULL;
        return false;
    }
    
    // Right servo channel
    channel_conf.channel = LEDC_CHANNEL_SERVO_R;
    channel_conf.gpio_num = PIN_PWM_SERVO_R;
    err = ledc_channel_config(&channel_conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ledc_channel_config (servo_r) failed: %d", err);
        vSemaphoreDelete(s_control_mutex);
        s_control_mutex = NULL;
        return false;
    }
    
    // Configure GPIO for LEDs
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << PIN_LED1) | (1ULL << PIN_LED2),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    
    err = gpio_config(&io_conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "gpio_config failed: %d", err);
        vSemaphoreDelete(s_control_mutex);
        s_control_mutex = NULL;
        return false;
    }
    
    // Initialize outputs to safe state
    s_outputs.motor_us = ESC_MIN_US;
    s_outputs.servo_l_us = SERVO_CENTER_US;
    s_outputs.servo_r_us = SERVO_CENTER_US;
    s_outputs.led1_state = false;
    s_outputs.led2_state = false;
    s_armed = false;
    s_failsafe_active = false;
    
    // Apply safe initial state
    apply_outputs();
    
    s_initialized = true;
    ESP_LOGI(TAG, "Control initialized: motor=%d servoL=%d servoR=%d LED1=%d LED2=%d",
             PIN_PWM_MOTOR, PIN_PWM_SERVO_L, PIN_PWM_SERVO_R, PIN_LED1, PIN_LED2);
    
    return true;
}

void control_update_from_sbus(const uint16_t channels[16])
{
    if (channels == NULL || !s_initialized) {
        return;
    }
    
    if (xSemaphoreTake(s_control_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        // Check for failsafe
        if (s_failsafe_active) {
            xSemaphoreGive(s_control_mutex);
            return;
        }
        
        // Map SBUS channels to control outputs
        
        // CH3 (Throttle) -> Motor
        // Note: SBUS channel 2 is throttle (0-indexed)
        if (s_armed) {
            s_outputs.motor_us = sbus_to_us(channels[SBUS_CH_THROTTLE]);
        } else {
            s_outputs.motor_us = ESC_MIN_US;
        }
        
        // CH1 (Aileron) + CH2 (Elevator) -> Servos
        // Mixing for V-tail or dual aileron configuration
        int16_t aileron = (int16_t)channels[SBUS_CH_AILERON] - 991;  // Center at ~991
        int16_t elevator = (int16_t)channels[SBUS_CH_ELEVATOR] - 991;
        
        // Left servo: aileron + elevator
        int16_t servo_l_raw = 991 + aileron + elevator;
        if (servo_l_raw < 172) servo_l_raw = 172;
        if (servo_l_raw > 1811) servo_l_raw = 1811;
        s_outputs.servo_l_us = sbus_to_us((uint16_t)servo_l_raw);
        
        // Right servo: -aileron + elevator (reversed aileron for differential)
        int16_t servo_r_raw = 991 - aileron + elevator;
        if (servo_r_raw < 172) servo_r_raw = 172;
        if (servo_r_raw > 1811) servo_r_raw = 1811;
        s_outputs.servo_r_us = sbus_to_us((uint16_t)servo_r_raw);
        
        // CH5 (AUX1) -> LED1 (threshold at midpoint ~991)
        s_outputs.led1_state = channels[SBUS_CH_AUX1] > 1500;
        
        // CH6 (AUX2) -> LED2 (threshold at midpoint ~991)
        s_outputs.led2_state = channels[SBUS_CH_AUX2] > 1500;
        
        // Check for arming command (typically throttle stick down-right)
        // This is a simple implementation - can be enhanced
        if (!s_armed && channels[SBUS_CH_THROTTLE] < 200 && 
            channels[SBUS_CH_RUDDER] > 1700) {
            control_arm();
        } else if (s_armed && channels[SBUS_CH_THROTTLE] < 200 && 
                   channels[SBUS_CH_RUDDER] < 300) {
            control_disarm();
        }
        
        apply_outputs();
        xSemaphoreGive(s_control_mutex);
    }
}

void control_set_outputs(const control_outputs_t *outputs)
{
    if (outputs == NULL || !s_initialized) {
        return;
    }
    
    if (xSemaphoreTake(s_control_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        // Copy outputs with safety limits
        if (s_armed) {
            s_outputs.motor_us = outputs->motor_us;
            if (s_outputs.motor_us < ESC_MIN_US) s_outputs.motor_us = ESC_MIN_US;
            if (s_outputs.motor_us > ESC_MAX_US) s_outputs.motor_us = ESC_MAX_US;
        } else {
            s_outputs.motor_us = ESC_MIN_US;
        }
        
        s_outputs.servo_l_us = outputs->servo_l_us;
        s_outputs.servo_r_us = outputs->servo_r_us;
        
        // Clamp servo values
        if (s_outputs.servo_l_us < SERVO_MIN_US) s_outputs.servo_l_us = SERVO_MIN_US;
        if (s_outputs.servo_l_us > SERVO_MAX_US) s_outputs.servo_l_us = SERVO_MAX_US;
        if (s_outputs.servo_r_us < SERVO_MIN_US) s_outputs.servo_r_us = SERVO_MIN_US;
        if (s_outputs.servo_r_us > SERVO_MAX_US) s_outputs.servo_r_us = SERVO_MAX_US;
        
        s_outputs.led1_state = outputs->led1_state;
        s_outputs.led2_state = outputs->led2_state;
        
        apply_outputs();
        xSemaphoreGive(s_control_mutex);
    }
}

void control_get_outputs(control_outputs_t *outputs)
{
    if (outputs == NULL || !s_initialized) {
        return;
    }
    
    if (xSemaphoreTake(s_control_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        memcpy(outputs, &s_outputs, sizeof(control_outputs_t));
        xSemaphoreGive(s_control_mutex);
    }
}

void control_arm(void)
{
    if (!s_initialized) return;
    
    if (xSemaphoreTake(s_control_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        if (!s_armed) {
            s_armed = true;
            s_failsafe_active = false;
            s_outputs.motor_us = ESC_MIN_US;
            apply_outputs();
            ESP_LOGI(TAG, "System ARMED");
        }
        xSemaphoreGive(s_control_mutex);
    }
}

void control_disarm(void)
{
    if (!s_initialized) return;
    
    if (xSemaphoreTake(s_control_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        if (s_armed) {
            s_armed = false;
            s_outputs.motor_us = ESC_MIN_US;
            apply_outputs();
            ESP_LOGI(TAG, "System DISARMED");
        }
        xSemaphoreGive(s_control_mutex);
    }
}

bool control_is_armed(void)
{
    bool armed = false;
    if (s_initialized && s_control_mutex != NULL) {
        if (xSemaphoreTake(s_control_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            armed = s_armed;
            xSemaphoreGive(s_control_mutex);
        }
    }
    return armed;
}

void control_failsafe(void)
{
    if (!s_initialized) return;
    
    if (xSemaphoreTake(s_control_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        s_failsafe_active = true;
        s_armed = false;
        
        // Cut motor
        s_outputs.motor_us = ESC_MIN_US;
        
        // Center servos
        s_outputs.servo_l_us = SERVO_CENTER_US;
        s_outputs.servo_r_us = SERVO_CENTER_US;
        
        // Flash LEDs as warning
        s_outputs.led1_state = true;
        s_outputs.led2_state = true;
        
        apply_outputs();
        ESP_LOGW(TAG, "FAILSAFE activated - motor cut, servos centered");
        
        xSemaphoreGive(s_control_mutex);
    }
}

void control_deinit(void)
{
    if (!s_initialized) {
        return;
    }
    
    // Set all outputs to safe state
    set_pwm_channel(LEDC_CHANNEL_MOTOR, ESC_MIN_US);
    set_pwm_channel(LEDC_CHANNEL_SERVO_L, SERVO_CENTER_US);
    set_pwm_channel(LEDC_CHANNEL_SERVO_R, SERVO_CENTER_US);
    gpio_set_level(PIN_LED1, 0);
    gpio_set_level(PIN_LED2, 0);
    
    // Reset state
    s_armed = false;
    s_failsafe_active = false;
    memset(&s_outputs, 0, sizeof(s_outputs));
    s_initialized = false;
    
    // Delete mutex
    if (s_control_mutex != NULL) {
        vSemaphoreDelete(s_control_mutex);
        s_control_mutex = NULL;
    }
    
    ESP_LOGI(TAG, "Control system deinitialized");
}