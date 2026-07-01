/**
 * @file control.c
 * @brief Flight Control Output Implementation for ESP32-C3 Flight Controller
 *
 * Refactored to match include/control.h / include/config.h.
 * - SG90/servo: outputs via LEDC PWM (50Hz)
 * - DSHOT300/600: SAFE STUB (does not generate DSHOT waveform). Throttle is cut.
 */

#include "control.h"
#include "config.h"
#include "xm_plus.h"

#include <string.h>

#include "driver/ledc.h"
#include "driver/gpio.h"
#include "esp_log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"


static const char *TAG = "control";

// Static state for control system
static ctrl_value_outputs_t s_value_output = {0};

static SemaphoreHandle_t    s_control_mutex = NULL;
static TaskHandle_t         s_ctrl_task_handle = NULL;
static bool s_armed = false;
static bool s_failsafe_active = false;

static bool s_ctrl_initialized = false;

// Queue for decoded SBUS data (from xm_plus_task/main)
static QueueHandle_t s_sbus_queue = NULL;

// Called by main.c to provide the decoded SBUS queue.
void control_set_sbus_queue(QueueHandle_t q);


static void set_pwm_duty(ledc_channel_t ch, uint32_t duty)
{
    ledc_set_duty(LEDC_LOW_SPEED_MODE, ch, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, ch);
}

static uint32_t clamp_u32(uint32_t v, uint32_t lo, uint32_t hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static uint16_t clamp_u16(uint16_t v, uint16_t lo, uint16_t hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

// SBUS (172..1811) -> 0..1000
static uint16_t sbus_to_0_1000(uint16_t sbus_value)
{
    const uint16_t sbus_min = SBUS_VALUE_MIN;
    const uint16_t sbus_max = SBUS_VALUE_MAX;
    sbus_value = clamp_u16(sbus_value, sbus_min, sbus_max);
    return (uint16_t)(((uint32_t)(sbus_value - sbus_min) * 1000U) / (uint32_t)(sbus_max - sbus_min));
}

// Map servo pulse us [SG90_PWM_MIN_US..SG90_PWM_MAX_US] from control units [0..1000] where center=500
// We interpret:
static uint16_t ctrl_to_servo_us(uint16_t ctrl_0_1000)
{
    // ctrl_0_1000 clamped to 0..1000
    uint16_t c = clamp_u16(ctrl_0_1000, 0, 1000);
    return (uint16_t)(((uint32_t)c * (SG90_PWM_MAX_US - SG90_PWM_MIN_US)) / 1000U + SG90_PWM_MIN_US);
}

static uint32_t servo_us_to_duty(uint16_t us)
{
    // LEDC timer is configured for SG90_PWM_FREQ_HZ and PWM_RESOLUTION_BITS.
    const uint32_t period_us = 1000000U / SG90_PWM_FREQ_HZ;
    const uint32_t duty = ((uint32_t)us * ((1U << SG90_PWM_RESOLUTION_BITS) - 1U)) / period_us;
    const uint32_t max_duty = (1U << SG90_PWM_RESOLUTION_BITS) - 1U;
    return clamp_u32(duty, 0, max_duty);
}

static void apply_outputs_locked(void)
{
    // If disarmed OR failsafe OR output type is DSHOT => cut throttle.
    if (s_value_output.dig_ch1_state >= 1000) {
        s_armed = true;
    } else {
        s_armed = false;
    }

    bool cut_throttle = (!s_armed) || s_failsafe_active;

    uint16_t throttle_ctrl = cut_throttle ? 0 : clamp_u16(s_value_output.pwm_ch1_us, 0, 1000);
    uint16_t roll_ctrl_u = clamp_u16(s_value_output.pwm_ch2_us, 0, 1000);
    uint16_t pitch_ctrl_u = clamp_u16(s_value_output.pwm_ch3_us, 0, 1000);
    uint16_t yaw_ctrl_u = clamp_u16(s_value_output.pwm_ch4_us, 0, 1000);

    // Convert to servo pulse widths
    uint16_t throttle_us = ctrl_to_servo_us(throttle_ctrl);
    uint16_t roll_us = ctrl_to_servo_us(roll_ctrl_u);
    uint16_t pitch_us = ctrl_to_servo_us(pitch_ctrl_u);
    uint16_t yaw_us = ctrl_to_servo_us(yaw_ctrl_u);
    uint16_t aux1_us = ctrl_to_servo_us(clamp_u16(s_value_output.pwm_ch5_us, 0, 1000));
    uint16_t aux2_us = ctrl_to_servo_us(clamp_u16(s_value_output.pwm_ch6_us, 0, 1000));

    // Set LEDC duties
    set_pwm_duty((ledc_channel_t)TIM_CH_1, servo_us_to_duty(throttle_us));
    set_pwm_duty((ledc_channel_t)TIM_CH_2, servo_us_to_duty(roll_us));
    set_pwm_duty((ledc_channel_t)TIM_CH_3, servo_us_to_duty(pitch_us));
    set_pwm_duty((ledc_channel_t)TIM_CH_4, servo_us_to_duty(yaw_us));

    // AUX PWM channels (optional)
    set_pwm_duty((ledc_channel_t)TIM_CH_5, servo_us_to_duty(aux1_us));
    set_pwm_duty((ledc_channel_t)TIM_CH_6, servo_us_to_duty(aux2_us));

    // Digital outputs (LEDs): dig_ch1_state/dig_ch2_state -> PIN_OUTPUT_1/2
    gpio_set_level(PIN_OUTPUT_1, s_value_output.dig_ch1_state ? 1 : 0);
    gpio_set_level(PIN_OUTPUT_2, s_value_output.dig_ch2_state ? 1 : 0);
}

void control_tmp(void)
{
    // ESP_LOGI(TAG, "Control task started");
    xm_plus_data_t xm_data;

        if (s_sbus_queue != NULL) {
            // Receive SBUS frames in order (queue length > 1).
            if (xQueueReceive(s_sbus_queue, &xm_data, pdMS_TO_TICKS(50)) == pdTRUE) {

                if (xm_data.data_valid) {

                    control_update_from_sbus(xm_data.channels);
                } else {
                    // Invalid or failsafe
                    control_failsafe();
                }
            }
        } else {
            // Fallback: apply current outputs periodically (old behavior)
            if (s_control_mutex != NULL && xSemaphoreTake(s_control_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                apply_outputs_locked();
                xSemaphoreGive(s_control_mutex);
            }
    }

}

// static void apply_outputs(void)
// {
//     if (s_control_mutex == NULL) return;
//     if (xSemaphoreTake(s_control_mutex, pdMS_TO_TICKS(100)) != pdTRUE) return;
//     apply_outputs_locked();
//     xSemaphoreGive(s_control_mutex);
// }

// bool control_set_output_type(cfg_output_type_t type)
// {
    // if (type == OUTPUT_TYPE_NONE) return false;

    // if (s_control_mutex != NULL && xSemaphoreTake(s_control_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
    //     s_output_type = type;
    //     xSemaphoreGive(s_control_mutex);
    //     return true;
    // }

    // // If not initialized yet, just set.
    // s_output_type = type;
    // return true;
// }

bool control_init(void)
{
    if (s_ctrl_initialized) {
        ESP_LOGW(TAG, "Already initialized");
        return true;
    }

    s_control_mutex = xSemaphoreCreateMutex();
    if (s_control_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create mutex");
        return false;
    }

    bool need_ledc_timer = false;
    for (int i = 0; i < CONTROL_MAX_OUTPUTS; i++) {
        if (BOARD_OUTPUT_MAP[i].type == CFG_OUTPUT_TYPE_SERVO) {
            need_ledc_timer = true;
            break;
        }
    }

    if (need_ledc_timer) {
        ledc_timer_config_t timer_conf = {
            .duty_resolution = SG90_PWM_RESOLUTION_BITS,
            .freq_hz = SG90_PWM_FREQ_HZ,
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .timer_num = LEDC_TIMER_0,
            .clk_cfg = LEDC_AUTO_CLK
        };
        if (ledc_timer_config(&timer_conf) != ESP_OK) {
            ESP_LOGE(TAG, "Failed to init LEDC Timer");
            return false;
        }
    }

    
    uint8_t ledc_ch_counter = 0;
    
    for (int i = 0; i < CONTROL_MAX_OUTPUTS; i++) {
        cfg_hw_output_t cfg = BOARD_OUTPUT_MAP[i];
        
        if (cfg.type == CFG_OUTPUT_TYPE_NONE) continue;

        if (cfg.type == CFG_OUTPUT_TYPE_SERVO) {
            ledc_channel_config_t ch_conf = {
                .channel = (ledc_channel_t)ledc_ch_counter++,
                .duty = 0,
                .gpio_num = cfg.gpio_num,
                .speed_mode = LEDC_LOW_SPEED_MODE,
                .hpoint = 0,
                .timer_sel = LEDC_TIMER_0
            };
            if (ledc_channel_config(&ch_conf) != ESP_OK) {
                ESP_LOGE(TAG, "Failed to init LEDC Channel on GPIO %d", cfg.gpio_num);
                return false;
            }
            ESP_LOGI(TAG, "Channel %d: Initialized as SERVO on GPIO %d", i, cfg.gpio_num);

        } 
        else if (cfg.type == CFG_OUTPUT_TYPE_DSHOT300) {
            // Tobe implemented: DSHOT300 initialization (if needed)
            ESP_LOGI(TAG, "Channel %d: Configured as DSHOT300 on GPIO %d (STUB)", i, cfg.gpio_num);
        }
    }

    // Configure digital outputs (LEDs)
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << PIN_OUTPUT_1) | (1ULL << PIN_OUTPUT_2),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    esp_err_t err = gpio_config(&io_conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "gpio_config failed: %d", err);
        return false;
    }

    // Safe initial state
    s_value_output.pwm_ch1_us = 0;     // throttle ctrl -> 0
    s_value_output.pwm_ch2_us = SG90_PWM_CENTER_US;   // roll center
    s_value_output.pwm_ch3_us = SG90_PWM_CENTER_US;   // pitch center
    s_value_output.pwm_ch4_us = SG90_PWM_CENTER_US;   // yaw center
    s_value_output.pwm_ch5_us = 0;
    s_value_output.pwm_ch6_us = 0;
    s_value_output.dig_ch1_state = 0;
    s_value_output.dig_ch2_state = 0;

    s_armed = false;
    s_failsafe_active = false;

    if (xSemaphoreTake(s_control_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        apply_outputs_locked();
        xSemaphoreGive(s_control_mutex);
    }
    
    // BaseType_t result = xTaskCreate(control_task, "control_task", 4096, 
    //                                     NULL, configMAX_PRIORITIES - 1, NULL);
    // if (result != pdPASS) {
    //     ESP_LOGE(TAG, "Failed to create control task");
    //     return false;
    // }

    s_ctrl_initialized = true;
    ESP_LOGI(TAG, "Control system initialized successfully");
    return true;
}

void control_update_from_sbus(const uint16_t channels[16])
{
    if (!s_ctrl_initialized || channels == NULL) return;


    // If we were in failsafe lock, first valid frame should release it.
    if (s_failsafe_active) {
        s_failsafe_active = false;
    }

    // ESP_LOGI(TAG,
    //          "Channels: CH1=%u, CH2=%u, CH3=%u, CH4=%u, CH5=%u, CH6=%u, CH7=%u, CH8=%u, CH9=%u, CH10=%u, CH11=%u, CH12=%u, CH13=%u, CH14=%u, CH15=%u, CH16=%u",
    //          channels[SBUS_CH_1],
    //          channels[SBUS_CH_2],
    //          channels[SBUS_CH_3],
    //          channels[SBUS_CH_4],
    //          channels[SBUS_CH_5],
    //          channels[SBUS_CH_6],
    //          channels[SBUS_CH_7],
    //          channels[SBUS_CH_8],
    //          channels[SBUS_CH_9],
    //          channels[SBUS_CH_10],
    //          channels[SBUS_CH_11],
    //          channels[SBUS_CH_12],
    //          channels[SBUS_CH_13],
    //          channels[SBUS_CH_14],
    //          channels[SBUS_CH_15],
    //          channels[SBUS_CH_16]);
    if (xSemaphoreTake(s_control_mutex, pdMS_TO_TICKS(100)) != pdTRUE) return;

    uint16_t roll_ctrl = sbus_to_0_1000(channels[SBUS_CH_1]);

    uint16_t pitch_ctrl = sbus_to_0_1000(channels[SBUS_CH_2]);
    uint16_t throttle_ctrl = sbus_to_0_1000(channels[SBUS_CH_3]);
    uint16_t yaw_ctrl = sbus_to_0_1000(channels[SBUS_CH_4]);

    s_value_output.pwm_ch1_us = throttle_ctrl; // throttle
    s_value_output.pwm_ch2_us = roll_ctrl;
    s_value_output.pwm_ch3_us = pitch_ctrl;
    s_value_output.pwm_ch4_us = yaw_ctrl;

    // AUX PWM (optional): store AUX CTRL scaled to 0..1000
    s_value_output.pwm_ch5_us = sbus_to_0_1000(channels[SBUS_CH_7]);
    s_value_output.pwm_ch6_us = sbus_to_0_1000(channels[SBUS_CH_8]);

    // AUX -> digital states
    s_value_output.dig_ch1_state = (channels[SBUS_CH_5]);
    s_value_output.dig_ch2_state = (channels[SBUS_CH_6]);
    s_value_output.dig_ch3_state = (channels[SBUS_CH_9]);
    s_value_output.dig_ch4_state = (channels[SBUS_CH_10]);
    s_value_output.dig_ch5_state = (channels[SBUS_CH_11]);
    s_value_output.dig_ch6_state = (channels[SBUS_CH_12]);
    // s_value_output.dig_ch7_state = (channels[SBUS_CH_13]);
    // s_value_output.dig_ch8_state = (channels[SBUS_CH_14]);
    apply_outputs_locked();

    xSemaphoreGive(s_control_mutex);
}

void control_set_outputs(const ctrl_value_outputs_t *outputs)
{
    if (!s_ctrl_initialized || outputs == NULL) return;

    if (xSemaphoreTake(s_control_mutex, pdMS_TO_TICKS(100)) != pdTRUE) return;

    s_value_output = *outputs;

    // Clamp safety
    s_value_output.pwm_ch1_us = clamp_u16(s_value_output.pwm_ch1_us, 0, 1000);
    s_value_output.pwm_ch2_us = clamp_u16(s_value_output.pwm_ch2_us, 0, 1000);
    s_value_output.pwm_ch3_us = clamp_u16(s_value_output.pwm_ch3_us, 0, 1000);
    s_value_output.pwm_ch4_us = clamp_u16(s_value_output.pwm_ch4_us, 0, 1000);
    s_value_output.pwm_ch5_us = clamp_u16(s_value_output.pwm_ch5_us, 0, 1000);
    s_value_output.pwm_ch6_us = clamp_u16(s_value_output.pwm_ch6_us, 0, 1000);
    s_value_output.dig_ch1_state = outputs->dig_ch1_state ? 1 : 0;
    s_value_output.dig_ch2_state = outputs->dig_ch2_state ? 1 : 0;

    // If disarmed or failsafe active, enforce throttle cut in apply stage.

    apply_outputs_locked();

    xSemaphoreGive(s_control_mutex);
}

void control_get_outputs(ctrl_value_outputs_t *outputs)
{
    if (!s_ctrl_initialized || outputs == NULL) return;

    if (xSemaphoreTake(s_control_mutex, pdMS_TO_TICKS(100)) != pdTRUE) return;
    memcpy(outputs, &s_value_output, sizeof(ctrl_value_outputs_t));
    xSemaphoreGive(s_control_mutex);
}

void control_arm(void)
{
    if (!s_ctrl_initialized) return;

    if (xSemaphoreTake(s_control_mutex, pdMS_TO_TICKS(100)) != pdTRUE) return;

    if (!s_armed) {
        s_armed = true;
        s_failsafe_active = false;

        // Keep centers
        s_value_output.pwm_ch2_us = 500;
        s_value_output.pwm_ch3_us = 500;
        s_value_output.pwm_ch4_us = 500;
        s_value_output.pwm_ch1_us = 0;

        apply_outputs_locked();
        ESP_LOGI(TAG, "System ARMED");
    }

    xSemaphoreGive(s_control_mutex);
}

void control_disarm(void)
{
    if (!s_ctrl_initialized) return;

    if (xSemaphoreTake(s_control_mutex, pdMS_TO_TICKS(100)) != pdTRUE) return;

    if (s_armed) {
        s_armed = false;
        // throttle cut in apply stage
        s_value_output.pwm_ch1_us = 0;
        apply_outputs_locked();
        ESP_LOGI(TAG, "System DISARMED");
    }

    xSemaphoreGive(s_control_mutex);
}

bool control_is_armed(void)
{
    if (!s_ctrl_initialized || s_control_mutex == NULL) return false;

    bool armed = false;
    if (xSemaphoreTake(s_control_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        armed = s_armed;
        xSemaphoreGive(s_control_mutex);
    }
    return armed;
}

void control_failsafe(void)
{
    if (!s_ctrl_initialized) return;

    if (xSemaphoreTake(s_control_mutex, pdMS_TO_TICKS(100)) != pdTRUE) return;

    s_failsafe_active = true;
    s_armed = false;

    // Failsafe policy: cut throttle, center attitude, turn LEDs on.
    s_value_output.pwm_ch1_us = 0;
    s_value_output.pwm_ch2_us = 500;
    s_value_output.pwm_ch3_us = 500;
    s_value_output.pwm_ch4_us = 500;
    s_value_output.dig_ch1_state = 1;
    s_value_output.dig_ch2_state = 1;

    apply_outputs_locked();
    // ESP_LOGW(TAG, "FAILSAFE activated");

    xSemaphoreGive(s_control_mutex);
}

void control_set_sbus_queue(QueueHandle_t q)
{
    s_sbus_queue = q;
}

void control_deinit(void)
{
    if (!s_ctrl_initialized) return;

    if (s_control_mutex != NULL) {
        xSemaphoreTake(s_control_mutex, pdMS_TO_TICKS(100));
        xSemaphoreGive(s_control_mutex);
        vSemaphoreDelete(s_control_mutex);
        s_control_mutex = NULL;
    }

    // Reset state
    s_armed = false;
    s_failsafe_active = false;
    memset(&s_value_output, 0, sizeof(s_value_output));
    s_ctrl_initialized = false;

    // Stop PWM outputs: set duty 0
    for (int i = 0; i < 6; i++) {
        set_pwm_duty((ledc_channel_t)i, 0);
    }



    gpio_set_level(PIN_OUTPUT_1, 0);
    gpio_set_level(PIN_OUTPUT_2, 0);

    ESP_LOGI(TAG, "Control system deinitialized");
}

