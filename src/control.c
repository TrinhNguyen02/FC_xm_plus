/**
 * @file control.c
 * @brief Flight Control Output — PWM and GPIO for ESP32-C3 Flight Controller
 *
 * Consumes ctrl_value_input_t from the algorithm queue and drives:
 *   - 4 servo/ESC PWM channels via ESP32-C3 LEDC peripheral
 *   - 2 auxiliary PWM channels
 *   - 2 digital GPIO outputs (LEDs, buzzers, etc.)
 *
 * Arm/disarm logic:
 *   - CH5 (aux1) < 600  → arm switch ready (low position)
 *   - CH5 (aux1) ≥ 900  → arm, only if throttle = 0 and switch was previously low
 *   - Failsafe or disarm → throttle cut immediately
 */

#include "control.h"
#include "config.h"
#include "xm_plus.h"
#include <string.h>
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

/*================ Static Variables =================*/

static const char *TAG = "control";

static ctrl_value_output_t s_value_output = {0};
// static ctrl_value_input_t  s_value_input  = {0};

static SemaphoreHandle_t s_control_mutex  = NULL;
static TaskHandle_t      s_ctrl_task_handle = NULL;
static QueueHandle_t     s_ctrl_queue     = NULL;

static bool s_armed            = false;
static bool s_arm_switch_ready = false;
static bool s_failsafe_active  = false;
static bool s_ctrl_initialized = false;

/*================ Helper Functions =================*/

static void set_pwm_duty(ledc_channel_t ch, uint32_t duty)
{
    ledc_set_duty(LEDC_LOW_SPEED_MODE, ch, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, ch);
}

static uint32_t clamp_u32(uint32_t v, uint32_t lo, uint32_t hi)
{
    if (v < lo) { return lo; }
    if (v > hi) { return hi; }
    return v;
}

static uint16_t clamp_u16(uint16_t v, uint16_t lo, uint16_t hi)
{
    if (v < lo) { return lo; }
    if (v > hi) { return hi; }
    return v;
}

// static uint16_t ctrl_to_0_1000(uint16_t ctrl_value)
// {
//     const uint16_t ctrl_min = CTRL_VALUE_MIN;
//     const uint16_t ctrl_max = CTRL_VALUE_MAX;
//     ctrl_value = clamp_u16(ctrl_value, ctrl_min, ctrl_max);
//     return (uint16_t)(((uint32_t)(ctrl_value - ctrl_min) * 1000U) / (uint32_t)(ctrl_max - ctrl_min));
// }

static uint16_t ctrl_to_servo_us(uint16_t ctrl_0_1000)
{
    uint16_t c = clamp_u16(ctrl_0_1000, 0, 1000);
    return (uint16_t)(((uint32_t)c * (SG90_PWM_MAX_US - SG90_PWM_MIN_US)) / 1000U + SG90_PWM_MIN_US);
}

static uint32_t servo_us_to_duty(uint16_t us)
{
    const uint32_t period_us = 1000000U / SG90_PWM_FREQ_HZ;
    const uint32_t duty      = ((uint32_t)us * ((1U << SG90_PWM_RESOLUTION_BITS) - 1U)) / period_us;
    const uint32_t max_duty  = (1U << SG90_PWM_RESOLUTION_BITS) - 1U;
    return clamp_u32(duty, 0, max_duty);
}

/*================ Arm / Disarm Logic =================*/
/* Must be called with s_control_mutex held */

static void apply_outputs_locked(void)
{
    /* Arm switch low → mark switch as ready; disarm if currently armed */
    if (s_value_output.dig_ch3_state <= 600)
    {
        s_arm_switch_ready = true;

        if (s_armed)
        {
            ESP_LOGI(TAG, "DISARMED");
            s_armed = false;
        }
    }
    else if (s_arm_switch_ready &&
             !s_armed &&
             s_value_output.dig_ch3_state >= 900 &&
             s_value_output.pwm_ch1_us == 0)
    {
        /* Arm: switch high + throttle at zero + switch was previously low */
        ESP_LOGI(TAG, "ARMED");
        s_armed = true;
    }

    bool cut_throttle = (!s_armed) || s_failsafe_active;

    uint16_t throttle_ctrl = cut_throttle ? 0 : s_value_output.pwm_ch1_us;
    uint16_t roll_ctrl_u   = s_value_output.pwm_ch2_us;
    uint16_t pitch_ctrl_u  = s_value_output.pwm_ch3_us;
    uint16_t yaw_ctrl_u    = s_value_output.pwm_ch4_us;

    set_pwm_duty((ledc_channel_t)TIM_CH_1, servo_us_to_duty(ctrl_to_servo_us(throttle_ctrl)));
    set_pwm_duty((ledc_channel_t)TIM_CH_2, servo_us_to_duty(ctrl_to_servo_us(roll_ctrl_u)));
    set_pwm_duty((ledc_channel_t)TIM_CH_3, servo_us_to_duty(ctrl_to_servo_us(pitch_ctrl_u)));
    set_pwm_duty((ledc_channel_t)TIM_CH_4, servo_us_to_duty(ctrl_to_servo_us(yaw_ctrl_u)));

    set_pwm_duty((ledc_channel_t)TIM_CH_5, servo_us_to_duty(ctrl_to_servo_us(clamp_u16(s_value_output.pwm_ch5_us, 0, 1000))));
    set_pwm_duty((ledc_channel_t)TIM_CH_6, servo_us_to_duty(ctrl_to_servo_us(clamp_u16(s_value_output.pwm_ch6_us, 0, 1000))));

    gpio_set_level(PIN_OUTPUT_1, s_value_output.dig_ch1_state ? 1 : 0);
    gpio_set_level(PIN_OUTPUT_2, s_value_output.dig_ch2_state ? 1 : 0);
}

/*================ Control Task =================*/

static void control_task(void *arg)
{
    ESP_LOGI(TAG, "Control task started");

    ctrl_value_input_t ctrl_data;
    uint32_t frames_total    = 0;
    uint32_t last_stats_time = xTaskGetTickCount();
    const TickType_t stats_interval_ticks = pdMS_TO_TICKS(SBUS_STATUS_INTERVAL_MS);

    while (1)
    {
        if (s_ctrl_queue != NULL)
        {
            if (xQueueReceive(s_ctrl_queue, &ctrl_data, pdMS_TO_TICKS(1)) == pdTRUE)
            {
                frames_total++;
                control_update_from_alg(&ctrl_data);
            }
        }
        else
        {
            /* Fallback: apply last known outputs if no queue is connected */
            if (s_control_mutex != NULL && xSemaphoreTake(s_control_mutex, pdMS_TO_TICKS(20)) == pdTRUE)
            {
                apply_outputs_locked();
                xSemaphoreGive(s_control_mutex);
            }
        }

        uint32_t now = xTaskGetTickCount();

        if ((now - last_stats_time) >= stats_interval_ticks)
        {
            uint32_t elapsed_ms = (now - last_stats_time) * portTICK_PERIOD_MS;
            float fps = (elapsed_ms > 0) ? ((float)frames_total * 1000.0f / (float)elapsed_ms) : 0.0f;

            // ESP_LOGI(TAG, "Freq=%.1f fps", fps);

            frames_total     = 0;
            last_stats_time  = now;
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/*================ Initialization / Deinitialization =================*/

bool control_init(void)
{
    ESP_LOGI(TAG, "Initializing control system...");
    if (s_ctrl_initialized)
    {
        return true;
    }

    s_control_mutex = xSemaphoreCreateMutex();

    if (s_control_mutex == NULL)
    {
        return false;
    }

    /* Configure LEDC timer if any output is servo type */
    bool need_ledc_timer = false;

    for (int i = 0; i < CONTROL_MAX_OUTPUTS; i++)
    {
        if (BOARD_OUTPUT_MAP[i].type == CFG_OUTPUT_TYPE_SERVO)
        {
            need_ledc_timer = true;
            break;
        }
    }

    if (need_ledc_timer)
    {
        ledc_timer_config_t timer_conf = {
            .duty_resolution = SG90_PWM_RESOLUTION_BITS,
            .freq_hz         = SG90_PWM_FREQ_HZ,
            .speed_mode      = LEDC_LOW_SPEED_MODE,
            .timer_num       = LEDC_TIMER_0,
            .clk_cfg         = LEDC_AUTO_CLK
        };

        if (ledc_timer_config(&timer_conf) != ESP_OK)
        {
            return false;
        }
    }

    /* Assign LEDC channels to servo outputs */
    uint8_t ledc_ch_counter = 0;

    for (int i = 0; i < CONTROL_MAX_OUTPUTS; i++)
    {
        cfg_hw_output_t cfg = BOARD_OUTPUT_MAP[i];

        if (cfg.type == CFG_OUTPUT_TYPE_SERVO)
        {
            ledc_channel_config_t ch_conf = {
                .channel    = (ledc_channel_t)ledc_ch_counter++,
                .duty       = 0,
                .gpio_num   = cfg.gpio_num,
                .speed_mode = LEDC_LOW_SPEED_MODE,
                .hpoint     = 0,
                .timer_sel  = LEDC_TIMER_0
            };
            esp_err_t err = ledc_channel_config(&ch_conf);

            // ESP_LOGI(TAG, "LEDC ch=%d gpio=%d err=%s",
            //          ch_conf.channel, ch_conf.gpio_num, esp_err_to_name(err));
        }
    }

    /* Configure digital output pins */
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << PIN_OUTPUT_1) | (1ULL << PIN_OUTPUT_2),
        .mode         = GPIO_MODE_OUTPUT
    };
    gpio_config(&io_conf);

    ESP_LOGI(TAG, "Control system initialized successfully.");

    if (xTaskCreate(control_task, "control_task", STACK_CONTROL, NULL,
                    TASK_CONTROL_PRIORITY, &s_ctrl_task_handle) != pdPASS)
    {
        ESP_LOGE(TAG, "Failed to create control task");
        return false;
    }

    s_ctrl_initialized = true;
    return true;
}

void control_deinit(void)
{
    if (s_control_mutex != NULL)
    {
        vSemaphoreDelete(s_control_mutex);
        s_control_mutex = NULL;
    }

    s_ctrl_initialized = false;
}

/*================ Public API =================*/

void control_set_input_queue(QueueHandle_t q)
{
    s_ctrl_queue = q;
}

void control_update_from_alg(const ctrl_value_input_t *channel)
{
    if (!s_ctrl_initialized || channel == NULL)
    {
        return;
    }

    if (s_failsafe_active)
    {
        s_failsafe_active = false;
    }

    if (xSemaphoreTake(s_control_mutex, pdMS_TO_TICKS(100)) == pdTRUE)
    {
        s_value_output.pwm_ch1_us = channel->throttle;
        s_value_output.pwm_ch2_us = channel->roll;
        s_value_output.pwm_ch3_us = channel->pitch;
        s_value_output.pwm_ch4_us = channel->yaw;
        s_value_output.pwm_ch5_us = channel->aux3;
        s_value_output.pwm_ch6_us = channel->aux4;

        s_value_output.dig_ch1_state = channel->aux2;
        s_value_output.dig_ch2_state = channel->aux6;
        s_value_output.dig_ch3_state = channel->aux1;

        apply_outputs_locked();
        xSemaphoreGive(s_control_mutex);
    }
}

void control_arm(void)
{
    if (xSemaphoreTake(s_control_mutex, pdMS_TO_TICKS(100)) == pdTRUE)
    {
        if (!s_armed)
        {
            s_armed           = true;
            s_failsafe_active = false;
            apply_outputs_locked();
        }
        xSemaphoreGive(s_control_mutex);
    }
}

void control_disarm(void)
{
    if (xSemaphoreTake(s_control_mutex, pdMS_TO_TICKS(100)) == pdTRUE)
    {
        s_armed = false;
        apply_outputs_locked();
        xSemaphoreGive(s_control_mutex);
    }
}

void control_failsafe(void)
{
    if (xSemaphoreTake(s_control_mutex, pdMS_TO_TICKS(100)) == pdTRUE)
    {
        s_failsafe_active = true;
        s_armed           = false;
        apply_outputs_locked();
        xSemaphoreGive(s_control_mutex);
    }
}

bool control_is_armed(void)
{
    return s_armed;
}