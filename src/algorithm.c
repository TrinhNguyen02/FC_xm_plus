/**
 * @file algorithm.c
 * @brief Flight Control Algorithm - Cascaded PID for Fixed-Wing UAV
 *
 * Two-task architecture:
 *
 *  ┌─────────────────────────────────────────────────────────────────┐
 *  │  imu_task (1kHz, in imu.c)                                      │
 *  │    BMI160 → Madgwick → expose Euler + Gyro via atomic API       │
 *  └──────────────┬──────────────────────────┬────────────────────────┘
 *                 │ imu_get_euler_deg()       │ imu_get_gyro_values()
 *                 ▼                           ▼
 *  ┌──────────────────────────┐   ┌──────────────────────────────────┐
 *  │  alg_outer_task (100Hz)  │   │  alg_inner_task (500Hz)          │
 *  │                          │   │                                  │
 *  │  Angle PID:              │   │  Rate PID:                       │
 *  │  euler_err → rate_sp     │──►│  rate_err → servo_out            │
 *  │                          │   │                                  │
 *  │  Read:  SBUS queue       │   │  Read:  s_flight_sp (atomic)     │
 *  │  Write: s_flight_sp      │   │  Write: control queue            │
 *  └──────────────────────────┘   └──────────────────────────────────┘
 *
 * Inter-task communication:
 *   s_flight_sp — 3 float atomics (roll/pitch/yaw rate setpoint in °/s)
 *   On single-core ESP32-C3, _Atomic is sufficient; no true race conditions.
 *   Worst-case: inner reads a rate_sp that is 10ms old (1 outer cycle) — acceptable.
 *
 * CPU estimate (ESP32-C3, single-core, 160MHz):
 *   imu_task   1kHz:  I2C read + Madgwick ≈ 200–400μs/cycle → ~30% CPU
 *   inner_task 500Hz: PID float ≈ 5–10μs/cycle              → < 1% CPU
 *   outer_task 100Hz: PID float + SBUS parse ≈ 20μs/cycle   → < 1% CPU
 *   → Enough headroom for SBUS, control, idle tasks.
 */

#include "algorithm.h"
#include "imu.h"
#include "xm_plus.h"
#include "control.h"
#include "config.h"

#include <math.h>
#include <string.h>
#include <stdatomic.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_log.h"
#include "esp_timer.h"

/*================ Static Variables =================*/

static const char *TAG = "algorithm";

extern flight_mode_t g_fc_control_mode;
static flight_mode_t s_prev_control_mode = CONTROL_MODE_ANGLE;

/* Outer → inner communication */
static flight_setpoint_t    s_flight_sp = {0};
static aux_channel_t s_other_ch  = {0};

/* Queue receiving SBUS frames (outer task reads) */
static QueueHandle_t s_sbus_queue    = NULL;

/* Queue sending servo commands to control.c (inner task writes) */
static QueueHandle_t s_control_queue = NULL;

/*
 * TODO (GPS): Add when GPS module is available
 * static QueueHandle_t s_gps_queue = NULL;
 */

/*================ PID Helpers =================*/

static void pid_reset(fc_pid_t *pid)
{
    pid->integral   = 0.0f;
    pid->prev_error = 0.0f;
}

static float pid_compute(fc_pid_t *pid, float setpoint, float measured, float dt)
{
    if (dt <= 0.0f || dt > 0.5f)
    {
        return 0.0f;
    }

    float error    = setpoint - measured;
    pid->integral += error * dt;

    if (pid->integral >  pid->integral_limit) { pid->integral =  pid->integral_limit; }
    if (pid->integral < -pid->integral_limit) { pid->integral = -pid->integral_limit; }

    float derivative = (error - pid->prev_error) / dt;
    pid->prev_error  = error;

    return (pid->Kp * error) + (pid->Ki * pid->integral) + (pid->Kd * derivative);
}

static float clamp_f(float v, float lo, float hi)
{
    if (v < lo) { return lo; }
    if (v > hi) { return hi; }
    return v;
}

static float sbus_to_norm(uint16_t raw)
{
    return clamp_f(((float)raw - SBUS_MID) / SBUS_HALF_RANGE, -1.0f, 1.0f);
}

static inline uint16_t norm_to_1000(float norm)
{
    return (uint16_t)((norm + 1.0f) * 500.0f + 0.5f);
}

/* Map PID output [-range, +range] → servo value [0, 1000], center = 500 */
static uint16_t pid_to_servo(float output, float range)
{
    float servo = OUTPUT_MID + clamp_f(output / range, -1.0f, 1.0f) * OUTPUT_MID;
    return (uint16_t)servo;
}

/*================ Outer PID State =================*/
/* Only accessed by outer task — no atomic needed */

static fc_pid_t s_roll_outer  = { .Kp=4.0f, .Ki=0.0f, .Kd=0.0f, .integral=0, .prev_error=0, .integral_limit=INTEGRAL_LIMIT };
static fc_pid_t s_pitch_outer = { .Kp=4.0f, .Ki=0.0f, .Kd=0.0f, .integral=0, .prev_error=0, .integral_limit=INTEGRAL_LIMIT };

static void outer_reset_all(void)
{
    pid_reset(&s_roll_outer);
    pid_reset(&s_pitch_outer);
}

/*================ Outer Task Flight Mode Handlers =================*/

static void outer_run_angle(const uint16_t channels[16],
                             float roll_deg, float pitch_deg,
                             float dt)
{
    float roll_norm  = sbus_to_norm(channels[SBUS_CH_1]);
    float pitch_norm = sbus_to_norm(channels[SBUS_CH_2]);
    float yaw_norm   = sbus_to_norm(channels[SBUS_CH_4]);
    float thr_norm   = (sbus_to_norm(channels[SBUS_CH_3]) + 1.0f) / 2.0f;

    float roll_sp_deg  = roll_norm  * MAX_ROLL_ANGLE_DEG;
    float pitch_sp_deg = pitch_norm * MAX_PITCH_ANGLE_DEG;

    float roll_rate  = pid_compute(&s_roll_outer,  roll_sp_deg,  roll_deg,  dt);
    float pitch_rate = pid_compute(&s_pitch_outer, pitch_sp_deg, pitch_deg, dt);

    atomic_store(&s_flight_sp.roll_setpoint,  clamp_f(roll_rate,  -MAX_ROLL_RATE_DPS,  MAX_ROLL_RATE_DPS));
    atomic_store(&s_flight_sp.pitch_setpoint, clamp_f(pitch_rate, -MAX_PITCH_RATE_DPS, MAX_PITCH_RATE_DPS));
    atomic_store(&s_flight_sp.yaw_setpoint,   yaw_norm * MAX_YAW_RATE_DPS);
    atomic_store(&s_flight_sp.throttle,       clamp_f(thr_norm, 0.0f, 1.0f));
}

static void outer_run_horizon(const uint16_t channels[16],
                               float roll_deg, float pitch_deg,
                               float dt)
{
    float roll_norm  = sbus_to_norm(channels[SBUS_CH_1]);
    float pitch_norm = sbus_to_norm(channels[SBUS_CH_2]);
    float yaw_norm   = sbus_to_norm(channels[SBUS_CH_4]);
    float thr_norm   = (sbus_to_norm(channels[SBUS_CH_3]) + 1.0f) / 2.0f;

    /* Roll blend: angle-mode when stick is small, rate-mode when stick is large */
    float abs_roll = fabsf(roll_norm);
    float roll_rate;

    if (abs_roll <= HORIZON_THRESHOLD)
    {
        float roll_sp_deg = roll_norm * MAX_ROLL_ANGLE_DEG;
        roll_rate = pid_compute(&s_roll_outer, roll_sp_deg, roll_deg, dt);
    }
    else
    {
        pid_reset(&s_roll_outer);   /* outer irrelevant at full stick deflection */
        float t = (abs_roll - HORIZON_THRESHOLD) / (1.0f - HORIZON_THRESHOLD);
        roll_rate = roll_norm * MAX_ROLL_RATE_DPS * t;
    }

    /* Pitch blend */
    float abs_pitch = fabsf(pitch_norm);
    float pitch_rate;

    if (abs_pitch <= HORIZON_THRESHOLD)
    {
        float pitch_sp_deg = pitch_norm * MAX_PITCH_ANGLE_DEG;
        pitch_rate = pid_compute(&s_pitch_outer, pitch_sp_deg, pitch_deg, dt);
    }
    else
    {
        pid_reset(&s_pitch_outer);
        float t = (abs_pitch - HORIZON_THRESHOLD) / (1.0f - HORIZON_THRESHOLD);
        pitch_rate = pitch_norm * MAX_PITCH_RATE_DPS * t;
    }

    atomic_store(&s_flight_sp.roll_setpoint,  clamp_f(roll_rate,  -MAX_ROLL_RATE_DPS,  MAX_ROLL_RATE_DPS));
    atomic_store(&s_flight_sp.pitch_setpoint, clamp_f(pitch_rate, -MAX_PITCH_RATE_DPS, MAX_PITCH_RATE_DPS));
    atomic_store(&s_flight_sp.yaw_setpoint,   yaw_norm * MAX_YAW_RATE_DPS);
    atomic_store(&s_flight_sp.throttle,       clamp_f(thr_norm, 0.0f, 1.0f));
}

static void outer_run_acro(const uint16_t channels[16])
{
    /* ACRO: outer PID bypassed, rate setpoint comes directly from stick */
    atomic_store(&s_flight_sp.roll_setpoint,
        sbus_to_norm(channels[SBUS_CH_1]) * MAX_ROLL_RATE_DPS);
    atomic_store(&s_flight_sp.pitch_setpoint,
        sbus_to_norm(channels[SBUS_CH_2]) * MAX_PITCH_RATE_DPS);
    atomic_store(&s_flight_sp.yaw_setpoint,
        sbus_to_norm(channels[SBUS_CH_4]) * MAX_YAW_RATE_DPS);

    float thr = (sbus_to_norm(channels[SBUS_CH_3]) + 1.0f) / 2.0f;
    atomic_store(&s_flight_sp.throttle, clamp_f(thr, 0.0f, 1.0f));
}

static void outer_run_rth(void)
{
    /*
     * TODO (GPS): Compute bearing error from current position to home,
     * feed into navigation PID → rate setpoint.
     *
     * gps_data_t gps;
     * if (xQueuePeek(s_gps_queue, &gps, 0) == pdTRUE) {
     *     float bear_err = calc_bearing_to_home(&gps);
     *     float roll_cmd = pid_compute(&nav_pid, 0.0f, bear_err, dt);
     *     atomic_store(&s_flight_sp.roll_setpoint, clamp_f(roll_cmd, ...));
     * }
     */
    atomic_store(&s_flight_sp.roll_setpoint,  0.0f);
    atomic_store(&s_flight_sp.pitch_setpoint, 0.0f);
    atomic_store(&s_flight_sp.yaw_setpoint,   0.0f);
    atomic_store(&s_flight_sp.throttle,       0.4f);
    ESP_LOGW(TAG, "RTH: GPS not implemented, holding neutral");
}

static void outer_run_waypoint(void)
{
    /* TODO (GPS + Mission): Same as RTH but target is the next waypoint */
    atomic_store(&s_flight_sp.roll_setpoint,  0.0f);
    atomic_store(&s_flight_sp.pitch_setpoint, 0.0f);
    atomic_store(&s_flight_sp.yaw_setpoint,   0.0f);
    atomic_store(&s_flight_sp.throttle,       0.5f);
    ESP_LOGW(TAG, "WAYPOINT: GPS not implemented, holding neutral");
}

static void pass_other_channels(const uint16_t channels[16])
{
    atomic_store(&s_other_ch.aux1,  norm_to_1000(sbus_to_norm(channels[SBUS_CH_5])));
    atomic_store(&s_other_ch.aux2,  norm_to_1000(sbus_to_norm(channels[SBUS_CH_6])));
    atomic_store(&s_other_ch.aux3,  norm_to_1000(sbus_to_norm(channels[SBUS_CH_7])));
    atomic_store(&s_other_ch.aux4,  norm_to_1000(sbus_to_norm(channels[SBUS_CH_8])));
    atomic_store(&s_other_ch.aux5,  norm_to_1000(sbus_to_norm(channels[SBUS_CH_9])));
    atomic_store(&s_other_ch.aux6,  norm_to_1000(sbus_to_norm(channels[SBUS_CH_10])));
    atomic_store(&s_other_ch.aux7,  norm_to_1000(sbus_to_norm(channels[SBUS_CH_11])));
    atomic_store(&s_other_ch.aux8,  norm_to_1000(sbus_to_norm(channels[SBUS_CH_12])));
    atomic_store(&s_other_ch.aux9,  norm_to_1000(sbus_to_norm(channels[SBUS_CH_13])));
    atomic_store(&s_other_ch.aux10, norm_to_1000(sbus_to_norm(channels[SBUS_CH_14])));
}

/*================ OUTER TASK (100Hz) =================*/
/*
 * Responsibility: read SBUS + Madgwick Euler → run Angle PID → write s_flight_sp
 *
 * Flight modes handled here:
 *   ANGLE    → full outer PID,  rate_sp = outer output
 *   HORIZON  → outer PID blended with stick rate when stick is large
 *   ACRO     → outer PID skipped, rate_sp directly from stick
 *   RTH      → rate_sp from GPS navigation (TODO)
 *   WAYPOINT → rate_sp from mission planner (TODO)
 */

void alg_outer_task(void *pvParameters)
{
    (void)pvParameters;
    ESP_LOGI(TAG, "Outer task started @ %u Hz", OUTER_HZ);

    xm_plus_data_t xm_data;
    memset(&xm_data, 0, sizeof(xm_data));

    uint64_t last_us   = esp_timer_get_time();
    // uint32_t log_count = 0;

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(OUTER_PERIOD_MS);

    while (1)
    {
        /* 1. Receive SBUS — non-blocking (timeout=1ms):
         *    if no new frame, reuse previous xm_data (hold setpoint). */
        if (s_sbus_queue != NULL)
        {
            xQueueReceive(s_sbus_queue, &xm_data, pdMS_TO_TICKS(1));
        }

        /*
         * TODO (GPS):
         * gps_data_t gps;
         * if (s_gps_queue != NULL) {
         *     xQueueReceive(s_gps_queue, &gps, 0);
         * }
         */

        /* 2. Compute dt */
        uint64_t now_us = esp_timer_get_time();
        float dt = (float)(now_us - last_us) * 1e-6f;
        last_us  = now_us;

        if (dt <= 0.0f || dt > 0.5f)
        {
            dt = (float)OUTER_PERIOD_MS / 1000.0f;
        }

        /* 3. Update flight mode and detect transitions */
        g_fc_control_mode = xm_plus_get_control_mode();

        if (g_fc_control_mode != s_prev_control_mode)
        {
            outer_reset_all();
            ESP_LOGI(TAG, "Mode: %d -> %d", (int)s_prev_control_mode, (int)g_fc_control_mode);
            s_prev_control_mode = g_fc_control_mode;
        }

        /* 4. Read Euler angles from Madgwick */
        float roll_deg = 0.0f, pitch_deg = 0.0f, yaw_deg = 0.0f;
        imu_get_euler_deg(&roll_deg, &pitch_deg, &yaw_deg);
        /*
         * TODO: imu_get_yaw(&yaw_deg) when magnetometer is available
         */

        /* 5. Run outer PID based on flight mode */
        if (!xm_data.data_valid)
        {
            /* Signal lost: zero all setpoints → level flight */
            atomic_store(&s_flight_sp.roll_setpoint,  0.0f);
            atomic_store(&s_flight_sp.pitch_setpoint, 0.0f);
            atomic_store(&s_flight_sp.yaw_setpoint,   0.0f);
            atomic_store(&s_flight_sp.throttle,       0.0f);
        }
        else
        {
            switch (g_fc_control_mode)
            {
                case CONTROL_MODE_ANGLE:
                    outer_run_angle(xm_data.channels, roll_deg, pitch_deg, dt);
                    break;

                case CONTROL_MODE_HORIZON:
                    outer_run_horizon(xm_data.channels, roll_deg, pitch_deg, dt);
                    break;

                case CONTROL_MODE_ACRO:
                    outer_reset_all();
                    outer_run_acro(xm_data.channels);
                    break;

                case CONTROL_MODE_RTH:
                    outer_reset_all();
                    outer_run_rth();
                    break;

                case CONTROL_MODE_WAYPOINT:
                    outer_reset_all();
                    outer_run_waypoint();
                    break;

                default:
                    atomic_store(&s_flight_sp.roll_setpoint,  0.0f);
                    atomic_store(&s_flight_sp.pitch_setpoint, 0.0f);
                    atomic_store(&s_flight_sp.yaw_setpoint,   0.0f);
                    atomic_store(&s_flight_sp.throttle,       0.0f);
                    break;
            }

            pass_other_channels(xm_data.channels);
        }

        /* 6. Periodic log (disabled by default) */
        // if (++log_count >= OUTER_HZ * 2) {
        //     ESP_LOGI(TAG, "Outer | mode=%d roll=%.1f° pitch=%.1f° | rsp roll=%.1f pitch=%.1f",
        //              (int)g_fc_control_mode, roll_deg, pitch_deg,
        //              atomic_load(&s_flight_sp.roll_setpoint),
        //              atomic_load(&s_flight_sp.pitch_setpoint));
        //     log_count = 0;
        // }

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

/*================ INNER TASK (500Hz) =================*/
/*
 * Responsibility: read s_flight_sp (atomic) + raw gyro → Rate PID → control queue
 *
 * All flight modes converge here; the rate setpoint may come from:
 *   - outer_task        (ANGLE, HORIZON)
 *   - outer_run_acro()  (ACRO — stick direct)
 *   - navigation law    (RTH, WAYPOINT — TODO)
 * Inner task is mode-agnostic: it only needs rate_sp + gyro.
 *
 * Timing: vTaskDelayUntil with xFrequency = 2 ticks (2ms nominal).
 * FreeRTOS jitter ≈ ±50μs at 160MHz — acceptable for 500Hz.
 * For tighter timing, replace with esp_timer_create() callback.
 */

/* Inner PID state — only accessed by inner task, no atomic needed */
static fc_pid_t s_roll_inner  = { .Kp=1.5f, .Ki=0.05f, .Kd=0.02f, .integral=0, .prev_error=0, .integral_limit=INTEGRAL_LIMIT };
static fc_pid_t s_pitch_inner = { .Kp=1.5f, .Ki=0.05f, .Kd=0.02f, .integral=0, .prev_error=0, .integral_limit=INTEGRAL_LIMIT };
static fc_pid_t s_yaw_inner   = { .Kp=1.0f, .Ki=0.02f, .Kd=0.01f, .integral=0, .prev_error=0, .integral_limit=INTEGRAL_LIMIT };

void alg_inner_task(void *pvParameters) 
{
    (void)pvParameters;
    ESP_LOGI(TAG, "Inner task started @ %u Hz", INNER_HZ);

    /*
     * Use esp_timer_get_time() for accurate dt instead of pdMS_TO_TICKS(2),
     * because portTICK_PERIOD_MS = 1ms gives only 1ms resolution.
     * Actual dt oscillates ≈ 1.9–2.1ms — timer gives the correct value.
     */
    uint64_t last_us   = esp_timer_get_time();
    uint32_t log_count = 0;

    TickType_t xLastWakeTime = xTaskGetTickCount();

    /*
     * FreeRTOS tick = 1ms → vTaskDelayUntil(2 ticks) = 2ms nominal.
     * This is the scheduler resolution limit. For accurate 500Hz,
     * use esp_timer_create() with a direct PID callback instead.
     */
    const TickType_t xFrequency = pdMS_TO_TICKS(1000U / INNER_HZ); /* 2 ticks */

    while (1)
    {
        /* 1. Read raw gyro (rad/s from imu.c, converted below) */
        float gx_dps = 0.0f, gy_dps = 0.0f, gz_dps = 0.0f;
        imu_get_gyro_values(&gx_dps, &gy_dps, &gz_dps);
        /*
         * NOTE: imu.c currently outputs gx/gy/gz in rad/s (multiplied by DEG_TO_RAD).
         * algorithm.c expects °/s. Ensure units are consistent when refactoring.
         * Suggested prototype: void imu_get_gyro_dps(float *gx, float *gy, float *gz);
         */

        /* 2. Read rate setpoints from outer task (atomic) */
        float roll_rate_sp  = atomic_load(&s_flight_sp.roll_setpoint);
        float pitch_rate_sp = atomic_load(&s_flight_sp.pitch_setpoint);
        float yaw_rate_sp   = atomic_load(&s_flight_sp.yaw_setpoint);
        float throttle      = atomic_load(&s_flight_sp.throttle);

        /* 3. Compute dt */
        uint64_t now_us = esp_timer_get_time();
        float dt = (float)(now_us - last_us) * 1e-6f;
        last_us  = now_us;

        if (dt <= 0.0f || dt > 0.1f)
        {
            dt = 1.0f / (float)INNER_HZ;
        }

        /* 4. Inner Rate PID: rate error → servo output */
        float roll_out  = pid_compute(&s_roll_inner,  roll_rate_sp,  gx_dps, dt);
        float pitch_out = pid_compute(&s_pitch_inner, pitch_rate_sp, gy_dps, dt);
        float yaw_out   = pid_compute(&s_yaw_inner,   yaw_rate_sp,   gz_dps, dt);

        /* 5. Pack PID output → control input */
        ctrl_value_input_t out = {
            .throttle = (uint16_t)(throttle * OUTPUT_MAX),
            .roll     = pid_to_servo(roll_out,  OUTPUT_MID),
            .pitch    = pid_to_servo(pitch_out, OUTPUT_MID),
            .yaw      = pid_to_servo(yaw_out,   OUTPUT_MID),
            .aux1     = atomic_load(&s_other_ch.aux1),
            .aux2     = atomic_load(&s_other_ch.aux2),
            .aux3     = atomic_load(&s_other_ch.aux3),
            .aux4     = atomic_load(&s_other_ch.aux4),
            .aux5     = atomic_load(&s_other_ch.aux5),
            .aux6     = atomic_load(&s_other_ch.aux6),
            .aux7     = atomic_load(&s_other_ch.aux7),
            .aux8     = atomic_load(&s_other_ch.aux8),
            .aux9     = atomic_load(&s_other_ch.aux9),
            .aux10    = atomic_load(&s_other_ch.aux10)
        };

        /* 6. Send to control task */
        if (s_control_queue != NULL)
        {
            xQueueOverwrite(s_control_queue, &out);
        }

        /* 7. Periodic log (disabled by default) */
        if (++log_count >= INNER_HZ * 2)
        {
            // ESP_LOGI(TAG, "Inner | rsp=%.1f/%.1f/%.1f/%.1f gyr=%.1f/%.1f/%.1f | out=%u/%u/%u/%u",
            //          throttle, roll_rate_sp, pitch_rate_sp, yaw_rate_sp,
            //          gx_dps, gy_dps, gz_dps,
            //          out.throttle, out.roll, out.pitch, out.yaw);
            log_count = 0;
        }

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

/*================ Public API =================*/

bool algorithm_init(void)
{
    ESP_LOGI(TAG, "Initializing inner and outer algorithm PID...");
    if (xTaskCreate(alg_inner_task, "alg_inner",
                    INNER_TASK_STACK, NULL,
                    INNER_TASK_PRIORITY, NULL) != pdPASS)
    {
        ESP_LOGE(TAG, "Failed to create inner task");
        return false;
    }

    if (xTaskCreate(alg_outer_task, "alg_outer",
                    OUTER_TASK_STACK, NULL,
                    OUTER_TASK_PRIORITY, NULL) != pdPASS)
    {
        ESP_LOGE(TAG, "Failed to create outer task");
        return false;
    }

    ESP_LOGI(TAG, "Algorithm initialized: outer=%uHz inner=%uHz", OUTER_HZ, INNER_HZ);
    return true;
}

void algorithm_set_sbus_queue(QueueHandle_t q)
{
    s_sbus_queue = q;
}

void algorithm_set_ctrl_queue(QueueHandle_t q)
{
    s_control_queue = q;
}

/*
 * TODO (GPS):
 * void algorithm_set_gps_queue(QueueHandle_t q) { s_gps_queue = q; }
 */