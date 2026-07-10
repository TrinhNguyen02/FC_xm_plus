/**
 * @file main.c
 * @brief Main Application Entry Point — ESP32-C3 Flight Controller
 *
 * Initializes all subsystems and creates FreeRTOS tasks for:
 *   - IMU sensor (BMI160 + Madgwick AHRS)
 *   - Cascaded PID algorithm (outer 100Hz + inner 500Hz)
 *   - SBUS receiver processing (XM+)
 *   - Flight control output (LEDC PWM)
 *
 * Architecture:
 *   - FreeRTOS for real-time multitasking on single-core ESP32-C3
 *   - Two FreeRTOS queues: sbus_queue (SBUS→algorithm) and
 *     control_queue (algorithm→control)
 *   - Failsafe handling for loss of RC signal
 */

#include "config.h"
#include "xm_plus.h"
#include "control.h"
#include "sbus.h"
#include "imu.h"
#include "algorithm.h"

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_system.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"

static const char *TAG = "main";

static bool s_system_running = false;

/*================ Private Functions =================*/

static esp_err_t init_nvs(void)
{
    esp_err_t err = nvs_flash_init();

    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_LOGW(TAG, "NVS partition was truncated, erasing...");
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }

    return err;
}

/*================ Application Entry Point =================*/

void app_main(void)
{
    vTaskDelay(pdMS_TO_TICKS(3000));  /* Wait for power rails to stabilize */
    ESP_LOGI("BOOT", "boot ok");

    esp_log_level_set(TAG,       ESP_LOG_INFO);
    esp_log_level_set("xm_plus", ESP_LOG_INFO);
    esp_log_level_set("control", ESP_LOG_INFO);

    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "  ESP32-C3 Flight Controller v1.0");
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "Free heap at startup: %lu bytes", esp_get_free_heap_size());

    esp_err_t err = init_nvs();

    if (err != ESP_OK)
    {
        ESP_LOGW(TAG, "NVS initialization failed: %d", err);
        /* Continue — NVS is not critical for basic operation */
    }

    /* Create SBUS queue: xm_plus → algorithm */
    QueueHandle_t sbus_queue = xQueueCreate(SBUS_QUEUE_LENGTH, sizeof(xm_plus_data_t));

    if (sbus_queue == NULL)
    {
        ESP_LOGE(TAG, "Failed to create SBUS queue");
    }
    else
    {
        algorithm_set_sbus_queue(sbus_queue);
        xm_plus_set_output_queue(sbus_queue);
    }

    /* Create control queue: algorithm → control */
    QueueHandle_t control_queue = xQueueCreate(CONTROL_QUEUE_LENGTH, sizeof(ctrl_value_input_t));

    if (control_queue == NULL)
    {
        ESP_LOGE(TAG, "Failed to create control queue");
    }
    else
    {
        algorithm_set_ctrl_queue(control_queue);
        control_set_input_queue(control_queue);
    }

    /* Initialize subsystems in dependency order */
    ESP_LOGI(TAG, "Initializing IMU module...");

    if (!imu_init())
    {
        ESP_LOGE(TAG, "Failed to initialize IMU module!");
        return;
    }

    ESP_LOGI(TAG, "Initializing algorithm function...");
    if (!algorithm_init())
    {
        ESP_LOGE(TAG, "Failed to initialize algorithm task!");
        return;
    }

    ESP_LOGI(TAG, "Initializing control system...");
    if (!control_init())
    {
        ESP_LOGE(TAG, "Failed to initialize control system!");
        return;
    }

    ESP_LOGI(TAG, "Initializing XM+ SBUS receiver...");
    if (!xm_plus_init())
    {
        ESP_LOGE(TAG, "Failed to initialize XM+ receiver!");
        return;
    }

    ESP_LOGI(TAG, "Initialization complete. System running.");
    ESP_LOGI(TAG, "Free heap after init: %lu bytes", esp_get_free_heap_size());

    s_system_running = true;

    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}