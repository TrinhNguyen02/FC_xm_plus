/**
 * @file main.c
 * @brief Main Application for ESP32-C3 Flight Controller
 * 
 * This is the main entry point for the flight controller firmware.
 * It initializes all subsystems and creates FreeRTOS tasks for:
 * - SBUS receiver processing (XM+)
 * - Flight control output (PWM)
 * - System monitoring and telemetry
 * 
 * Architecture:
 * - FreeRTOS for real-time multitasking
 * - Thread-safe communication between tasks via mutexes
 * - Failsafe handling for loss of signal
 */

#include "config.h"
#include "xm_plus.h"
#include "control.h"
#include "sbus.h"

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"

static const char *TAG = "main";

// System state
static bool s_system_running = false;
static uint32_t s_loop_counter = 0;

/**
 * @brief System monitor task
 * 
 * Periodically logs system status including:
 * - Free heap memory
 * - Task stack watermarks
 * - SBUS signal status
 * - Armed/disarmed state
 */

/**
 * @brief Initialize non-volatile storage (NVS)
 * 
 * Required for WiFi/BT if used later, and for storing calibration data.
 */
static esp_err_t init_nvs(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS partition was truncated, erasing...");
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    return err;
}

/**
 * @brief Main application entry point
 */
void app_main(void)
{
    vTaskDelay(pdMS_TO_TICKS(3000));  // Wait for system to stabilize
    ESP_LOGI("BOOT", "boot ok");

    esp_err_t err;
    
    // Set log level
    esp_log_level_set(TAG, ESP_LOG_INFO);
    esp_log_level_set("xm_plus", ESP_LOG_INFO);
    esp_log_level_set("control", ESP_LOG_INFO);

    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "  ESP32-C3 Flight Controller v1.0");
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "Free heap at startup: %lu bytes", esp_get_free_heap_size());
    
    // Initialize NVS (needed for future WiFi/BT features)
    err = init_nvs();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "NVS initialization failed: %d", err);
        // Continue anyway, NVS is not critical for basic operation
    }
    
    // Initialize control system first (PWM outputs)
    // This ensures all outputs are in a safe state before we start reading inputs
    ESP_LOGI(TAG, "Initializing control system...");
    if (!control_init()) {
        ESP_LOGE(TAG, "Failed to initialize control system!");
        return;
    }
    
    // Initialize XM+ SBUS receiver
    ESP_LOGI(TAG, "Initializing XM+ SBUS receiver...");
    if (!xm_plus_init()) {
        ESP_LOGE(TAG, "Failed to initialize XM+ receiver!");
        control_deinit();
        return;
    }
    
    ESP_LOGI(TAG, "Initialization complete. System running.");
    ESP_LOGI(TAG, "Free heap after init: %lu bytes", esp_get_free_heap_size());
    
    s_system_running = true;
    
    // Create message queue for decoded SBUS data.
    // Mode: keep ALL frames (no overwrite). Queue size should be large enough to avoid drop.
    // NOTE: If control_task can't keep up, FreeRTOS will block xQueueSend (we use short block time).
    enum { SBUS_QUEUE_LENGTH = 32 };
    QueueHandle_t sbus_queue = xQueueCreate(SBUS_QUEUE_LENGTH, sizeof(xm_plus_data_t));

    if (sbus_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create SBUS queue");
    } else {
        control_set_sbus_queue(sbus_queue);
    }

    while (1) {
        s_loop_counter++;

        // Read latest SBUS data from xm_plus_task.
        xm_plus_data_t xm_data;
        xm_plus_get_data(&xm_data);

        // Send to control task via queue (non-blocking, short timeout)
        if (sbus_queue != NULL) {
            if (xQueueSend(sbus_queue, &xm_data, pdMS_TO_TICKS(5)) != pdTRUE) {
                ESP_LOGW(TAG, "SBUS queue full, dropping frame");
            }
        }
        control_tmp();
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

