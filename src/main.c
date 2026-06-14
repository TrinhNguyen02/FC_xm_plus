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
static void monitor_task(void *arg)
{
    ESP_LOGI(TAG, "Monitor task started");
    
    while (1) {
        // Log system status every 10 seconds
        vTaskDelay(pdMS_TO_TICKS(10000));
        
        // Get free heap memory
        uint32_t free_heap = esp_get_free_heap_size();
        
        // Get SBUS status
        bool sbus_valid = xm_plus_is_valid();
        bool sbus_failsafe = xm_plus_is_failsafe();
        bool armed = control_is_armed();
        
        // Log status
        ESP_LOGI(TAG, "Heap: %lu | SBUS: %s | Failsafe: %s | Armed: %s | Loop: %lu | Stack: %d",
                 free_heap, 
                 sbus_valid ? "OK" : "LOST",
                 sbus_failsafe ? "ACTIVE" : "OK",
                 armed ? "YES" : "NO",
                 s_loop_counter,
                 uxTaskGetStackHighWaterMark(NULL));
        
        // Check for low memory warning
        if (free_heap < 10000) {
            ESP_LOGW(TAG, "Low memory: %lu bytes free", free_heap);
        }
    }
}

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
    
    // Create system monitor task
    err = xTaskCreate(
        monitor_task,
        "monitor_task",
        TASK_MONITOR_STACK,
        NULL,
        TASK_MONITOR_PRIORITY,
        NULL
    );
    
    if (err != pdPASS) {
        ESP_LOGW(TAG, "Failed to create monitor task");
        // Continue anyway, monitor is not critical
    }
    
    ESP_LOGI(TAG, "Initialization complete. System running.");
    ESP_LOGI(TAG, "Free heap after init: %lu bytes", esp_get_free_heap_size());
    
    s_system_running = true;
    
    // Main loop - monitors SBUS failsafe and calls control update
    // The actual SBUS processing happens in the xm_plus task
    // This loop just coordinates between subsystems
    while (1) {
        s_loop_counter++;
        
        // Get latest SBUS data
        xm_plus_data_t xm_data;
        xm_plus_get_data(&xm_data);
        
        // Check for failsafe / signal validity and update control directly
        if (xm_data.data_valid) {
            if (xm_data.flags & 0x08) {
                if (control_is_armed()) {
                    control_failsafe();
                    ESP_LOGW(TAG, "Failsafe triggered - disarming");
                }
            } else {
                control_update_from_sbus(xm_data.channels);
            }
        } else {
            if (control_is_armed()) {
                control_failsafe();
                ESP_LOGW(TAG, "Signal lost while armed - failsafe");
            }
        }

        // Small delay to prevent watchdog triggers and allow other tasks

        vTaskDelay(pdMS_TO_TICKS(10));  // 100Hz main loop
    }
}