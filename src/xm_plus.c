/**
 * @file xm_plus.c
 * @brief XM+ SBUS Receiver Implementation for ESP32-C3 Flight Controller
 * 
 * This module handles communication with the FrSky XM+ receiver via SBUS protocol.
 * SBUS uses inverted UART at 100000 baud, 8 data bits, even parity, 2 stop bits.
 */

#include "xm_plus.h"
#include "config.h"
#include "sbus.h"

#include <string.h>
#include <stdio.h>
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

static const char *TAG = "xm_plus";

static xm_plus_data_t s_xm_data = {0};
static SemaphoreHandle_t s_xm_mutex = NULL;
static TaskHandle_t s_xm_task_handle = NULL;
static bool s_initialized = false;

static uint8_t s_rx_buffer[128];
static uint8_t s_parser_buffer[256];
static int s_parser_write_idx = 0;

static bool validate_sbus_frame(const uint8_t frame[SBUS_FRAME_SIZE])
{
    // Check frame header
    if (frame[0] != 0x0F) {
        return false;
    }
    
    // Check that all channel values are within valid SBUS range (172-1811)
    for (int i = 0; i < 16; i++) {
        uint16_t ch = (frame[1 + i * 2] << 8) | frame[2 + i * 2];
        // Only check first few bytes since SBUS is packed differently
        // We'll validate after decoding instead
    }
    
    return true;
}

static int process_sbus_frames(void)
{
    int frames_processed = 0;
    
    for (int i = 0; i <= s_parser_write_idx - SBUS_FRAME_SIZE; i++) {
        if (s_parser_buffer[i] == 0x0F) {
            uint8_t frame[SBUS_FRAME_SIZE];
            memcpy(frame, &s_parser_buffer[i], SBUS_FRAME_SIZE);
            
            if (validate_sbus_frame(frame)) {
                uint16_t channels[SBUS_CHANNEL_COUNT];
                uint8_t flags = 0;
                
                if (sbus_decode_frame(frame, channels, &flags)) {
                    // Validate channel values - SBUS range is 172-1811
                    bool valid_frame = true;
                    for (int ch = 0; ch < 16; ch++) {
                        if (channels[ch] < 100 || channels[ch] > 2000) {
                            valid_frame = false;
                            break;
                        }
                    }
                    
                    if (valid_frame && xSemaphoreTake(s_xm_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                        memcpy(s_xm_data.channels, channels, sizeof(channels));
                        s_xm_data.flags = flags;
                        s_xm_data.data_valid = true;
                        s_xm_data.last_update = xTaskGetTickCount();
                        xSemaphoreGive(s_xm_mutex);
                        frames_processed++;
                        
                        ESP_LOGI(TAG, "CH1:%4u CH2:%4u CH3:%4u CH4:%4u CH5:%4u CH6:%4u CH7:%4u CH8:%4u CH9:%4u CH10:%4u CH11:%4u CH12:%4u CH13:%4u CH14:%4u CH15:%4u CH16:%4u | Flags:0x%02X",
                                 channels[0], channels[1], channels[2], channels[3],
                                 channels[4], channels[5], channels[6], channels[7],
                                 channels[8], channels[9], channels[10], channels[11],
                                 channels[12], channels[13], channels[14], channels[15],
                                 flags);
                    }
                }
            }
            i += SBUS_FRAME_SIZE - 1;
        }
    }
    
    return frames_processed;
}

static void xm_plus_task(void *arg)
{
    ESP_LOGI(TAG, "SBUS task started");
    
    uint32_t last_status_log = xTaskGetTickCount();
    uint32_t total_frames = 0;
    uint32_t total_bytes = 0;
    
    while (1) {
        // Read bytes with short timeout to prevent WDT
        int bytes_read = uart_read_bytes(XM_PLUS_UART_PORT, s_rx_buffer, 32, pdMS_TO_TICKS(5));
        
        if (bytes_read > 0) {
            total_bytes += bytes_read;
            
            for (int i = 0; i < bytes_read && i < 32; i++) {
                s_parser_buffer[s_parser_write_idx] = s_rx_buffer[i];
                s_parser_write_idx++;
                
                if (s_parser_write_idx >= (int)sizeof(s_parser_buffer)) {
                    memmove(s_parser_buffer, &s_parser_buffer[128], 128);
                    s_parser_write_idx = 128;
                }
            }
            
            int frames = process_sbus_frames();
            total_frames += frames;
            
            // Clean up buffer
            if (s_parser_write_idx > 128) {
                int keep_bytes = s_parser_write_idx - 128;
                memmove(s_parser_buffer, &s_parser_buffer[128], keep_bytes);
                s_parser_write_idx = keep_bytes;
            }
        }
        
        // Always yield to prevent WDT
        vTaskDelay(pdMS_TO_TICKS(1));
        
        // Status check every second
        uint32_t now = xTaskGetTickCount();
        if (now - last_status_log >= pdMS_TO_TICKS(1000)) {
            last_status_log = now;
            
            if (xSemaphoreTake(s_xm_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                if (s_xm_data.data_valid) {
                    if (now - s_xm_data.last_update > pdMS_TO_TICKS(500)) {
                        s_xm_data.data_valid = false;
                        ESP_LOGW(TAG, "SBUS signal lost");
                    }
                }
                xSemaphoreGive(s_xm_mutex);
            }
            
            // Stats every 5 seconds
            static uint32_t stats_timer = 0;
            if (now - stats_timer >= pdMS_TO_TICKS(5000)) {
                stats_timer = now;
                ESP_LOGI(TAG, "Stats: %lu bytes, %lu frames, rate: %.1f fps", 
                         total_bytes, total_frames, 
                         (float)total_frames * 1000 / 5000);
                total_bytes = 0;
                total_frames = 0;
            }
        }
    }
}

bool xm_plus_init(void)
{
    if (s_initialized) {
        ESP_LOGW(TAG, "Already initialized");
        return true;
    }
    
    ESP_LOGI(TAG, "Initializing XM+ SBUS receiver on GPIO%d", PIN_XM_PLUS_UART_RX);
    
    memset(s_parser_buffer, 0, sizeof(s_parser_buffer));
    s_parser_write_idx = 0;
    
    s_xm_mutex = xSemaphoreCreateMutex();
    if (s_xm_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create mutex");
        return false;
    }
    
    uart_config_t uart_config = {
        .baud_rate = XM_PLUS_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_EVEN,
        .stop_bits = UART_STOP_BITS_2,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
        .rx_flow_ctrl_thresh = 122,
    };
    
    esp_err_t err = uart_param_config(XM_PLUS_UART_PORT, &uart_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "uart_param_config failed: %d", err);
        vSemaphoreDelete(s_xm_mutex);
        return false;
    }
    
    err = uart_set_pin(XM_PLUS_UART_PORT, UART_PIN_NO_CHANGE, PIN_XM_PLUS_UART_RX, 
                       UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "uart_set_pin failed: %d", err);
        vSemaphoreDelete(s_xm_mutex);
        return false;
    }
    
    err = uart_driver_install(XM_PLUS_UART_PORT, XM_PLUS_UART_BUF_SIZE, 0, 0, NULL, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "uart_driver_install failed: %d", err);
        vSemaphoreDelete(s_xm_mutex);
        return false;
    }
    
    err = uart_set_line_inverse(XM_PLUS_UART_PORT, UART_SIGNAL_RXD_INV);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "uart_set_line_inverse failed: %d", err);
    }
    
    BaseType_t result = xTaskCreate(xm_plus_task, "xm_plus_task", TASK_XM_PLUS_STACK,
                                    NULL, TASK_XM_PLUS_PRIORITY, &s_xm_task_handle);
    if (result != pdPASS) {
        ESP_LOGE(TAG, "Failed to create SBUS task");
        uart_driver_delete(XM_PLUS_UART_PORT);
        vSemaphoreDelete(s_xm_mutex);
        return false;
    }
    
    s_initialized = true;
    ESP_LOGI(TAG, "XM+ SBUS receiver initialized");
    return true;
}

void xm_plus_get_data(xm_plus_data_t *data)
{
    if (data == NULL || s_xm_mutex == NULL) return;
    
    if (xSemaphoreTake(s_xm_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        memcpy(data, &s_xm_data, sizeof(xm_plus_data_t));
        xSemaphoreGive(s_xm_mutex);
    }
}

bool xm_plus_get_channel(uint8_t channel, uint16_t *value)
{
    if (channel >= 16 || value == NULL || s_xm_mutex == NULL) return false;
    
    bool valid = false;
    if (xSemaphoreTake(s_xm_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        if (s_xm_data.data_valid) {
            *value = s_xm_data.channels[channel];
            valid = true;
        }
        xSemaphoreGive(s_xm_mutex);
    }
    return valid;
}

bool xm_plus_is_valid(void)
{
    bool valid = false;
    if (s_xm_mutex != NULL && xSemaphoreTake(s_xm_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        valid = s_xm_data.data_valid;
        xSemaphoreGive(s_xm_mutex);
    }
    return valid;
}

bool xm_plus_is_failsafe(void)
{
    bool failsafe = false;
    if (s_xm_mutex != NULL && xSemaphoreTake(s_xm_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        failsafe = (s_xm_data.flags & SBUS_FAILSAFE_MASK) != 0;
        xSemaphoreGive(s_xm_mutex);
    }
    return failsafe;
}

bool xm_plus_get_flags(uint8_t *flags)
{
    if (flags == NULL || s_xm_mutex == NULL) return false;
    
    bool valid = false;
    if (xSemaphoreTake(s_xm_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        if (s_xm_data.data_valid) {
            *flags = s_xm_data.flags;
            valid = true;
        }
        xSemaphoreGive(s_xm_mutex);
    }
    return valid;
}

void xm_plus_deinit(void)
{
    if (!s_initialized) return;
    
    if (s_xm_task_handle != NULL) {
        vTaskDelete(s_xm_task_handle);
        s_xm_task_handle = NULL;
    }
    
    uart_driver_delete(XM_PLUS_UART_PORT);
    
    if (s_xm_mutex != NULL) {
        vSemaphoreDelete(s_xm_mutex);
        s_xm_mutex = NULL;
    }
    
    memset(&s_xm_data, 0, sizeof(s_xm_data));
    s_initialized = false;
    
    ESP_LOGI(TAG, "XM+ SBUS receiver deinitialized");
}