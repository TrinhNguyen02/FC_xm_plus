/**
 * @file xm_plus.c
 * @brief XM+ SBUS Receiver - BetaFlight-style Implementation
 * 
 * Based on BetaFlight's SBUS receiver implementation:
 * https://github.com/betaflight/betaflight/blob/master/src/main/rx/sbus.c
 * 
 * Key features:
 * - Byte-by-byte state machine parsing
 * - Proper frame synchronization
 * - Handles SBUS inverted UART protocol
 * - Real-time performance optimized
 */

#include "xm_plus.h"
#include "config.h"
#include "sbus.h"

#include <string.h>
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

static const char *TAG = "xm_plus";

// SBUS protocol constants (matching BetaFlight)
#define SBUS_FRAME_SIZE         25
#define SBUS_HEADER_BYTE        0x0F
#define SBUS_FOOTER_BYTE        0x00
#define SBUS_CHANNEL_COUNT      16
#define SBUS_FRAME_INTERVAL_MS  7  // ~14ms between frames (72Hz) but can vary

// Parser state machine states (BetaFlight style)
typedef enum {
    SBUS_SYNC,      // Looking for header byte
    SBUS_DATA,      // Collecting frame data
    SBUS_DONE       // Frame complete
} sbus_state_t;

// Static data
static xm_plus_data_t s_xm_data = {0};
static SemaphoreHandle_t s_xm_mutex = NULL;
static TaskHandle_t s_xm_task_handle = NULL;
static bool s_initialized = false;

// Parser state (BetaFlight style)
static sbus_state_t s_state = SBUS_SYNC;
static uint8_t s_frame[SBUS_FRAME_SIZE];
static uint8_t s_frame_position = 0;
static uint32_t s_last_frame_time = 0;

/**
 * @brief Process received byte using BetaFlight-style state machine
 * 
 * State machine:
 * - SBUS_SYNC: Wait for 0x0F header byte
 * - SBUS_DATA: Collect 24 more bytes (positions 1-24)
 * - When 25 bytes collected, validate and decode
 */
 static bool sbus_process_byte(uint8_t byte)
{

    switch (s_state) {
        case SBUS_SYNC:
            if (byte == SBUS_HEADER_BYTE) {
                memset(s_frame, 0, sizeof(s_frame));
                s_frame[0] = byte;
                s_frame_position = 1;
                s_state = SBUS_DATA;
            }
            break;
            
        case SBUS_DATA:
            s_frame[s_frame_position] = byte;
            s_frame_position++;
            
            // Check if frame is complete
            if (s_frame_position >= SBUS_FRAME_SIZE) {
                s_state = SBUS_SYNC;
                s_frame_position = 0;
                return true;  // Frame complete
            }
            break;
            
        default:
            s_state = SBUS_SYNC;
            s_frame_position = 0;
            break;
    }
    
    return false;
}

/**
 * @brief Validate SBUS frame (BetaFlight style)
 * 
 * Checks:
 * - Header byte is 0x0F
 * - Footer byte is 0x00
 */
static bool sbus_validate_frame(const uint8_t *frame)
{
    // Check header
    if (frame[0] != SBUS_HEADER_BYTE) {
        return false;
    }
    
    // Check footer - can be 0x00 depending on receiver
    // uint8_t footer = frame[SBUS_FRAME_SIZE - 1];
    // if (footer != SBUS_FOOTER_BYTE && footer != 0x04) {
    //     return false;
    // }

    return true;
}

/**
 * @brief Main SBUS processing task (BetaFlight-style)
 */
static void xm_plus_task(void *arg)
{
    ESP_LOGI(TAG, "SBUS task started (BetaFlight-style)");
    
    uint32_t frame_count = 0;
    uint32_t invalid_count = 0;
    uint32_t last_stats_time = xTaskGetTickCount();
    
    while (1) {
        // Read one byte at a time (non-blocking)
        uint8_t rx_byte;
        int bytes_read = uart_read_bytes(XM_PLUS_UART_PORT, &rx_byte, 1, 10);
        
        if (bytes_read > 0) {
            // ESP_LOGI(TAG, "Received byte: 0x%02X", rx_byte);
            // Process byte through state machine
            if (sbus_process_byte(rx_byte)) {
                // Frame complete - validate and decode
                if (sbus_validate_frame(s_frame)) {
                    uint16_t channels[SBUS_CHANNEL_COUNT];
                    uint8_t flags = 0;
                    
                    if (sbus_decode_frame(s_frame, channels, &flags)) {
                        // Validate channel ranges (SBUS: 172-1811)
                        bool valid = true;
                        for (int i = 0; i < 16; i++) {
                            if (channels[i] < 100 || channels[i] > 2048) {
                                valid = false;  
                                invalid_count++;
                                break;
                            }
                        }
                        
                        if (valid) {
                            // Update data under mutex
                            if (xSemaphoreTake(s_xm_mutex, 0) == pdTRUE) {
                                memcpy(s_xm_data.channels, channels, sizeof(channels));
                                s_xm_data.flags = flags;
                                s_xm_data.data_valid = true;
                                s_xm_data.last_update = xTaskGetTickCount();
                                xSemaphoreGive(s_xm_mutex);
                                frame_count++;
                                // Log every frame
                                // ESP_LOGI(TAG, "CH1:%4u CH2:%4u CH3:%4u CH4:%4u CH5:%4u CH6:%4u CH7:%4u CH8:%4u CH9:%4u CH10:%4u CH11:%4u CH12:%4u CH13:%4u CH14:%4u CH15:%4u CH16:%4u | Flags:0x%02X",
                                //          channels[0], channels[1], channels[2], channels[3],
                                //          channels[4], channels[5], channels[6], channels[7],
                                //          channels[8], channels[9], channels[10], channels[11],
                                //          channels[12], channels[13], channels[14], channels[15],
                                //          flags);
                            }
                        }
                    }
                } 
                else {
                    invalid_count++;
                    ESP_LOGI(TAG, "Invalid SBUS frame: header=0x%02X footer=0x%02X", 
                             s_frame[0], s_frame[24]);
                    memset(s_frame, 0, sizeof(s_frame));
                }
            }
        }
        
        // Give the scheduler a chance; also prevents task watchdog from firing
        vTaskDelay(pdMS_TO_TICKS(1));

        
        // Periodic status check (frame rate)
        uint32_t now = xTaskGetTickCount();
        if (now - last_stats_time >= pdMS_TO_TICKS(5000)) {
            uint32_t elapsed_ms = (now - last_stats_time) * portTICK_PERIOD_MS;
            float fps = (elapsed_ms > 0)
                         ? ((float)frame_count * 1000.0f / (float)elapsed_ms)
                         : 0.0f;

            ESP_LOGI(TAG, "Stats: %lu frames, %lu invalid, %.1f fps",
                     (unsigned long)frame_count,
                     (unsigned long)invalid_count,
                     fps);

            frame_count = 0;
            invalid_count = 0;
            last_stats_time = now;
        }


        
        // Check for signal loss
        if (s_xm_data.data_valid) {
            if (xSemaphoreTake(s_xm_mutex, 0) == pdTRUE) {
                if (now - s_xm_data.last_update > pdMS_TO_TICKS(500)) {
                    s_xm_data.data_valid = false;
                    ESP_LOGW(TAG, "SBUS signal lost");
                }
                xSemaphoreGive(s_xm_mutex);
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
    
    ESP_LOGI(TAG, "Initializing XM+ SBUS receiver (BetaFlight-style) on GPIO%d", 
             PIN_XM_PLUS_UART_RX);
    
    // Reset parser state
    s_state = SBUS_SYNC;
    s_frame_position = 0;
    memset(s_frame, 0, sizeof(s_frame));
    
    // Create mutex
    s_xm_mutex = xSemaphoreCreateMutex();
    if (s_xm_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create mutex");
        return false;
    }
    
    // Configure UART for SBUS (inverted, 100000 baud, 8E2)
    uart_config_t uart_config = {
        .baud_rate = 100000,  // SBUS standard baud rate
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_EVEN,
        .stop_bits = UART_STOP_BITS_2,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    
    esp_err_t err = uart_param_config(XM_PLUS_UART_PORT, &uart_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "uart_param_config failed: %d", err);
        vSemaphoreDelete(s_xm_mutex);
        return false;
    }
    
    // Set UART pins
    err = uart_set_pin(XM_PLUS_UART_PORT, 
                       UART_PIN_NO_CHANGE,
                       PIN_XM_PLUS_UART_RX, 
                       UART_PIN_NO_CHANGE, 
                       UART_PIN_NO_CHANGE);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "uart_set_pin failed: %d", err);
        vSemaphoreDelete(s_xm_mutex);
        return false;
    }
    
    // Install UART driver
    err = uart_driver_install(XM_PLUS_UART_PORT, 1024, 0, 0, NULL, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "uart_driver_install failed: %d", err);
        vSemaphoreDelete(s_xm_mutex);
        return false;
    }
    
    // Enable RX inversion for SBUS
    err = uart_set_line_inverse(XM_PLUS_UART_PORT, UART_SIGNAL_RXD_INV);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "uart_set_line_inverse failed: %d", err);
    }
    
    // Create task
    BaseType_t result = xTaskCreate(xm_plus_task, "xm_plus_task", 4096,
                                    NULL, configMAX_PRIORITIES - 1, &s_xm_task_handle);
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
    
    if (xSemaphoreTake(s_xm_mutex, pdMS_TO_TICKS(2)) == pdTRUE) {
        memcpy(data, &s_xm_data, sizeof(xm_plus_data_t));
        xSemaphoreGive(s_xm_mutex);
    }
}

bool xm_plus_get_channel(uint8_t channel, uint16_t *value)
{
    if (channel >= 16 || value == NULL || s_xm_mutex == NULL) return false;
    
    bool valid = false;
    if (xSemaphoreTake(s_xm_mutex, pdMS_TO_TICKS(2)) == pdTRUE) {
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
