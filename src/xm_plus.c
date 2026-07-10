/**
 * @file xm_plus.c
 * @brief XM+ SBUS Receiver — BetaFlight-style Implementation for ESP32-C3
 *
 * Handles SBUS frame reception over inverted UART (100000 baud, 8E2),
 * decodes 16 RC channels, validates frame integrity, and publishes
 * decoded data to a FreeRTOS queue consumed by the algorithm task.
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

/*================ Constants & Macros =================*/

static const char *TAG = "xm_plus";

/*================ Global Variables =================*/

flight_mode_t g_fc_control_mode = CONTROL_MODE_ANGLE;

/*================ Static Variables =================*/

static xm_plus_data_t    s_xm_data         = {0};
static SemaphoreHandle_t s_xm_mutex        = NULL;
static TaskHandle_t      s_xm_task_handle  = NULL;
static bool              s_xm_initialized  = false;
static QueueHandle_t     s_output_queue    = NULL;

static sbus_state_t s_state          = SBUS_SYNC;
static uint8_t      s_frame[SBUS_FRAME_SIZE];
static uint8_t      s_frame_position = 0;

/*================ Private Function Prototypes =================*/

static bool sbus_process_byte(uint8_t byte);
static bool sbus_validate_frame(const uint8_t *frame);
static void xm_plus_task(void *arg);

/*================ SBUS Frame Parser =================*/

static bool sbus_process_byte(uint8_t byte)
{
    switch (s_state)
    {
        case SBUS_SYNC:
            if (byte == SBUS_HEADER_BYTE)
            {
                memset(s_frame, 0, sizeof(s_frame));
                s_frame[0]       = byte;
                s_frame_position = 1;
                s_state          = SBUS_DATA;
            }
            break;

        case SBUS_DATA:
            s_frame[s_frame_position] = byte;
            s_frame_position++;

            if (s_frame_position >= SBUS_FRAME_SIZE)
            {
                s_state          = SBUS_SYNC;
                s_frame_position = 0;
                return true;
            }
            break;

        default:
            s_state          = SBUS_SYNC;
            s_frame_position = 0;
            break;
    }

    return false;
}

static bool sbus_validate_frame(const uint8_t *frame)
{
    if (frame[0] != SBUS_HEADER_BYTE)
    {
        return false;
    }

    uint8_t footer = frame[SBUS_FRAME_SIZE - 1];

    if (footer != SBUS_FOOTER_BYTE && footer != SBUS_FOOTER_2_BYTE)
    {
        return false;
    }

    return true;
}

/*================ SBUS Task =================*/

static void xm_plus_task(void *arg)
{
    ESP_LOGI(TAG, "XM+ SBUS task started");

    uint32_t frame_count      = 0;
    uint32_t invalid_count    = 0;
    uint32_t last_stats_time  = xTaskGetTickCount();

    while (1)
    {
        uint8_t rx_buf[64];
        int bytes_read = uart_read_bytes(XM_PLUS_UART_PORT, rx_buf, sizeof(rx_buf), SBUS_READ_TIMEOUT_MS);

        if (bytes_read > 0)
        {
            for (int i = 0; i < bytes_read; i++)
            {
                if (sbus_process_byte(rx_buf[i]))
                {
                    if (sbus_validate_frame(s_frame))
                    {
                        uint16_t channels[SBUS_CHANNEL_COUNT];
                        uint8_t  flags = 0;

                        if (sbus_decode_frame(s_frame, channels, &flags))
                        {
                            bool valid = true;

                            for (int c = 0; c < SBUS_CHANNEL_COUNT; c++)
                            {
                                if (channels[c] < SBUS_CHANNEL_VALUE_MIN || channels[c] > SBUS_CHANNEL_VALUE_MAX)
                                {
                                    valid = false;
                                    invalid_count++;
                                    break;
                                }
                            }

                            if (valid)
                            {
                                if (xSemaphoreTake(s_xm_mutex, 0) == pdTRUE)
                                {
                                    memcpy(s_xm_data.channels, channels, sizeof(channels));
                                    s_xm_data.flags       = flags;
                                    s_xm_data.data_valid  = true;
                                    s_xm_data.last_update = xTaskGetTickCount();
                                    xSemaphoreGive(s_xm_mutex);
                                    frame_count++;

                                    if (s_output_queue != NULL)
                                    {
                                        xQueueOverwrite(s_output_queue, &s_xm_data);
                                    }
                                }
                            }
                        }
                    }
                    else
                    {
                        invalid_count++;
                        memset(s_frame, 0, sizeof(s_frame));
                    }
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(1));

        uint32_t now = xTaskGetTickCount();

        if (now - last_stats_time >= pdMS_TO_TICKS(SBUS_STATUS_INTERVAL_MS))
        {
            uint32_t elapsed_ms = (now - last_stats_time) * portTICK_PERIOD_MS;
            float fps = (elapsed_ms > 0) ? ((float)frame_count * 1000.0f / (float)elapsed_ms) : 0.0f;

            ESP_LOGI(TAG, "Status: %lu frames, %lu invalid | Freq: %.1f fps",
                     (unsigned long)frame_count, (unsigned long)invalid_count, fps);

            frame_count      = 0;
            invalid_count    = 0;
            last_stats_time  = now;
        }

        /* Invalidate data if no frame received within timeout */
        if (s_xm_data.data_valid)
        {
            if (xSemaphoreTake(s_xm_mutex, 0) == pdTRUE)
            {
                if (now - s_xm_data.last_update > pdMS_TO_TICKS(SBUS_SIGNAL_LOST_TIMEOUT_MS))
                {
                    s_xm_data.data_valid = false;
                    ESP_LOGW(TAG, "SBUS signal lost");
                }
                xSemaphoreGive(s_xm_mutex);
            }
        }
    }
}

/*================ Initialization / Deinitialization =================*/

bool xm_plus_init(void)
{
    ESP_LOGI(TAG, "Initializing XM+ receiver...");
    if (s_xm_initialized)
    {
        return true;
    }

    s_state          = SBUS_SYNC;
    s_frame_position = 0;
    memset(s_frame, 0, sizeof(s_frame));

    s_xm_mutex = xSemaphoreCreateMutex();

    if (s_xm_mutex == NULL)
    {
        return false;
    }

    uart_config_t uart_config = {
        .baud_rate  = 100000,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_EVEN,
        .stop_bits  = UART_STOP_BITS_2,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };

    if (uart_param_config(XM_PLUS_UART_PORT, &uart_config) != ESP_OK)
    {
        vSemaphoreDelete(s_xm_mutex);
        return false;
    }

    if (uart_set_pin(XM_PLUS_UART_PORT, UART_PIN_NO_CHANGE, PIN_XM_PLUS_UART_RX,
                     UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE) != ESP_OK)
    {
        vSemaphoreDelete(s_xm_mutex);
        return false;
    }

    if (uart_driver_install(XM_PLUS_UART_PORT, 1024, 0, 0, NULL, 0) != ESP_OK)
    {
        vSemaphoreDelete(s_xm_mutex);
        return false;
    }

    uart_set_line_inverse(XM_PLUS_UART_PORT, UART_SIGNAL_RXD_INV);

    ESP_LOGI(TAG, "XM+ receiver initialized successfully.");
    if (xTaskCreate(xm_plus_task, "xm_plus_task", STACK_XM_PLUS, NULL,
                    TASK_XM_PLUS_PRIORITY, &s_xm_task_handle) != pdPASS)
    {
        uart_driver_delete(XM_PLUS_UART_PORT);
        vSemaphoreDelete(s_xm_mutex);
        return false;
    }

    s_xm_initialized = true;
    return true;
}

void xm_plus_deinit(void)
{
    if (!s_xm_initialized)
    {
        return;
    }

    if (s_xm_task_handle != NULL)
    {
        vTaskDelete(s_xm_task_handle);
        s_xm_task_handle = NULL;
    }

    uart_driver_delete(XM_PLUS_UART_PORT);

    if (s_xm_mutex != NULL)
    {
        vSemaphoreDelete(s_xm_mutex);
        s_xm_mutex = NULL;
    }

    memset(&s_xm_data, 0, sizeof(s_xm_data));
    s_xm_initialized = false;
}

/*================ Public API =================*/

void xm_plus_set_output_queue(QueueHandle_t q)
{
    s_output_queue = q;
}

flight_mode_t xm_plus_get_control_mode(void)
{
    uint16_t ch9_value = 0;

    if (xm_plus_get_channel(SBUS_CH_9, &ch9_value))
    {
        if (ch9_value < 700)
        {
            g_fc_control_mode = CONTROL_MODE_ANGLE;
        }
        else if (ch9_value < 1500)
        {
            g_fc_control_mode = CONTROL_MODE_HORIZON;
        }
        else
        {
            g_fc_control_mode = CONTROL_MODE_ACRO;
        }
    }
    else
    {
        g_fc_control_mode = CONTROL_MODE_ANGLE;
    }

    return g_fc_control_mode;
}

void xm_plus_get_data(xm_plus_data_t *data)
{
    if (data != NULL && s_xm_mutex != NULL &&
        xSemaphoreTake(s_xm_mutex, pdMS_TO_TICKS(2)) == pdTRUE)
    {
        memcpy(data, &s_xm_data, sizeof(xm_plus_data_t));
        xSemaphoreGive(s_xm_mutex);
    }
}

bool xm_plus_get_channel(uint8_t channel, uint16_t *value)
{
    if (channel >= 16 || value == NULL || s_xm_mutex == NULL)
    {
        return false;
    }

    bool valid = false;

    if (xSemaphoreTake(s_xm_mutex, pdMS_TO_TICKS(2)) == pdTRUE)
    {
        if (s_xm_data.data_valid)
        {
            *value = s_xm_data.channels[channel];
            valid  = true;
        }
        xSemaphoreGive(s_xm_mutex);
    }

    return valid;
}

bool xm_plus_is_valid(void)
{
    bool valid = false;

    if (s_xm_mutex != NULL && xSemaphoreTake(s_xm_mutex, pdMS_TO_TICKS(100)) == pdTRUE)
    {
        valid = s_xm_data.data_valid;
        xSemaphoreGive(s_xm_mutex);
    }

    return valid;
}

bool xm_plus_is_failsafe(void)
{
    bool failsafe = false;

    if (s_xm_mutex != NULL && xSemaphoreTake(s_xm_mutex, pdMS_TO_TICKS(100)) == pdTRUE)
    {
        failsafe = (s_xm_data.flags & SBUS_FAILSAFE_MASK) != 0;
        xSemaphoreGive(s_xm_mutex);
    }

    return failsafe;
}

bool xm_plus_get_flags(uint8_t *flags)
{
    if (flags == NULL || s_xm_mutex == NULL)
    {
        return false;
    }

    bool valid = false;

    if (xSemaphoreTake(s_xm_mutex, pdMS_TO_TICKS(100)) == pdTRUE)
    {
        if (s_xm_data.data_valid)
        {
            *flags = s_xm_data.flags;
            valid  = true;
        }
        xSemaphoreGive(s_xm_mutex);
    }

    return valid;
}