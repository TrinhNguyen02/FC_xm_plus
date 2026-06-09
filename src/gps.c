/**
 * @file gps.c
 * @brief GPS Module Handler Stub for ESP32-C3 Flight Controller
 * 
 * This module will handle GPS data acquisition and processing.
 * Currently a stub for future development.
 * 
 * Planned features:
 * - NMEA sentence parsing
 * - Position, velocity, time data extraction
 * - GPS quality indicators
 * - Integration with flight controller for position hold, RTH
 */

#include "config.h"
#include "esp_log.h"

static const char *TAG = "gps";

// GPS module will use UART2 for communication
// Planned pin assignments:
// - GPS TX -> ESP32-C3 RX (GPIO2)
// - GPS RX -> ESP32-C3 TX (GPIO3)

/**
 * @brief GPS data structure (for future use)
 */
typedef struct {
    double latitude;
    double longitude;
    float altitude;
    float speed;
    float course;
    uint8_t satellites;
    uint8_t fix_type;
    uint32_t timestamp;
    bool valid;
} gps_data_t;

bool gps_init(void)
{
    ESP_LOGI(TAG, "GPS module initializing (stub - not implemented)");
    ESP_LOGI(TAG, "Planned configuration:");
    ESP_LOGI(TAG, "  - UART2: RX=GPIO2, TX=GPIO3");
    ESP_LOGI(TAG, "  - Baud rate: 115200 (configurable)");
    ESP_LOGI(TAG, "  - Protocol: NMEA 0183");
    ESP_LOGI(TAG, "  - Update rate: 10Hz");
    
    // TODO: Implement GPS initialization
    // 1. Configure UART2 for GPS communication
    // 2. Set up DMA for efficient data reception
    // 3. Create GPS parsing task
    // 4. Initialize NMEA parser
    // 5. Set up periodic position updates
    
    return true;
}

/**
 * @brief Get latest GPS data (thread-safe)
 * 
 * @param data Pointer to structure to receive GPS data
 * @return true if data is valid
 */
bool gps_get_data(gps_data_t *data)
{
    // TODO: Implement thread-safe GPS data retrieval
    (void)data;
    return false;
}

/**
 * @brief Check if GPS has valid fix
 * 
 * @return true if GPS fix is valid
 */
bool gps_has_fix(void)
{
    // TODO: Implement GPS fix check
    return false;
}

/**
 * @brief Get number of satellites in view
 * 
 * @return Number of satellites
 */
uint8_t gps_get_satellites(void)
{
    // TODO: Implement satellite count retrieval
    return 0;
}

/**
 * @brief Deinitialize GPS module
 */
void gps_deinit(void)
{
    ESP_LOGI(TAG, "GPS module deinitialized (stub)");
    // TODO: Clean up UART and task resources
}