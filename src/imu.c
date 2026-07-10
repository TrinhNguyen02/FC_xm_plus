#include <stdio.h>
#include <string.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"

#include "imu.h"
#include "config.h"
#include "MadgwickAHRS.h"

#include "nvs.h"

#include "esp_log.h"
#include "esp_timer.h" 
#include "esp_private/esp_clk.h"
#include "esp_system.h"
#include "esp_heap_caps.h"


/*================ Constants & Macros =================*/
#define I2C_PORT            I2C_NUM_0
#define I2C_SDA             GPIO_NUM_8
#define I2C_SCL             GPIO_NUM_9
#define I2C_FREQ_HZ         400000
#define BMI160_ADDR         BMI160_I2C_ADDR

#define RAD_TO_DEG          (180.0f / M_PI)
#define DEG_TO_RAD          (M_PI / 180.0f)
#define GYRO_SENS_2000DPS   16.4f
#define ACCEL_SENS_16G      2048.0f

/*================ Static Variables =================*/
static const char *TAG = "imu";

static i2c_master_bus_handle_t s_bus_handle = NULL;
static i2c_master_dev_handle_t s_bmi_dev_handle = NULL;
static struct bmi160_dev s_sensor;
static bool s_imu_initialized = false;

static imu_raw_data_t s_imu_raw_data = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
static imu_attitude_t s_imu_euler_deg = {0.0f, 0.0f, 0.0f};

static float gyro_bias_x = 0.0f, gyro_bias_y = 0.0f, gyro_bias_z = 0.0f;
static float max_noise_x = 0.0f, max_noise_y = 0.0f, max_noise_z = 0.0f;
static const int calib_samples = 2000;
static imu_calib_data_t s_calib_data;

static struct bmi160_sensor_data accel;
static struct bmi160_sensor_data gyro;

/*================ Bosch I2C Callbacks =================*/
static int8_t bmi160_i2c_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
    (void)dev_addr;
    esp_err_t ret = i2c_master_transmit_receive(s_bmi_dev_handle, &reg_addr, 1, data, len, -1);
    return (ret == ESP_OK) ? BMI160_OK : BMI160_E_COM_FAIL;
}

static int8_t bmi160_i2c_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
    (void)dev_addr;
    uint8_t tx[32];
    if (len > (sizeof(tx) - 1))
    {
        return BMI160_E_COM_FAIL;
    }
    tx[0] = reg_addr;
    memcpy(&tx[1], data, len);
    esp_err_t ret = i2c_master_transmit(s_bmi_dev_handle, tx, len + 1, -1);
    return (ret == ESP_OK) ? BMI160_OK : BMI160_E_COM_FAIL;
}

static void bmi160_delay_ms(uint32_t ms)
{
    vTaskDelay(pdMS_TO_TICKS(ms));
}

/*================ Helper Functions =================*/
static float max3(float a, float b, float c)
{
    float max = a;
    if (b > max)
    {
        max = b;
    }
    if (c > max)
    {
        max = c;
    }
    return max;
}
// void print_ram_usage() {
//     // Lấy thông tin về Heap tổng quát
//     uint32_t free_heap = esp_get_free_heap_size();
//     uint32_t min_free_heap = esp_get_minimum_free_heap_size();
    
//     ESP_LOGI("RAM_MONITOR", "Free Heap: %lu bytes", free_heap);
//     ESP_LOGI("RAM_MONITOR", "Min Free Heap ever: %lu bytes", min_free_heap);

//     // Kiểm tra chi tiết heap nội bộ (Internal) và PSRAM (nếu có)
//     ESP_LOGI("RAM_MONITOR", "Internal Free Heap: %lu bytes", esp_get_free_internal_heap_size());
// }
/*======================= NVS Operations ==========================*/
/* Save calibration data to NVS */
static bool save_imu_calibration(imu_calib_data_t *data) 
{
    nvs_handle_t handle;
    if (nvs_open("imu_cfg", NVS_READWRITE, &handle) != ESP_OK) 
    {
        return false;
    }

    if (nvs_set_blob(handle, "calib", data, sizeof(imu_calib_data_t)) != ESP_OK) 
    {
        nvs_close(handle);
        return false;
    }

    nvs_commit(handle);
    nvs_close(handle);
    return true;
}

/* Load calibration data from NVS */
static bool load_imu_calibration(imu_calib_data_t *data) 
{
    nvs_handle_t handle;
    if (nvs_open("imu_cfg", NVS_READONLY, &handle) != ESP_OK) 
    {
        return false;
    }

    size_t size = sizeof(imu_calib_data_t);
    esp_err_t err = nvs_get_blob(handle, "calib", data, &size);
    nvs_close(handle);
    
    return (err == ESP_OK);
}

/*================ Initialization Functions =================*/
static bool i2c_master_init(void)
{
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = I2C_PORT,
        .sda_io_num = I2C_SDA,
        .scl_io_num = I2C_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    if (i2c_new_master_bus(&bus_cfg, &s_bus_handle) != ESP_OK)
    {
        return false;
    }

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = BMI160_ADDR,
        .scl_speed_hz = I2C_FREQ_HZ,
    };

    return (i2c_master_bus_add_device(s_bus_handle, &dev_cfg, &s_bmi_dev_handle) == ESP_OK);
}

static bool bmi160_sensor_init(void)
{
    memset(&s_sensor, 0, sizeof(s_sensor));
    s_sensor.id = BMI160_ADDR;
    s_sensor.intf = BMI160_I2C_INTF;
    s_sensor.read = bmi160_i2c_read;
    s_sensor.write = bmi160_i2c_write;
    s_sensor.delay_ms = bmi160_delay_ms;

    if (bmi160_init(&s_sensor) != BMI160_OK)
    {
        return false;
    }

    s_sensor.accel_cfg.odr = BMI160_ACCEL_ODR_1600HZ;
    s_sensor.accel_cfg.range = BMI160_ACCEL_RANGE_16G;
    s_sensor.accel_cfg.bw = BMI160_ACCEL_BW_NORMAL_AVG4;
    s_sensor.accel_cfg.power = BMI160_ACCEL_NORMAL_MODE;

    s_sensor.gyro_cfg.odr = BMI160_GYRO_ODR_3200HZ;
    s_sensor.gyro_cfg.range = BMI160_GYRO_RANGE_2000_DPS;
    s_sensor.gyro_cfg.bw = BMI160_GYRO_BW_NORMAL_MODE;
    s_sensor.gyro_cfg.power = BMI160_GYRO_NORMAL_MODE;

    bmi160_set_sens_conf(&s_sensor);
    vTaskDelay(pdMS_TO_TICKS(20));

    uint8_t cmd_accel = 0x11;
    s_sensor.write(s_sensor.id, 0x7E, &cmd_accel, 1);
    vTaskDelay(pdMS_TO_TICKS(20));

    uint8_t cmd_gyro = 0x15;
    s_sensor.write(s_sensor.id, 0x7E, &cmd_gyro, 1);
    vTaskDelay(pdMS_TO_TICKS(100));
    
    return true;
}

bool imu_init(void)
{
    ESP_LOGI(TAG, "Initializing I2C and BMI160 sensor");

    if (s_imu_initialized)
    {
        return true;
    }

    if (!i2c_master_init())
    {
        return false;
    }

    if (!bmi160_sensor_init())
    {
        return false;
    }

/* Try to load existing calibration from NVS */
    if (load_imu_calibration(&s_calib_data)) 
    {
        gyro_bias_x = s_calib_data.bias_x;
        gyro_bias_y = s_calib_data.bias_y;
        gyro_bias_z = s_calib_data.bias_z;
        beta = s_calib_data.beta;
        ESP_LOGI(TAG, "Calibration loaded from NVS. Skipping 4s loop.");
    }
    else 
    {
        /* Perform full calibration if no NVS data exists */
        beta = 1.0f;
        ESP_LOGI(TAG, "Calibrating Gyro, please keep the FC steady...");
        
        for (int i = 0; i < calib_samples; i++) 
        {
            bmi160_get_sensor_data(BMI160_GYRO_SEL, NULL, &gyro, &s_sensor);
            gyro_bias_x += ((float)gyro.x / GYRO_SENS_2000DPS) * DEG_TO_RAD;
            gyro_bias_y += ((float)gyro.y / GYRO_SENS_2000DPS) * DEG_TO_RAD;
            gyro_bias_z += ((float)gyro.z / GYRO_SENS_2000DPS) * DEG_TO_RAD;
            vTaskDelay(pdMS_TO_TICKS(1));
        }
        
        gyro_bias_x /= calib_samples;
        gyro_bias_y /= calib_samples;
        gyro_bias_z /= calib_samples;
            
        for (int i = 0; i < calib_samples; i++) 
        {
            bmi160_get_sensor_data(BMI160_GYRO_SEL, NULL, &gyro, &s_sensor);
            float gx_clean = (((float)gyro.x / GYRO_SENS_2000DPS) * DEG_TO_RAD) - gyro_bias_x;
            float gy_clean = (((float)gyro.y / GYRO_SENS_2000DPS) * DEG_TO_RAD) - gyro_bias_y;
            float gz_clean = (((float)gyro.z / GYRO_SENS_2000DPS) * DEG_TO_RAD) - gyro_bias_z;
            
            if (fabsf(gx_clean) > max_noise_x) max_noise_x = fabsf(gx_clean);
            if (fabsf(gy_clean) > max_noise_y) max_noise_y = fabsf(gy_clean);
            if (fabsf(gz_clean) > max_noise_z) max_noise_z = fabsf(gz_clean);
            vTaskDelay(pdMS_TO_TICKS(1));
        }
        
        beta = 0.866025f * max3(max_noise_x, max_noise_y, max_noise_z);
        
        /* Save new calibration to NVS */
        s_calib_data.bias_x = gyro_bias_x;
        s_calib_data.bias_y = gyro_bias_y;
        s_calib_data.bias_z = gyro_bias_z;
        s_calib_data.beta   = beta;
        save_imu_calibration(&s_calib_data);
        
        ESP_LOGI(TAG, "Calibration complete. Beta: %f", beta);
    }
    ESP_LOGI(TAG, "IMU initialized successfully.");

    if (xTaskCreate(imu_task, "imu_task", STACK_IMU, NULL, 5, NULL) != pdPASS)
    {
        ESP_LOGE(TAG, "Failed to create IMU task");
        return false;
    }

    s_imu_initialized = true;
    return true;
}

/*================ IMU Task =================*/
void imu_task(void *pvParameters)
{
    ESP_LOGI(TAG, "IMU task started");

    uint64_t last_time = esp_timer_get_time();
    uint32_t print_counter = 0;

    sampleFreq = 1000.0f;

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(1);

    while (1)
    {
        bmi160_get_sensor_data(BMI160_ACCEL_SEL | BMI160_GYRO_SEL, &accel, &gyro, &s_sensor);

        uint64_t current_time = esp_timer_get_time();
        float dt = (float)(current_time - last_time) / 1000000.0f;
        last_time = current_time;

        if (dt > 0.0f)
        {
            sampleFreq = 1.0f / dt;
        }

        s_imu_raw_data.ax = (float)accel.x / ACCEL_SENS_16G;
        s_imu_raw_data.ay = (float)accel.y / ACCEL_SENS_16G;
        s_imu_raw_data.az = (float)accel.z / ACCEL_SENS_16G;

        s_imu_raw_data.gx = (((float)gyro.x / GYRO_SENS_2000DPS) * DEG_TO_RAD) - gyro_bias_x;
        s_imu_raw_data.gy = (((float)gyro.y / GYRO_SENS_2000DPS) * DEG_TO_RAD) - gyro_bias_y;
        s_imu_raw_data.gz = (((float)gyro.z / GYRO_SENS_2000DPS) * DEG_TO_RAD) - gyro_bias_z;

        if (fabs(s_imu_raw_data.gz) < 0.005f)
        {
            s_imu_raw_data.gz = 0.0f;
        }

        MadgwickAHRSupdateIMU(s_imu_raw_data.gx, s_imu_raw_data.gy, s_imu_raw_data.gz, 
                            s_imu_raw_data.ax, s_imu_raw_data.ay, s_imu_raw_data.az);

        s_imu_euler_deg.roll  = atan2f(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2)) * RAD_TO_DEG;
        s_imu_euler_deg.pitch = asinf(2.0f * (q0 * q2 - q3 * q1)) * RAD_TO_DEG;
        s_imu_euler_deg.yaw   = atan2f(2.0f * (q0 * q3 + q1 * q2), 1.0f - 2.0f * (q2 * q2 + q3 * q3)) * RAD_TO_DEG;
 
        print_counter++;
        if (print_counter >= 2000)
        {
            ESP_LOGI(TAG, "Roll: %6.1f | Pitch: %6.1f | Yaw: %6.1f | Freq: %.1f Hz", 
                s_imu_euler_deg.roll, s_imu_euler_deg.pitch, s_imu_euler_deg.yaw, sampleFreq);
            // print_ram_usage();
            print_counter = 0;
        }

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

/*================ API Functions =================*/

void imu_get_euler_deg(float *roll, float *pitch, float *yaw)
{
    if (roll)  *roll  = s_imu_euler_deg.roll;
    if (pitch) *pitch = s_imu_euler_deg.pitch;
    if (yaw)   *yaw   = s_imu_euler_deg.yaw;
}

void imu_get_raw_data(imu_raw_data_t *data)
{
    if (data)
    {
        memcpy(data, &s_imu_raw_data, sizeof(imu_raw_data_t));
    }
}

void imu_get_gyro_values(float *gx, float *gy, float *gz)
{
    if (gx) *gx = s_imu_raw_data.gx;
    if (gy) *gy = s_imu_raw_data.gy;
    if (gz) *gz = s_imu_raw_data.gz;
}

void imu_get_accel_values(float *ax, float *ay, float *az)
{
    if (ax) *ax = s_imu_raw_data.ax;
    if (ay) *ay = s_imu_raw_data.ay;
    if (az) *az = s_imu_raw_data.az;
}
