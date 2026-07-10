/**
 * @file imu.h
 * @brief Inertial Measurement Unit (IMU) interface.
 *
 * This module provides initialization, sensor calibration, raw sensor
 * acquisition, and attitude estimation using the BMI160 IMU together
 * with the Madgwick AHRS algorithm.
 */

#ifndef IMU_H
#define IMU_H

#ifdef __cplusplus
extern "C" {
#endif

/*==============================================================================
 * Includes
 *============================================================================*/

#include <stdbool.h>

#include "bmi160.h"


/*==============================================================================
 * Type Definitions
 *============================================================================*/

/**
 * @brief Estimated attitude in Euler angles.
 *
 * All angles are expressed in degrees.
 */
typedef struct
{
    float roll;
    float pitch;
    float yaw;
} imu_attitude_t;


/**
 * @brief Raw IMU sensor measurements.
 *
 * Gyroscope units:
 *      degrees/second
 *
 * Accelerometer units:
 *      g
 */
typedef struct
{
    float gx;
    float gy;
    float gz;

    float ax;
    float ay;
    float az;
} imu_raw_data_t;


/**
 * @brief IMU calibration parameters.
 */
typedef struct
{
    float bias_x;
    float bias_y;
    float bias_z;

    /**
     * @brief Madgwick filter gain.
     */
    float beta;

} imu_calib_data_t;


/*==============================================================================
 * Public API
 *============================================================================*/

/**
 * @brief Initialize the IMU module.
 *
 * This function performs:
 *  - I2C initialization
 *  - BMI160 initialization
 *  - Sensor calibration
 *  - Madgwick AHRS initialization
 *
 * @return true if initialization succeeds.
 * @return false if initialization fails.
 */
bool imu_init(void);


/**
 * @brief IMU processing task.
 *
 * This task periodically:
 *  - Reads raw BMI160 data
 *  - Applies gyro bias correction
 *  - Updates the Madgwick AHRS filter
 *  - Stores the latest attitude estimate
 *
 * @param pvParameters FreeRTOS task parameter.
 */
void imu_task(void *pvParameters);


/**
 * @brief Get the latest attitude estimate.
 *
 * @param[out] roll  Roll angle (degrees)
 * @param[out] pitch Pitch angle (degrees)
 * @param[out] yaw   Yaw angle (degrees)
 */
void imu_get_euler_deg(float *roll,
                       float *pitch,
                       float *yaw);


/**
 * @brief Get the latest raw IMU measurements.
 *
 * @param[out] data Pointer to the destination structure.
 */
void imu_get_raw_data(imu_raw_data_t *data);


/**
 * @brief Get the latest gyroscope measurements.
 *
 * @param[out] gx X-axis angular rate (deg/s)
 * @param[out] gy Y-axis angular rate (deg/s)
 * @param[out] gz Z-axis angular rate (deg/s)
 */
void imu_get_gyro_values(float *gx,
                         float *gy,
                         float *gz);


/**
 * @brief Get the latest accelerometer measurements.
 *
 * @param[out] ax X-axis acceleration (g)
 * @param[out] ay Y-axis acceleration (g)
 * @param[out] az Z-axis acceleration (g)
 */
void imu_get_accel_values(float *ax,
                          float *ay,
                          float *az);


#ifdef __cplusplus
}
#endif

#endif /* IMU_H */