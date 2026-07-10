/**
 * @file algorithm.h
 * @brief Flight control algorithm interface.
 *
 * This module implements the primary flight control algorithms used by the
 * Flight Controller, including:
 *
 *  - Flight mode processing
 *  - Stick input normalization
 *  - Outer-loop attitude controller
 *  - Inner-loop rate controller
 *  - PID control
 *  - Control output generation
 */

#ifndef ALGORITHM_H
#define ALGORITHM_H

#ifdef __cplusplus
extern "C" {
#endif

/*==============================================================================
 * Includes
 *============================================================================*/

#include <stdint.h>
#include <stdbool.h>
#include <stdatomic.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"


/*==============================================================================
 * Control Loop Configuration
 *============================================================================*/

/**
 * @brief Outer attitude control loop frequency.
 */
#define OUTER_HZ                       100U
#define OUTER_PERIOD_MS                (1000U / OUTER_HZ)

/**
 * @brief Inner rate control loop frequency.
 */
#define INNER_HZ                       500U
#define INNER_PERIOD_US                (1000000U / INNER_HZ)


/*==============================================================================
 * Receiver Configuration
 *============================================================================*/

/**
 * @brief SBUS normalization constants.
 */
#define SBUS_MID                       992.0f
#define SBUS_HALF_RANGE                820.0f


/*==============================================================================
 * Flight Control Limits
 *============================================================================*/

/**
 * @brief Maximum commanded attitude.
 */
#define MAX_ROLL_ANGLE_DEG             45.0f
#define MAX_PITCH_ANGLE_DEG            30.0f

/**
 * @brief Maximum angular rate setpoints.
 */
#define MAX_ROLL_RATE_DPS              200.0f
#define MAX_PITCH_RATE_DPS             150.0f
#define MAX_YAW_RATE_DPS               100.0f

/**
 * @brief Horizon mode blending threshold.
 */
#define HORIZON_THRESHOLD              0.7f

/**
 * @brief Integral anti-windup limit.
 */
#define INTEGRAL_LIMIT                 200.0f

/**
 * @brief Servo output range.
 */
#define OUTPUT_MIN                     0.0f
#define OUTPUT_MID                     500.0f
#define OUTPUT_MAX                     1000.0f


/*==============================================================================
 * Task Configuration
 *============================================================================*/

/**
 * @brief FreeRTOS task configuration.
 */
#define INNER_TASK_STACK               STACK_ALG_INNER
#define INNER_TASK_PRIORITY            TASK_ALG_INNER_PRIORITY

#define OUTER_TASK_STACK               STACK_ALG_OUTER
#define OUTER_TASK_PRIORITY            TASK_ALG_OUTER_PRIORITY


/*==============================================================================
 * Type Definitions
 *============================================================================*/

/**
 * @brief PID controller state.
 */
typedef struct
{
    float Kp;
    float Ki;
    float Kd;

    float integral;
    float prev_error;

    float integral_limit;

} fc_pid_t;


/**
 * @brief Flight control setpoints.
 *
 * Shared between the outer-loop and inner-loop controllers.
 * Atomic variables are used to ensure safe access between tasks.
 */
typedef struct
{
    _Atomic float roll_setpoint;
    _Atomic float pitch_setpoint;
    _Atomic float yaw_setpoint;

    /**
     * @brief Normalized throttle command.
     *
     * Range:
     *      0.0 ~ 1.0
     */
    _Atomic float throttle;

} flight_setpoint_t;


/**
 * @brief Auxiliary control channels.
 */
typedef struct
{
    _Atomic uint16_t aux1;
    _Atomic uint16_t aux2;
    _Atomic uint16_t aux3;
    _Atomic uint16_t aux4;
    _Atomic uint16_t aux5;
    _Atomic uint16_t aux6;
    _Atomic uint16_t aux7;
    _Atomic uint16_t aux8;
    _Atomic uint16_t aux9;
    _Atomic uint16_t aux10;

} aux_channel_t;


/*==============================================================================
 * Public API
 *============================================================================*/

/**
 * @brief Initialize the flight control algorithm.
 *
 * This function creates the control tasks and initializes all
 * algorithm-related resources.
 *
 * @return true if initialization succeeds.
 * @return false otherwise.
 */
bool algorithm_init(void);


/**
 * @brief Set the SBUS input queue.
 *
 * The algorithm module receives decoded SBUS frames from this queue.
 *
 * @param q Queue handle.
 */
void algorithm_set_sbus_queue(QueueHandle_t q);


/**
 * @brief Set the control output queue.
 *
 * The generated actuator commands are written to this queue.
 *
 * @param q Queue handle.
 */
void algorithm_set_ctrl_queue(QueueHandle_t q);


#ifdef __cplusplus
}
#endif

#endif /* ALGORITHM_H */