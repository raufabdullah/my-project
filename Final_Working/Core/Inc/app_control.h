#ifndef APP_CONTROL_H
#define APP_CONTROL_H

#include <stdint.h>

/**
 * @file app_control.h
 * @brief Robot control loop definition and shared variables.
 */

// Shared variables for telemetry logging and state tracking
extern volatile float shared_angle;         ///< Latest calculated pitch angle
extern volatile float shared_pid_output;    ///< Latest computed PID output
extern volatile uint8_t display_flag;       ///< Flag to notify main loop to print telemetry
extern volatile uint8_t shared_motor_cutoff;///< Flag indicating if motors are cut off due to large tilt

/**
 * @brief High-frequency control loop tick function.
 *        Reads IMU, calculates complementary filter angle, runs PID, 
 *        and updates motor speeds. Usually called inside a timer ISR.
 */
void Control_LoopTick(void);

#endif