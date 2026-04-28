/**
 * @file app_control.c
 * @brief Application control loop for balancing robot.
 * 
 * This file contains the main PID control loop, sensor reading,
 * and motor control dispatching for a self-balancing robot.
 */

#include "main.h"
#include "app_control.h"

#include "app_config.h"
#include "app_imu.h"
#include "app_motor.h"
#include "app_uart.h"

#include <math.h>
#include <stdio.h>

volatile float shared_angle = 0.0f;
volatile float shared_pid_output = 0.0f;
volatile uint8_t display_flag = 0;
volatile uint8_t shared_motor_cutoff = 0;

static float rotation_start_angle = 0.0f;
static uint8_t rotate_clockwise = 1;
static uint8_t in_rotation = 0;
static int rotation_cooldown = 0;
static int rotation_pid_interference_counter = 0;

void Control_LoopTick(void) {
    static float tilt_angle = 0.0f;
    static float integral = 0.0f;
    static float prev_error = 0.0f;
    static int counter = 0;
    static uint8_t motor_cutoff_latched = 0;
    static int balance_counter = 0;

    float gx, gy, gz;
    float ax, ay, az;
    
    // Read raw and calibrated sensor data
    Gyro_ReadXYZ(&gx, &gy, &gz);
    Accel_ReadXYZ(&ax, &ay, &az);

    // Calculate accelerometer angle in degrees
    float acc_angle = atan2f(ax, az) * RAD2DEG;
    
    // Complementary filter: combine gyro data (short-term) and accel data (long-term)
    tilt_angle = ALPHA * (tilt_angle + gy * DT) + (1.0f - ALPHA) * acc_angle;

    // Safety check: Latch motor cutoff if tilt angle exceeds threshold
    if (!motor_cutoff_latched && (fabsf(tilt_angle) >= MOTOR_OFF_ANGLE_DEG)) {
        motor_cutoff_latched = 1;
    }

    // If cutoff is engaged, disable motors and reset control states
    if (motor_cutoff_latched) {
        Motor_SetSpeed(0.0f);
        integral = 0.0f;
        prev_error = 0.0f;
        balance_counter = 0;
        in_rotation = 0;
        rotation_cooldown = 0;
        rotation_pid_interference_counter = 0;

        shared_angle = tilt_angle;
        shared_pid_output = 0.0f;
        shared_motor_cutoff = 1;

        // Display update pacing
        counter++;
        if (counter >= 20) {
            display_flag = 1;
            counter = 0;
        }
        return;
    }

    // Manage rotation cooldown period
    if (USE_ROTATION && rotation_cooldown > 0) {
        rotation_cooldown--;
    }

    // Calculate PID error from setpoint
    float error = SETPOINT - tilt_angle;
    float p_term = KP * error;

    // Update integral term and apply anti-windup clamping
    integral += error * DT;
    if (integral > INTEGRAL_MAX) {
        integral = INTEGRAL_MAX;
    }
    if (integral < -INTEGRAL_MAX) {
        integral = -INTEGRAL_MAX;
    }
    float i_term = KI * integral;

    // Calculate derivative term (rate of error change)
    float derivative = (error - prev_error) / DT;
    float d_term = KD * derivative;
    prev_error = error;

    // Sum all terms for final target output
    float output = p_term + i_term + d_term;

    // Apply output limits
    if (output > PID_OUTPUT_MAX) {
        output = PID_OUTPUT_MAX;
    }
    if (output < PID_OUTPUT_MIN) {
        output = PID_OUTPUT_MIN;
    }

    // Rotation Logic
    if (USE_ROTATION) {
        // If the robot is stable, increment balance timer
        if (fabsf(tilt_angle) < BALANCE_ANGLE_THRES &&
            fabsf(output) < BALANCE_PID_OUTPUT_THRES &&
            !in_rotation && rotation_cooldown == 0) {
            balance_counter++;
        } else {
            balance_counter = 0;
        }

        // Engage rotation mode if balanced for a sufficient time
        if (balance_counter >= BALANCE_TIMEOUT_CNT) {
            in_rotation = 1;
            balance_counter = 0;
            rotation_start_angle = tilt_angle;
            rotation_cooldown = 0;
            char msg[128];
            snprintf(msg, sizeof(msg), "Starting rotation (clockwise=%d)\r\n", rotate_clockwise);
            UART_Print(msg);
        }
    } else {
        in_rotation = 0;
        balance_counter = 0;
        rotation_cooldown = 0;
        rotation_pid_interference_counter = 0;
    }

    // Execution of rotation mode
    if (in_rotation) {
        // Check if PID needs to recover balance aggressively
        if (fabsf(output) >= BALANCE_PID_OUTPUT_THRES || fabsf(tilt_angle) >= BALANCE_ANGLE_THRES) {
            rotation_pid_interference_counter++;
        } else {
            rotation_pid_interference_counter = 0;
        }

        // Cancel rotation if struggling to balance
        if (rotation_pid_interference_counter >= PID_INTERFERENCE_STOP_CNT) {
            in_rotation = 0;
            balance_counter = 0;
            rotation_pid_interference_counter = 0;
            Motor_SetSpeed(output); // Give control back to raw PID
        } else {
            // Apply single-motor rotation speeds for rotation
            float rotation_angle = fabsf(tilt_angle - rotation_start_angle);
            if (rotation_angle < MAX_ROTATION_DEG) {
                float rotation_delta = ROTATION_SPEED;
                // Rotate using only one motor per direction, other motor idle (no balance needed during pure rotation)
                if (rotate_clockwise) {
                    // Rotate clockwise: left motor forward, right motor idle
                    float motor_a = rotation_delta;
                    float motor_b = 0.0f;
                    Motor_SetDifferentialSpeed(motor_a, motor_b);
                } else {
                    // Rotate counterclockwise: right motor backward, left motor idle
                    float motor_a = 0.0f;
                    float motor_b = rotation_delta;
                    Motor_SetDifferentialSpeed(motor_a, motor_b);
                }
            } else {
                // Rotation complete: reset states and toggle direction for next rotation
                Motor_SetSpeed(0.0f);
                in_rotation = 0;
                rotate_clockwise = 1 - rotate_clockwise;
                rotation_cooldown = ROTATION_COOLDOWN_CNT;
                balance_counter = 0;
                rotation_pid_interference_counter = 0;
                char msg[128];
                snprintf(msg, sizeof(msg), "Rotation complete. Next: %s\r\n",
                         rotate_clockwise ? "clockwise" : "counterclockwise");
                UART_Print(msg);
            }
        }
    } else {
        // Standard non-rotational speed mapping
        Motor_SetSpeed(output);
    }

    // Update shared variables for display/metrics
    shared_pid_output = output;
    shared_angle = tilt_angle;
    shared_motor_cutoff = 0;

    // Display refresh pacing
    counter++;
    if (counter >= 20) {
        display_flag = 1;
        counter = 0;
    }
}