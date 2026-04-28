#ifndef APP_MOTOR_H
#define APP_MOTOR_H

/**
 * @file app_motor.h
 * @brief Interfaces for motor control using PWM.
 */

/**
 * @brief Apply uniform speed across left and right motors.
 * @param pid_output PWM control value (bounded by PWM period).
 */
void Motor_SetSpeed(float pid_output);

/**
 * @brief Apply differential speeds across motors.
 * @param motor_a_speed Speed for the first motor (left/right).
 * @param motor_b_speed Speed for the second motor (right/left).
 */
void Motor_SetDifferentialSpeed(float motor_a_speed, float motor_b_speed);

#endif