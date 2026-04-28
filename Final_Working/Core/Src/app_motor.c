#include "main.h"
#include "app_motor.h"
#include "app_config.h"

#include <math.h>

extern TIM_HandleTypeDef htim2;

/**
 * @brief Sets the speed for both motors equally based on PID output.
 * @param pid_output The calculated PID control output (positive for forward, negative for reverse).
 *                   This value is clamped to the maximum PWM period.
 */
void Motor_SetSpeed(float pid_output) {
    // Clamp the PID output to the maximum PWM period limits
    if (pid_output > PWM_PERIOD) {
        pid_output = PWM_PERIOD;
    }
    if (pid_output < -PWM_PERIOD) {
        pid_output = -PWM_PERIOD;
    }

    uint32_t pwm_val = (uint32_t)fabsf(pid_output);

    // Set motor direction based on the sign of pid_output
    if (pid_output >= 0) {
        // Forward direction
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
    } else {
        // Reverse direction
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
    }

    // Apply the computed PWM value to the timer channels
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pwm_val);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, pwm_val);
}

/**
 * @brief Sets the speed for the left and right motors independently (differential drive).
 * @param motor_a_speed The target speed for Motor A (left or right).
 * @param motor_b_speed The target speed for Motor B (left or right).
 */
void Motor_SetDifferentialSpeed(float motor_a_speed, float motor_b_speed) {
    // Clamp the speeds to the maximum PWM period limits
    if (motor_a_speed > PWM_PERIOD) {
        motor_a_speed = PWM_PERIOD;
    }
    if (motor_a_speed < -PWM_PERIOD) {
        motor_a_speed = -PWM_PERIOD;
    }
    if (motor_b_speed > PWM_PERIOD) {
        motor_b_speed = PWM_PERIOD;
    }
    if (motor_b_speed < -PWM_PERIOD) {
        motor_b_speed = -PWM_PERIOD;
    }

    uint32_t pwm_a = (uint32_t)fabsf(motor_a_speed);
    uint32_t pwm_b = (uint32_t)fabsf(motor_b_speed);

    // Set Motor A direction
    if (motor_a_speed >= 0) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
    } else {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
    }

    // Set Motor B direction
    if (motor_b_speed >= 0) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
    } else {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
    }

    // Apply the independent PWM values to the timer channels
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pwm_a);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, pwm_b);
}