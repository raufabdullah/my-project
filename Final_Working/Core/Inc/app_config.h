/**
 * @file app_config.h
 * @brief Configuration settings for the balancing robot.
 * 
 * This header defines physical parameters, sensor registers, 
 * PID gains, limits, and runtime switches.
 */

#ifndef APP_CONFIG_H
#define APP_CONFIG_H

typedef struct {
    float offset_x;
    float offset_y;
    float offset_z;
} Gyro_Data;

typedef struct {
    float offset_x;
    float offset_y;
    float offset_z;
} Accel_Data;

// ---- LSM303DLHC Accelerometer (I2C1) ----
#define ACC_I2C_ADDR 0x32
#define ACC_CTRL_REG1_A 0x20
#define ACC_CTRL_REG4_A 0x23
#define ACC_OUT_X_L_A 0x28

// ---- L3GD20 Gyroscope (SPI1) ----
#define GYRO_WHO_AM_I 0x0F
#define GYRO_CTRL_REG1 0x20
#define GYRO_CTRL_REG4 0x23
#define GYRO_OUT_X_L 0x28

// Calibration samples
#define GYRO_CALIB_SAMPLES 20
#define ACCEL_CALIB_SAMPLES 20

// Calibration usage switches (1 = use calibrated input, 0 = use raw input)
#define USE_GYRO_CALIB 1
#define USE_ACCEL_CALIB 0

// Sensitivities
#define GYRO_SENSITIVITY 8.75f
#define ACC_SENSITIVITY 1.0f

// Control loop
#define DT 0.005f           // Control loop period (5ms)
#define ALPHA 0.98f         // Complementary filter weight for gyro
#define RAD2DEG 57.2957795f // Radians to degrees conversion factor

// ---- PID Gains ----
#define KP 35.0f
#define KI 0.20f
#define KD 0.25f

// PID limits
#define PID_OUTPUT_MAX 999.0f
#define PID_OUTPUT_MIN -999.0f
#define INTEGRAL_MAX 500.0f

// Setpoint: desired angle (0 = upright)
#define SETPOINT 0.0f

// Safety cutoff: once |tilt angle| reaches this value, motors are disabled
#define MOTOR_OFF_ANGLE_DEG 30.0f

// PWM period (must match TIM2 Period)
#define PWM_PERIOD 999

// Rotation switch (1 = enable auto-rotation, 0 = disable rotation feature)
#define USE_ROTATION 1

// Rotation feature timings and thresholds
#define BALANCE_TIME_MS 1000
#define BALANCE_TIMEOUT_CNT (BALANCE_TIME_MS / 5)
#define BALANCE_ANGLE_THRES 3.0f
#define BALANCE_PID_OUTPUT_THRES 25.0f
#define PID_INTERFERENCE_STOP_MS 200
#define PID_INTERFERENCE_STOP_CNT (PID_INTERFERENCE_STOP_MS / 5)
#define MAX_ROTATION_DEG 2.0f
#define ROTATION_SPEED 60.0f
#define ROTATION_COOLDOWN_MS 2000
#define ROTATION_COOLDOWN_CNT (ROTATION_COOLDOWN_MS / 5)

#endif