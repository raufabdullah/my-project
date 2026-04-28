/**
 * @file app_imu.c
 * @brief IMU configuration and data reading.
 * 
 * This file handles setup, calibration, and reading of the accelerometer (LSM303DLHC)
 * and gyroscope (L3GD20) mounted on the STM32F3 balancing robot over I2C and SPI.
 */

#include "main.h"
#include "app_imu.h"

extern SPI_HandleTypeDef hspi1;
extern I2C_HandleTypeDef hi2c1;

// Offsets used to calibrate sensor data
static Gyro_Data s_gyro_cal = {0.0f, 0.0f, 0.0f};
static Accel_Data s_accel_cal = {0.0f, 0.0f, 0.0f};

/**
 * @brief Helper to write to a gyro register over SPI
 */
static void Gyro_WriteReg(uint8_t reg, uint8_t val) {
    uint8_t txData[2] = {reg, val};
    HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, txData, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_SET);
}

/**
 * @brief Helper to read from a gyro register over SPI
 */
uint8_t Gyro_ReadReg(uint8_t reg) {
    uint8_t txData = reg | 0x80; // Set MSB for Read Operation
    uint8_t rxData = 0;
    HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, &txData, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive(&hspi1, &rxData, 1, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_SET);
    return rxData;
}

/**
 * @brief Initialize Gyroscope configuration
 */
void Gyro_Init(void) {
    // Make sure CS is high initially
    HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_SET);
    HAL_Delay(10);
    
    // Enable X, Y, Z axes and set proper output data rate
    Gyro_WriteReg(GYRO_CTRL_REG1, 0x6F); 
    
    // Set sensitivity
    Gyro_WriteReg(GYRO_CTRL_REG4, 0x80);
}

/**
 * @brief Read uncalibrated gyro raw values 
 */
static void Gyro_ReadRawXYZ(float *gx, float *gy, float *gz) {
    // Read starting from OUT_X_L with auto-increment
    uint8_t txAddr = GYRO_OUT_X_L | 0x80 | 0x40;
    uint8_t buf[6];

    HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, &txAddr, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive(&hspi1, buf, 6, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_SET);

    // Combine bytes into signed 16-bit integers
    int16_t raw_x = (int16_t)(buf[1] << 8 | buf[0]);
    int16_t raw_y = (int16_t)(buf[3] << 8 | buf[2]);
    int16_t raw_z = (int16_t)(buf[5] << 8 | buf[4]);

    // Apply sensitivity coefficient to get degrees per second
    *gx = raw_x * GYRO_SENSITIVITY * 0.001f;
    *gy = raw_y * GYRO_SENSITIVITY * 0.001f;
    *gz = raw_z * GYRO_SENSITIVITY * 0.001f;
}

/**
 * @brief Calibrate gyro by taking multiple samples at a standstill
 */
void Gyro_Calibration(void) {
    float sum_x = 0.0f;
    float sum_y = 0.0f;
    float sum_z = 0.0f;
    float gx, gy, gz;

    // Collect multiple samples and accumulate
    for (int i = 0; i < GYRO_CALIB_SAMPLES; i++) {
        Gyro_ReadRawXYZ(&gx, &gy, &gz);
        sum_x += gx;
        sum_y += gy;
        sum_z += gz;
        HAL_Delay(2);
    }

    // Average the samples to find the 0 dc-offset
    s_gyro_cal.offset_x = sum_x / (float)GYRO_CALIB_SAMPLES;
    s_gyro_cal.offset_y = sum_y / (float)GYRO_CALIB_SAMPLES;
    s_gyro_cal.offset_z = sum_z / (float)GYRO_CALIB_SAMPLES;
}

/**
 * @brief Retrieve calibrated or raw gyro data based on configuration
 */
void Gyro_ReadXYZ(float *gx, float *gy, float *gz) {
    float raw_gx, raw_gy, raw_gz;
    Gyro_ReadRawXYZ(&raw_gx, &raw_gy, &raw_gz);

    if (USE_GYRO_CALIB) {
        *gx = raw_gx - s_gyro_cal.offset_x;
        *gy = raw_gy - s_gyro_cal.offset_y;
        *gz = raw_gz - s_gyro_cal.offset_z;
    } else {
        *gx = raw_gx;
        *gy = raw_gy;
        *gz = raw_gz;
    }
}

/**
 * @brief Initialize Accelerometer configuration over I2C
 */
void Accel_Init(void) {
    uint8_t data;
    
    // Enable axes and set data rate
    data = 0x57;
    HAL_I2C_Mem_Write(&hi2c1, ACC_I2C_ADDR, ACC_CTRL_REG1_A,
                      I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);
    
    // Setup high resolution output
    data = 0x88;
    HAL_I2C_Mem_Write(&hi2c1, ACC_I2C_ADDR, ACC_CTRL_REG4_A,
                      I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);
}

/**
 * @brief Read uncalibrated accelerometer data
 */
static void Accel_ReadRawXYZ(float *ax, float *ay, float *az) {
    uint8_t buf[6];
    
    // Read 6 bytes starting from X_L with MSB set for auto-increment
    HAL_I2C_Mem_Read(&hi2c1, ACC_I2C_ADDR, ACC_OUT_X_L_A | 0x80,
                     I2C_MEMADD_SIZE_8BIT, buf, 6, HAL_MAX_DELAY);

    // Combine 12-bit accelerometer data into 16 bit integers then shift right by 4
    int16_t raw_x = (int16_t)(buf[1] << 8 | buf[0]) >> 4;
    int16_t raw_y = (int16_t)(buf[3] << 8 | buf[2]) >> 4;
    int16_t raw_z = (int16_t)(buf[5] << 8 | buf[4]) >> 4;

    // Apply sensitivity coefficient
    *ax = raw_x * ACC_SENSITIVITY;
    *ay = raw_y * ACC_SENSITIVITY;
    *az = raw_z * ACC_SENSITIVITY;
}

/**
 * @brief Calibrate accelerometer by taking multiple samples on a flat plane
 */
void Accel_Calibration(void) {
    float sum_x = 0.0f;
    float sum_y = 0.0f;
    float sum_z = 0.0f;
    float ax, ay, az;

    for (int i = 0; i < ACCEL_CALIB_SAMPLES; i++) {
        Accel_ReadRawXYZ(&ax, &ay, &az);
        sum_x += ax;
        sum_y += ay;
        sum_z += az;
        HAL_Delay(2);
    }

    s_accel_cal.offset_x = sum_x / (float)ACCEL_CALIB_SAMPLES;
    s_accel_cal.offset_y = sum_y / (float)ACCEL_CALIB_SAMPLES;
    s_accel_cal.offset_z = sum_z / (float)ACCEL_CALIB_SAMPLES;
}

/**
 * @brief Retrieve calibrated or raw accel data based on configuration
 */
void Accel_ReadXYZ(float *ax, float *ay, float *az) {
    float raw_ax, raw_ay, raw_az;
    Accel_ReadRawXYZ(&raw_ax, &raw_ay, &raw_az);

    if (USE_ACCEL_CALIB) {
        *ax = raw_ax - s_accel_cal.offset_x;
        *ay = raw_ay - s_accel_cal.offset_y;
    } else {
        *ax = raw_ax;
        *ay = raw_ay;
    }

    // Keep Z as gravity reference for atan2f(ax, az), typically Z rests at near +1/-1g 
    *az = raw_az;
}

/**
 * @brief Gets pointer to the gyro calibration data
 */
const Gyro_Data *Gyro_GetCalibration(void) {
    return &s_gyro_cal;
}

/**
 * @brief Gets pointer to the accel calibration data
 */
const Accel_Data *Accel_GetCalibration(void) {
    return &s_accel_cal;
}