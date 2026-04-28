#ifndef APP_IMU_H
#define APP_IMU_H

#include <stdint.h>
#include "app_config.h"

/**
 * @file app_imu.h
 * @brief Interfaces for IMU data acquisition and calibration (Gyro & Accel).
 */

/**
 * @brief Initializes the L3GD20 Gyroscope over SPI.
 */
void Gyro_Init(void);

/**
 * @brief Takes samples to compute steady-state gyro zero-rate offsets.
 */
void Gyro_Calibration(void);

/**
 * @brief Reads the angular rates (X, Y, Z) in dps from the Gyroscope.
 * @param gx Pointer to store X-axis angular rate.
 * @param gy Pointer to store Y-axis angular rate.
 * @param gz Pointer to store Z-axis angular rate.
 */
void Gyro_ReadXYZ(float *gx, float *gy, float *gz);

/**
 * @brief Low-level SPI read from the Gyroscope.
 * @param reg Register address to read.
 * @return The register value read.
 */
uint8_t Gyro_ReadReg(uint8_t reg);

/**
 * @brief Initializes the LSM303DLHC Accelerometer over I2C.
 */
void Accel_Init(void);

/**
 * @brief Takes samples to compute steady-state accelerometer offsets.
 */
void Accel_Calibration(void);

/**
 * @brief Reads the accelerations (X, Y, Z) in mg from the Accelerometer.
 * @param ax Pointer to store X-axis acceleration.
 * @param ay Pointer to store Y-axis acceleration.
 * @param az Pointer to store Z-axis acceleration.
 */
void Accel_ReadXYZ(float *ax, float *ay, float *az);

/**
 * @brief Gets a pointer to the current gyro calibration data.
 * @return Constant pointer to Gyro_Data struct.
 */
const Gyro_Data *Gyro_GetCalibration(void);

/**
 * @brief Gets a pointer to the current accelerometer calibration data.
 * @return Constant pointer to Accel_Data struct.
 */
const Accel_Data *Accel_GetCalibration(void);

#endif