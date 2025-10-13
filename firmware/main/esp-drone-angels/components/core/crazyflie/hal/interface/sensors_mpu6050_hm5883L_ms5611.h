/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * ESP-Drone Firmware
 *
 * Copyright 2019-2020  Espressif Systems (Shanghai)
 * Copyright (C) 2018 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 */

#ifndef __SENSORS_MPU9250_LPS25H_H__
#define __SENSORS_MPU9250_LPS25H_H__

#include "sensors.h"

void sensorsMpu6050Hmc5883lMs5611Init(void);
bool sensorsMpu6050Hmc5883lMs5611Test(void);
bool sensorsMpu6050Hmc5883lMs5611AreCalibrated(void);
bool sensorsMpu6050Hmc5883lMs5611ManufacturingTest(void);
void sensorsMpu6050Hmc5883lMs5611Acquire(sensorData_t *sensors, const uint32_t tick);
void sensorsMpu6050Hmc5883lMs5611WaitDataReady(void);
bool sensorsMpu6050Hmc5883lMs5611ReadGyro(Axis3f *gyro);
bool sensorsMpu6050Hmc5883lMs5611ReadAcc(Axis3f *acc);
bool sensorsMpu6050Hmc5883lMs5611ReadMag(Axis3f *mag);
bool sensorsMpu6050Hmc5883lMs5611ReadBaro(baro_t *baro);
void sensorsMpu6050Hmc5883lMs5611SetAccMode(accModes accMode);


// EDIT{

/**
 * @brief Receive one raw accelerometer sample from the queue.
 *
 * Copies the latest raw accelerometer data from it's queue into @p accelRaw.
 *
 * @note This call consumes one item from the queue; call it at most once per
 *       stabilizer loop.
 *
 * @param[out] accelRaw Pointer to destination struct that receives the data.
 *
 * @retval true  A sample was available and written to @p accelRaw.
 * @retval false No sample was available; @p accelRaw is not modified.
 */
bool readAccelRawQueue(Axis3i16 *accelRaw);


/**
 * @brief Receive one raw gyro sample from the queue.
 *
 * Copies the latest raw gyro data from it's queue into @p gyroRaw.
 *
 * @note This call consumes one item from the queue; call it at most once per
 *       stabilizer loop.
 *
 * @param[out] gyroRaw Pointer to destination struct that receives the data.
 *
 * @retval true  A sample was available and written to @p gyroRaw.
 * @retval false No sample was available; @p gyroRaw is not modified.
 */
bool readGyroRawQueue(Axis3i16 *gyroRaw);

/**
 * @brief Receive one processed acceleration sample from the queue.
 *
 * Copies the latest processed acceleration data from it's queue into @p accelProcessed.
 *
 * @note This call consumes one item from the queue; call it at most once per
 *       stabilizer loop.
 *
 * @param[out] accelProcessed Pointer to destination struct that receives the data.
 *
 * @retval true  A sample was available and written to @p accelProcessed.
 * @retval false No sample was available; @p accelProcessed is not modified.
 */
bool readAccelProcessedQueue(Axis3f *accelProcessed);

/**
 * @brief Receive one processed gyro sample from the queue.
 *
 * Copies the latest processed gyro data from it's queue into @p gyroProcessed.
 *
 * @note This call consumes one item from the queue; call it at most once per
 *       stabilizer loop.
 *
 * @param[out] gyroProcessed Pointer to destination struct that receives the data.
 *
 * @retval true  A sample was available and written to @p gyroProcessed.
 * @retval false No sample was available; @p gyroProcessed is not modified.
 */
bool readGyroProcessedQueue(Axis3f *gyroProcessed);

 /**
 * @brief Receive one IMU timestamp sample from the queue.
 *
 * Copies the latest IMU timestamp data from it's queue into @p imuTimestamp.
 *
 * @note This call consumes one item from the queue; call it at most once per
 *       stabilizer loop.
 *
 * @param[out] imuTimestamp Pointer to destination struct that receives the data.
 *
 * @retval true  A sample was available and written to @p imuTimestamp.
 * @retval false No sample was available; @p imuTimestamp is not modified.
 */
bool readImuTimestampQueue(uint64_t *imuTimestamp);

//}

#endif // __SENSORS_MPU9250_LPS25H_H__