/*
 * example.h
 *
 *  Created on: Feb 4, 2025
 *      Author: Kade
 */

#ifndef INC_EXAMPLE_H_
#define INC_EXAMPLE_H_

#include <stdint.h>
#include "../../Drivers/MPU6050-Driver/MPU6050.h"

#define HAL_I2C_TIMEOUT (100)

/**
 * Wrapper around HAL_I2C_Mem_Write. 
 * TODO: Convert to Macro, include return
 */
int MPU6050_REG_WRITE_STM32(uint16_t regAddr, uint8_t regValue);

/**
 * Wrapper around HAL_I2C_Mem_Read
 * //todo add return
 */
int MPU6050_REG_READ_STM32(uint16_t regAddr, uint8_t* valAddr);

int MPU6050_BURST_READ_STM32(uint16_t regAddr, uint8_t* data, uint16_t bytes);

/**
 * Periodically checks the fifo count. Uses readPeriod, sampleRate, and
 * numAxes to determine if the count matches the expected count.
 * @param readPeriodMs: How often to check the fifo count
 * @param sampleRate: rate at which an axis is written to the fifo
 * @param numAxes: Number of axis being written to the fifo (gyro and accel each have 3)
 */
void fifo_count_test(uint16_t readPeriodMs, uint16_t sampleRate, uint8_t numAxes, MPU6050_BURST_READ_TYPE burstRead, MPU6050_REG_READ_TYPE readReg);

/**
 * Periodically read the fifo. Convert raw data into gyro and accel readings
 * and print
 * Note: Expects the fifo to be collecting data from all 6 imu axes
 * @param readPeriodMs: How often to read the fifo
 */
void read_fifo_test(uint16_t readPeriodMs, MPU6050_BURST_READ_TYPE burstRead, MPU6050_REG_READ_TYPE readReg);


#endif /* INC_EXAMPLE_H_ */
