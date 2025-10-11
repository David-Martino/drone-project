// I2Cdev MMC5983MA
// Based on Memsic MMC5983MA datasheet, 3/2019 (Form Rev A)
// Adapted from HMC5883L I2Cdev driver by Jeff Rowberg, available at https://github.com/jrowberg/i2cdevlib
// 11/10/2025 by Nathan Mayhew

/* ============================================
 Copyright (c) Nathan Mayhew
 Adapted to Crazyflie FW by Bitcraze
 Adapted to ESP-Drone by Dell's Angels

 Permission is hereby granted, free of charge, to any person obtaining a copy
 of this software and associated documentation files (the "Software"), to deal
 in the Software without restriction, including without limitation the rights
 to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:

 The above copyright notice and this permission notice shall be included in
 all copies or substantial portions of the Software.

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 THE SOFTWARE.
 ===============================================
 */


#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "eprintf.h"
#include "mmc5983ma.h"
#include "i2cdev.h"
#define DEBUG_MODULE "MMC5983MA"
#include "debug_cf.h"

static uint8_t devAddr;
static uint8_t buffer[6];
static uint8_t mode;
static I2C_Dev *I2Cx;
static bool isInit;

/** Power on and prepare for general usage.
 * From HMC driver: "This will prepare the magnetometer with default settings, ready for single-
 * use mode (very low power requirements)." - why would they use single-use mode?? the MPU can't trigger one?
 */
void mmc5983maInit(I2C_Dev *i2cPort)
{
    if (isInit) {
        return;
    }

    I2Cx = i2cPort;
    devAddr = MMC5983MA_ADDRESS;

    // write CNTRL_0 register
    // nothing to do, defaults are good
    
    // write CNTRL_1 register
    i2cdevWriteByte(I2Cx, devAddr, MMC5983MA_RA_CTRL1,
                    (MMC5983MA_BW_100HZ << (MMC5983MA_CR1_BW_BIT - MMC5983MA_CR1_BW_LENGTH + 1)));  // 100Hz bandwidth


    // write CNTRL_2 register
    i2cdevWriteByte(I2Cx, devAddr, MMC5983MA_RA_CTRL2,
                    (MMC5983MA_CMRATE_200HZ << (MMC5983MA_CR2_CMF_BIT - MMC5983MA_CR2_CMF_LENGTH + 1)) |
                    (1 << MMC5983MA_CR2_CMEN_BIT) |
                    (MMC5983MA_PRDSET_1 << (MMC5983MA_CR2_PRDSET_BIT - MMC5983MA_CR2_PRDSET_LENGTH + 1)) |
                    (1 << MMC5983MA_CR2_ENPRD_BIT)); // @@ 200HZ continuous sample rate, with periodic set/reset each measurement

    // write CNTRL_3 register
    // nothing to do, defaults are good

    isInit = true;
}

/** Verify the I2C connection.
 * Make sure the device is connected and responds as expected.
 * @return True if connection is valid, false otherwise
 */
bool mmc5983maTestConnection()
{
    if (i2cdevReadReg8(I2Cx, devAddr, MMC5983MA_RA_PID, 1, buffer)) {
        return (buffer[0] == 0x30);
    }

    return false;
}


// CNTRL1 register

/** Get bandwidth of the decimation filter (the inverse of the bandwidth is the duration of each measurement)
 * @return Current bandwidth (0/1/2/3 for 100/200/400/800 respectively)
 * @see MMC5983MA_BW_400HZ
 * @see MMC5983MA_RA_CTRL1
 * @see MMC5983MA_CR1_BW_BIT
 * @see MMC5983MA_CR1_BW_LENGTH
 */
uint8_t mmc5983maGetBandwidth()
{
    i2cdevReadBits(I2Cx, devAddr, MMC5983MA_RA_CTRL1, MMC5983MA_CR1_BW_BIT, MMC5983MA_CR1_BW_LENGTH, buffer);
    return buffer[0];
}
/** Set bandwidth.
 * @param averaging New bandwidth (0/1/2/3 for 100/200/400/800 respectively)
 * @see MMC5983MA_RA_CTRL1
 * @see HMC5883L_CRA_AVERAGE_BIT
 * @see MMC5983MA_CR1_BW_LENGTH
 */
void mmc5983maSetBandwidth(uint8_t bandwidth)
{
    i2cdevReadBits(I2Cx, devAddr, MMC5983MA_RA_CTRL1, MMC5983MA_CR1_BW_BIT, MMC5983MA_CR1_BW_LENGTH, bandwidth);
}

/** Get data output rate value.
 * The Table below shows all selectable output rates in continuous measurement
 * mode.
 * Value | Typical Data Output Rate (Hz)
 * ------+------------------------------
 * 0     | continuous measurement off
 * 1     | 1
 * 2     | 10
 * 3     | 20
 * 4     | 50
 * 5     | 100
 * 6     | 200
 * 7     | 1000
 *
 * @return Current rate of data output to registers
 * @see MMC5983MA_CMRATE_200HZ
 * @see MMC5983MA_RA_CTRL2
 * @see MMC5983MA_CR2_CMF_BIT
 * @see MMC5983MA_CR2_CMF_LENGTH
 */
uint8_t mmc5983maGetSampleRate()
{
    i2cdevReadBits(I2Cx, devAddr, MMC5983MA_RA_CTRL2, MMC5983MA_CR2_CMF_BIT, MMC5983MA_CR2_CMF_LENGTH, buffer);
    return buffer[0];
}
/** Set data output rate value.
*/
void mmc5983maSetSampleRate(uint8_t rate)
{
    i2cdevWriteBits(I2Cx, devAddr, MMC5983MA_RA_CTRL2, MMC5983MA_CR2_CMF_BIT, MMC5983MA_CR2_CMF_LENGTH, rate);
}

/** Get the number of Sets per measurement in periodic SR mode.
 * Value | Sets per measurement
 * ------+------------------------------
 * 0     | 1
 * 1     | 25
 * 2     | 75
 * 3     | 100
 * 4     | 250
 * 5     | 500
 * 6     | 1000
 * 7     | 2000
 * 
 * @return Current bias value (0-2 for normal/positive/negative respectively)
 * @see MMC5983MA_PRDSET_1
 * @see MMC5983MA_RA_CTRL2
 * @see MMC5983MA_CR2_PRDSET_BIT
 * @see MMC5983MA_CR2_PRDSET_LENGTH
 */
uint8_t mmc5983maGetPeriodicSR()
{
    i2cdevReadBits(I2Cx, devAddr, MMC5983MA_RA_CTRL2, MMC5983MA_CR2_PRDSET_BIT, MMC5983MA_CR2_PRDSET_LENGTH, buffer);
    return buffer[0];
}

/** Set the number of Sets per measurement in periodic SR mode.
 */
void mmc5983maSetPeriodicSR(uint8_t rate)
{
    i2cdevWriteBits(I2Cx, devAddr, MMC5983MA_RA_CTRL2, MMC5983MA_CR2_PRDSET_BIT, MMC5983MA_CR2_PRDSET_LENGTH, rate);
}

// MODE

/** Get the measurement mode (continuous/single).
 * Value | Mode
 * ------+------------------------------
 * 0     | Single Mode (CM off)
 * 1     | Continuous Mode (CM on)
 */
uint8_t mmc5983maGetMode()
{
    i2cdevReadBit(I2Cx, devAddr, MMC5983MA_RA_CTRL2, MMC5983MA_CR2_CMEN_BIT, mode);
    return mode;
}

void mmc5983maSetMode(uint8_t mode)
{
    i2cdevWriteBit(I2Cx, devAddr, MMC5983MA_RA_CTRL2, MMC5983MA_CR2_CMEN_BIT, mode);
}


// DATA* registers

/** Get 3-axis heading measurements.
 * In the event the ADC reading overflows or underflows for the given channel,
 * or if there is a math overflow during the bias measurement, this data
 * register will contain the value -4096. This register value will clear when
 * after the next valid measurement is made.
 * 
 * The MMC operates in 16/18 bit mode. In 16 bit mode, just read bits 17:2
 * @param x 16-bit signed integer container for X-axis heading
 * @param y 16-bit signed integer container for Y-axis heading
 * @param z 16-bit signed integer container for Z-axis heading
 */
void mmc5983maGetHeading(int16_t *x, int16_t *y, int16_t *z)
{
    i2cdevReadReg8(I2Cx, devAddr, MMC5983MA_RA_XOUT0, 6, buffer);

    if (mode == MMC5983MA_CM_OFF) {
        i2cdevWriteByte(I2Cx, devAddr, MMC5983MA_RA_CTRL0, 1 << MMC5983MA_CR0_TMM_BIT);
    }

    *x = (((int16_t)buffer[0]) << 8) | buffer[1];
    *y = (((int16_t)buffer[4]) << 8) | buffer[5];
    *z = (((int16_t)buffer[2]) << 8) | buffer[3];
}
/** Get X-axis heading measurement.
 * @return 16-bit signed integer with X-axis heading
 * @see HMC5883L_RA_DATAX_H
 */
int16_t hmc5883lGetHeadingX()
{
    // each axis read requires that ALL axis registers be read, even if only
    // one is used; this was not done ineffiently in the code by accident
    i2cdevReadReg8(I2Cx, devAddr, MMC5983MA_RA_XOUT0, 6, buffer);

    if (mode == MMC5983MA_CM_OFF) {
        i2cdevWriteByte(I2Cx, devAddr, MMC5983MA_RA_CTRL0, 1 << MMC5983MA_CR0_TMM_BIT);
    }

    return (((int16_t)buffer[0]) << 8) | buffer[1];
}
/** Get Y-axis heading measurement.
 * @return 16-bit signed integer with Y-axis heading
 * @see HMC5883L_RA_DATAY_H
 */
int16_t hmc5883lGetHeadingY()
{
    // each axis read requires that ALL axis registers be read, even if only
    // one is used; this was not done ineffiently in the code by accident
    i2cdevReadReg8(I2Cx, devAddr, MMC5983MA_RA_XOUT0, 6, buffer);

    if (mode == MMC5983MA_CM_OFF) {
        i2cdevWriteByte(I2Cx, devAddr, MMC5983MA_RA_CTRL0, 1 << MMC5983MA_CR0_TMM_BIT);
    }

    return (((int16_t)buffer[4]) << 8) | buffer[5];
}
/** Get Z-axis heading measurement.
 * @return 16-bit signed integer with Z-axis heading
 * @see HMC5883L_RA_DATAZ_H
 */
int16_t hmc5883lGetHeadingZ()
{
    // each axis read requires that ALL axis registers be read, even if only
    // one is used; this was not done ineffiently in the code by accident
    i2cdevReadReg8(I2Cx, devAddr, MMC5983MA_RA_XOUT0, 6, buffer);

    if (mode == MMC5983MA_CM_OFF) {
        i2cdevWriteByte(I2Cx, devAddr, MMC5983MA_RA_CTRL0, 1 << MMC5983MA_CR0_TMM_BIT);
    }

    return (((int16_t)buffer[2]) << 8) | buffer[3];
}

// CONTROL 3 REGISTER (STATUS)

/** Get magnetometer data ready
 */
bool mmc5983maMagReadyStatus()
{
    i2cdevReadBit(I2Cx, devAddr, MMC5983MA_RA_CTRL3, MMC5983MA_STATUS_MREADY_BIT, buffer);
    return buffer[0];
}

/** Get temperature ready status
 */
bool mmc5983maTempReadyStatus()
{
    i2cdevReadBit(I2Cx, devAddr, MMC5983MA_RA_CTRL3, MMC5983MA_STATUS_TREADY_BIT, buffer);
    return buffer[0];
}

// ID registers

/** Get identification byte A
 * @return ID_A byte (should be 01001000, ASCII value 'H')
 */
uint8_t mmc5983maGetPID()
{
    i2cdevReadByte(I2Cx, devAddr, MMC5983MA_RA_PID, buffer);
    return buffer[0];
}

