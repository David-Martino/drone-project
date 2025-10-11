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

#ifndef MMC5983MA_H_
#define MMC5983MA_H_
#include <stdbool.h>
#include "i2cdev.h"

// @@ NOTE: MMC does NOT have a built-in self test, so remove ST references in driver.

#define MMC5983MA_ADDRESS            0x1E // @@ 7-bit address as is I2C0... but how will MPU6050 handle it?
#define MMC5983MA_DEFAULT_ADDRESS    0x1E
#define MMC5983MA_DEFAULT_PID        0x30

#define MMC5983MA_RA_XOUT0          0x00
#define MMC5983MA_RA_XOUT1          0x01
#define MMC5983MA_RA_YOUT0          0x02
#define MMC5983MA_RA_YOUT1          0x03
#define MMC5983MA_RA_ZOUT0          0x04
#define MMC5983MA_RA_ZOUT1          0x05
#define MMC5983MA_RA_XYZOUT2        0x06
#define MMC5983MA_RA_TOUT           0x07
#define MMC5983MA_RA_STATUS         0x08
#define MMC5983MA_RA_CTRL0          0x09
#define MMC5983MA_RA_CTRL1          0x0A
#define MMC5983MA_RA_CTRL2          0x0B
#define MMC5983MA_RA_CTRL3          0x0C
#define MMC5983MA_RA_PID            0x2F

#define MMC5983MA_CR0_TMM_BIT       0 // Take mag measurement
#define MMC5983MA_CR0_TMT_BIT       1 // Take temperature measurement
#define MMC5983MA_CR0_INT_BIT       2 // Enable interrupt pin
#define MMC5983MA_CR0_SET_BIT       3 // Performs set operation (cleared after complete)
#define MMC5983MA_CR0_RST_BIT       4 // Performs reset operation (cleared after complete)
#define MMC5983MA_CR0_AUTOSR_BIT    5 // Enable auto set/reset
#define MMC5983MA_CR0_OTP_BIT       6 // Lets device read OTP agai83

#define MMC5983MA_CR1_BW_BIT        1 // Bandwidth LSB
#define MMC5983MA_CR1_BW_LENGTH     2
#define MMC5983MA_CR1_XIBT_BIT      2 // (1)Disable X channel
#define MMC5983MA_CR1_YZIBT_BIT     4 // (11)Disable YZ channel 
#define MMC5983MA_CR1_YZIBT_LENGTH  2
#define MMC5983MA_CR1_SWRST_BIT     7 // 

#define MMC5983MA_BW_100HZ          0x00
#define MMC5983MA_BW_200HZ          0x01
#define MMC5983MA_BW_400HZ          0x02
#define MMC5983MA_BW_800HZ          0x083

#define MMC5983MA_CR2_CMF_BIT       2 // Continuous measurement frequency
#define MMC5983MA_CR2_CMF_LENGTH    3 
#define MMC5983MA_CR2_CMEN_BIT      3 // Enable continuous measurement mode
#define MMC5983MA_CR2_PRDSET_BIT    6 // How often to perform set/reset
#define MMC5983MA_CR2_PRDSET_LENGTH 3
#define MMC5983MA_CR2_ENPRD_BIT     7 // Enable periodic set/rese83

#define MMC5983MA_CMRATE_OFF        0x00
#define MMC5983MA_CMRATE_1HZ        0x01
#define MMC5983MA_CMRATE_10HZ       0x02
#define MMC5983MA_CMRATE_20HZ       0x03
#define MMC5983MA_CMRATE_50HZ       0x04
#define MMC5983MA_CMRATE_100HZ      0x05
#define MMC5983MA_CMRATE_200HZ      0x06
#define MMC5983MA_CMRATE_1000HZ     0x07
#define MMC5983MA_PRDSET_1          0x00
#define MMC5983MA_PRDSET_25         0x01
#define MMC5983MA_PRDSET_75         0x02
#define MMC5983MA_PRDSET_100        0x03
#define MMC5983MA_PRDSET_250        0x04
#define MMC5983MA_PRDSET_500        0x05
#define MMC5983MA_PRDSET_1000       0x06
#define MMC5983MA_PRDSET_2000       0x08

#define MMC5983MA_CM_OFF            0x00 // this is set in the CMEN bit of CR2
#define MMC5984MA_CM_ON             0x01

#define MMC5983MA_CR3_STENP_BIT     1 // Apply extra current flowing from positive to negative of a coil for self testing sensor saturation
#define MMC5983MA_CR3_STENM_BIT     2 // Apply extra current flowing from negative to positive of a coil for self testing sensor saturation
#define MMC5983MA_CR3_SPI3_BIT      6 // (1) Enable 3-Wire SPI mod83
#define MMC5983MA_STATUS_MREADY_BIT 0 // Status reg mag measurement done
#define MMC5983MA_STATUS_TREADY_BIT 1 // Status reg temp measurement done
#define MMC5983MA_STATUS_OTP_BIT    4 // Status reg OTP read successful

void mmc5983maInit(I2C_Dev *i2cPort);
bool mmc5983maTestConnection();

// CONFIG_A register
uint8_t mmc5983maGetBandwidth();
void mmc5983maSetBandwidth(uint8_t bandwidth);
uint8_t mmc5983maGetSampleRate();
void mmc5983maSetSampleRate(uint8_t rate);
uint8_t mmc5983maGetPeriodicSR();
void mmc5983maSetPeriodicSR(uint8_t rate);
uint8_t mmc5983maGetMode();
void mmc5983maSetMode(uint8_t mode);

// DATA* registers
void mmc5983maGetHeading(int16_t *x, int16_t *y, int16_t *z);
int16_t mmc5983maGetHeadingX();
int16_t mmc5983maGetHeadingY();
int16_t mmc5983maGetHeadingZ();
void mmc5983maGetTemperature(int16_t *temp);

// STATUS register
bool mmc5983maTempReadyStatus();
bool mmc5983maMagReadyStatus();

// ID_* registers
uint8_t mmc5983maGetPID();

#endif /* MMC5983MA_H_ */
