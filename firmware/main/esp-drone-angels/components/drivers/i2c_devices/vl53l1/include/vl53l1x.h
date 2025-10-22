/*******************************************************************************
 Copyright (C) 2016, STMicroelectronics International N.V.
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:
 * Redistributions of source code must retain the above copyright
 notice, this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright
 notice, this list of conditions and the following disclaimer in the
 documentation and/or other materials provided with the distribution.
 * Neither the name of STMicroelectronics nor the
 names of its contributors may be used to endorse or promote products
 derived from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE, AND
 NON-INFRINGEMENT OF INTELLECTUAL PROPERTY RIGHTS ARE DISCLAIMED.
 IN NO EVENT SHALL STMICROELECTRONICS INTERNATIONAL N.V. BE LIABLE FOR ANY
 DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 Modifications
     Copyright (C) Nathan Mayhew 2025
       - Adapted to 8-bit addressing and other compatibility changes
 ******************************************************************************/


#ifndef _VL53L1X_H_
#define _VL53L1X_H_

//#include "vl53l1_ll_def.h" // @@ Low level driver type definitions (this file is NOT in ULD)
//#include "vl53l1_platform_user_data.h" // @@ needed for VL53L1_Dev_t (this file is NOT in ULD)
#include "core/inc/VL53L1X_api.h" // @@ needed for the initialisation (and test) (also slightly renamed)
#include "i2cdev.h" // @@ needed for the i2c driver functions
#include "vl53l1_types.h" // @@ needed for ST's fuckass type definitions

#ifdef __cplusplus
extern "C"
{
#endif

#define VL53L1X_DEFAULT_ADDRESS 0x52 // @@ updated to 8-bit address
#define VL53L1X_ID			0xEACC


// #define USE_I2C_2V8 // @@TODO DOES ULD AFFECT THIS?? 

/**
 * @file   vl53l1_platform.h
 *
 * @brief  All end user OS/platform/application porting
 */

// @@ ULD defines this with a dummy variable in vl53l1x_platform.h, so I will do the equivalent here.
typedef struct {
	uint8_t   I2cDevAddr;
	I2C_Dev *I2Cx;
	// I2C_HandleTypeDef *I2cHandle;
} VL53L1_Dev_t; // @@ this is NOT the original VL53L1_Dev_t definition, the struct is only needed by the address and the pointer to the I2C handle.

typedef VL53L1_Dev_t *VL53L1_DEV;

bool vl53l1xInit(VL53L1_Dev_t *pdev, I2C_Dev *I2cHandle, int16_t offset);

bool vl53l1xTestConnection(VL53L1_Dev_t* pdev);

uint8_t vl53l1xSetI2CAddress(VL53L1_Dev_t* pdev, uint8_t address);

/**
 * @brief Writes the supplied byte buffer to the device
 *
 * @param[in]   pdev      : pointer to device structure (device handle)
 * @param[in]   index     : uint16_t register index value
 * @param[in]   pdata     : pointer to uint8_t (byte) buffer containing the data to be written
 * @param[in]   count     : number of bytes in the supplied byte buffer
 *
 * @return   VL53L1X_ERROR_NONE    Success
 * @return  "Other error code"    See ::VL53L1X_ERROR
 */

uint8_t VL53L1_WriteMulti(
		uint16_t dev,
		uint16_t      index,
		uint8_t      *pdata,
		uint32_t      count);


/**
 * @brief  Reads the requested number of bytes from the device
 *
 * @param[in]   pdev      : pointer to device structure (device handle)
 * @param[in]   index     : uint16_t register index value
 * @param[out]  pdata     : pointer to the uint8_t (byte) buffer to store read data
 * @param[in]   count     : number of bytes to read
 *
 * @return   VL53L1X_ERROR_NONE    Success
 * @return  "Other error code"    See ::VL53L1X_ERROR
 */

uint8_t VL53L1_ReadMulti(
		uint16_t dev,
		uint16_t      index,
		uint8_t      *pdata,
		uint32_t      count);


/**
 * @brief  Writes a single byte to the device
 *
 * @param[in]   pdev      : pointer to device structure (device handle)
 * @param[in]   index     : uint16_t register index value
 * @param[in]   data      : uint8_t data value to write
 *
 * @return   VL53L1X_ERROR_NONE    Success
 * @return  "Other error code"    See ::VL53L1X_ERROR
 */

uint8_t VL53L1_WrByte(
		uint16_t dev,
		uint16_t      index,
		uint8_t       data);


/**
 * @brief  Writes a single word (16-bit unsigned) to the device
 *
 * Manages the big-endian nature of the device register map
 * (first byte written is the MS byte).
 *
 * @param[in]   pdev      : pointer to device structure (device handle)
 * @param[in]   index     : uint16_t register index value
 * @param[in]   data      : uin16_t data value write
 *
 * @return   VL53L1X_ERROR_NONE    Success
 * @return  "Other error code"    See ::VL53L1X_ERROR
 */

uint8_t VL53L1_WrWord(
		uint16_t dev,
		uint16_t      index,
		uint16_t      data);


/**
 * @brief  Writes a single dword (32-bit unsigned) to the device
 *
 * Manages the big-endian nature of the device register map
 * (first byte written is the MS byte).
 *
 * @param[in]   pdev      : pointer to device structure (device handle)
 * @param[in]   index     : uint16_t register index value
 * @param[in]   data      : uint32_t data value to write
 *
 * @return   VL53L1X_ERROR_NONE    Success
 * @return  "Other error code"    See ::VL53L1X_ERROR
 */

uint8_t VL53L1_WrDWord(
		uint16_t dev,
		uint16_t      index,
		uint32_t      data);



/**
 * @brief  Reads a single byte from the device
 *
 * @param[in]   pdev      : pointer to device structure (device handle)
 * @param[in]   index     : uint16_t register index
 * @param[out]  pdata     : pointer to uint8_t data value
 *
 * @return   VL53L1X_ERROR_NONE    Success
 * @return  "Other error code"    See ::VL53L1X_ERROR
 *
 */

uint8_t VL53L1_RdByte(
		uint16_t dev,
		uint16_t      index,
		uint8_t      *pdata);


/**
 * @brief  Reads a single word (16-bit unsigned) from the device
 *
 * Manages the big-endian nature of the device (first byte read is the MS byte).
 *
 * @param[in]   pdev      : pointer to device structure (device handle)
 * @param[in]   index     : uint16_t register index value
 * @param[out]  pdata     : pointer to uint16_t data value
 *
 * @return   VL53L1X_ERROR_NONE    Success
 * @return  "Other error code"    See ::VL53L1X_ERROR
 */

uint8_t VL53L1_RdWord(
		uint16_t dev,
		uint16_t      index,
		uint16_t     *pdata);


/**
 * @brief  Reads a single dword (32-bit unsigned) from the device
 *
 * Manages the big-endian nature of the device (first byte read is the MS byte).
 *
 * @param[in]   pdev      : pointer to device structure (device handle)
 * @param[in]   index     : uint16_t register index value
 * @param[out]  pdata     : pointer to uint32_t data value
 *
 * @return   VL53L1X_ERROR_NONE    Success
 * @return  "Other error code"    See ::VL53L1X_ERROR
 */

uint8_t VL53L1_RdDWord(
		uint16_t dev,
		uint16_t      index,
		uint32_t     *pdata);



/**
 * @brief  Implements a programmable wait in us
 *
 * @param[in]   pdev      : pointer to device structure (device handle)
 * @param[in]   wait_us   : integer wait in micro seconds
 *
 * @return  VL53L1X_ERROR_NONE     Success
 * @return  "Other error code"    See ::VL53L1X_ERROR
 */

uint8_t VL53L1_WaitUs(
		uint16_t dev,
		int32_t       wait_us);


/**
 * @brief  Implements a programmable wait in ms
 *
 * @param[in]   pdev      : pointer to device structure (device handle)
 * @param[in]   wait_ms   : integer wait in milliseconds
 *
 * @return  VL53L1X_ERROR_NONE     Success
 * @return  "Other error code"    See ::VL53L1X_ERROR
 */

uint8_t VL53L1_WaitMs(
		uint16_t dev,
		int32_t       wait_ms);

#ifdef __cplusplus
}
#endif

#endif

