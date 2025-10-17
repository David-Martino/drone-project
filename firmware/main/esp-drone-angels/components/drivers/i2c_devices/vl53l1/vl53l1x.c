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
 ******************************************************************************/

/**
 * @file   vl53l1_platform.c
 * @brief  Code function definitions for Crazyflie
 *
 */
#include <stdio.h>      // sprintf(), vsnprintf(), printf()
#include <stdint.h>
#include <string.h>     // strncpy(), strnlen()

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "i2cdev.h"
#include "vl53l1x.h"
#define DEBUG_MODULE "VLX1"
#define VL53L1X_ERROR_CONTROL_INTERFACE 14 // @@ this is NOT from ULD, its an artefact of the OG driver
#include "debug_cf.h"

#define CALIB
#ifdef PAL_EXTENDED
//	#include "vl53l1_register_strings.h" // @@ not ULD
#else
	#define VL53L1_get_register_name(a,b)
#endif

// Set the start address 8 step after the VL53L0 dynamic addresses @@ using 0x52 as the address because it seems this is what ULD usees
static int nextI2CAddress = 0x60; // @@ was +8 originally - CF actually start from 0x60 though...

// @@ SMP FreeRTOS requires taskENTER_CRITICAL to have a spinlock, since the normal
// @@ taskENTER_CRITICAL just blocks ISRs from accessing a resource (but on SMP
// @@ another core could also access that resource. Do I understand this? Sure.)
static portMUX_TYPE my_spinlock = portMUX_INITIALIZER_UNLOCKED;


// @@ Declaring debugging functions
void i2c_test(VL53L1_Dev_t *dev);
int rd_write_verification( VL53L1_Dev_t *dev, uint16_t addr, uint32_t expected_value);


// @@ Initialisation follows the UM https://www.st.com/resource/en/user_manual/um2510-a-guide-to-using-the-vl53l1x-ultra-lite-driver-stmicroelectronics.pdf
// @@ and some of the 2D_Lidar example (https://www.st.com/en/embedded-software/stsw-img017.html),
bool vl53l1xInit(VL53L1_Dev_t *pdev, I2C_Dev *I2cHandle, int16_t offset)
{
  uint8_t status = 0;
  uint8_t Bootstate = 0;
  int newAddress;
  uint8_t count = 0;

  taskENTER_CRITICAL(&my_spinlock); 
  newAddress = nextI2CAddress + 2; // @@ Must increment address by 2 since ULD uses an 8-bit address and right shifts (not the 0x29). So bit 0 used for R/W and bit 1 is being incremented on
  nextI2CAddress = nextI2CAddress + 2; // @@ I'm just trying to be explicit here
  taskEXIT_CRITICAL(&my_spinlock);

  pdev->I2cDevAddr = VL53L1X_DEFAULT_ADDRESS;
  pdev->I2Cx = I2cHandle;

  i2cdevInit(pdev->I2Cx); //@@
  //i2c_scan(pdev->I2Cx); //@@ scan the i2c addresses

  // @@ Poll the device 4 times, skip if it does not respond
  while ((Bootstate == 0) && (count <= 3))
  {
  	status = VL53L1X_BootState(pdev->I2cDevAddr, &Bootstate);
	count++;
  }
  DEBUG_PRINTI("Boot Status: %d", status); // @@DEBUG
  
  if (status == 0) {
	status = VL53L1X_SensorInit(pdev->I2cDevAddr); // @@ Load default config into sensor
	DEBUG_PRINTI("Sensor Init status: %d", status); // @@DEBUG
	// @@ modifications to sensor parameters can occur now

	if (status == 0) {
		status = vl53l1xSetI2CAddress(pdev,newAddress); //@@ set new I2C address
		DEBUG_PRINTI("VL53L1 Address to %x [Status: %d], and pdev->i2cDevAddr = %x\n", newAddress, status, pdev->I2cDevAddr); // @@DEBUG
		
		// @@ Just checking ID registers (stock espdrone) (DEBUG)
		uint8_t byteData;
		uint16_t wordData;
		VL53L1_RdByte(pdev->I2cDevAddr, 0x010F, &byteData);
		DEBUG_PRINTI( "VL53L1X Model_ID: %02X\n\r", byteData);
		VL53L1_RdByte(pdev->I2cDevAddr, 0x0110, &byteData);
		DEBUG_PRINTI( "VL53L1X Module_Type: %02X\n\r", byteData);
		VL53L1_RdWord(pdev->I2cDevAddr, 0x010F, &wordData);
		DEBUG_PRINTI( "VL53L1X: %02X\n\r", wordData);


		/** Configure Device Params */

		VL53L1X_SetOffset(pdev->I2cDevAddr, offset);

		/** Set ROI to avoid prop guard interference (16x16 -> 4x4)
		 * You can adjust the center as well but seems not necessary
		 * |_|_|_|_|_|_|_|_|_|_|_|_|_|_|_|_|
		 * |_|_|_|_|_|_|_|_|_|_|_|_|_|_|_|_|
		 * |_|_|_|_|_|_|_|_|_|_|_|_|_|_|_|_|
		 * |_|_|_|_|_|_|_|_|_|_|_|_|_|_|_|_|
		 * |_|_|_|_|_|_|_|_|_|_|_|_|_|_|_|_|
		 * |_|_|_|_|_|_|_|_|_|_|_|_|_|_|_|_|
		 * |_|_|_|_|_|_|x|x|x|x|_|_|_|_|_|_|
		 * |_|_|_|_|_|_|x|x|x|x|_|_|_|_|_|_|
		 * |_|_|_|_|_|_|x|x|x|x|_|_|_|_|_|_|
		 * |_|_|_|_|_|_|x|x|x|x|_|_|_|_|_|_|
		 * |_|_|_|_|_|_|_|_|_|_|_|_|_|_|_|_|
		 * |_|_|_|_|_|_|_|_|_|_|_|_|_|_|_|_|
		 * |_|_|_|_|_|_|_|_|_|_|_|_|_|_|_|_|
		 * |_|_|_|_|_|_|_|_|_|_|_|_|_|_|_|_|
		 * |_|_|_|_|_|_|_|_|_|_|_|_|_|_|_|_|
		 * |_|_|_|_|_|_|_|_|_|_|_|_|_|_|_|_|
		 * */ 

		// status = VL53L1X_SetROI(pdev->I2cDevAddr, 8, 8);
		//status = VL53L1X_SetROICenter(pdev->I2cDevAddr, )



	}
	
  }
  

#ifdef SET_VL53LX_ROI
  VL53L1_UserRoi_t Roi0;
  Roi0.TopLeftX = 0; //set ROI according to requirement
  Roi0.TopLeftY = 15;
  Roi0.BotRightX = 15;
  Roi0.BotRightY = 0;
  status = VL53L1_SetUserROI(pdev, &Roi0); //SET region of interest
#endif
	//Restart sensor remove to zranger task
	//VL53L1_StopMeasurement(pdev);
	//status = VL53L1_SetDistanceMode(pdev,VL53L1_DISTANCEMODE_LONG);
	//status = VL53L1_SetMeasurementTimingBudgetMicroSeconds(pdev, 160000);
	//status = VL53L1_SetInterMeasurementPeriodMilliSeconds(pdev, 200);  
	//status = VL53L1_StartMeasurement(pdev);

  return status == 0;
}



/** Set I2C address
 * Any subsequent communication will be on the new address
 * The address passed is the 7bit I2C address from LSB (ie. without the
 * read/write bit)
 */
uint8_t vl53l1xSetI2CAddress(VL53L1_Dev_t* pdev, uint8_t address)
{
  uint8_t status = 0;

  status = VL53L1X_SetI2CAddress(pdev->I2cDevAddr, address); // @@ modified for ULD API
  pdev->I2cDevAddr = address;
  return  status;
}


/*
 * ----------------- COMMS FUNCTIONS -----------------
 */

uint8_t VL53L1_WriteMulti( // @@ clear
	uint16_t dev,
	uint16_t      index,
	uint8_t      *pdata,
	uint32_t      count)
{
	uint8_t status         = 0;

  if (!i2cdevWrite16(I2C1_DEV, dev, index, count, pdata))
  {
    status = VL53L1X_ERROR_CONTROL_INTERFACE;
  }

	return status;
}

uint8_t VL53L1_ReadMulti(
	uint16_t dev,
	uint16_t      index,
	uint8_t      *pdata,
	uint32_t      count)
{
	uint8_t status         = 0;

  if (!i2cdevRead16(I2C1_DEV, dev, index, count, pdata))
  {
    status = VL53L1X_ERROR_CONTROL_INTERFACE;
  }

	return status;
}


uint8_t VL53L1_WrByte(
	uint16_t dev,
	uint16_t      index,
	uint8_t       data)
{
	uint8_t status         = 0;

	if (!i2cdevWrite16(I2C1_DEV, dev, index, 1, &data))
	{
	  status = VL53L1X_ERROR_CONTROL_INTERFACE;
	}

	return status;
}


uint8_t VL53L1_WrWord(
	uint16_t dev,
	uint16_t      index,
	uint16_t      data)
{
  uint8_t status         = 0;
  uint8_t _I2CBuffer[2];
  _I2CBuffer[0] = data >> 8;
  _I2CBuffer[1] = data & 0x00FF;
  if (!i2cdevWrite16(I2C1_DEV, dev, index, 2, (uint8_t *)_I2CBuffer))
  {
    status = VL53L1X_ERROR_CONTROL_INTERFACE;
  }

	return status;
}


uint8_t VL53L1_WrDWord(
	uint16_t dev,
	uint16_t      index,
	uint32_t      data)
{
	uint8_t status = 0;
  	uint8_t _I2CBuffer[4];
	_I2CBuffer[0] = (data >> 24) & 0xFF;
	_I2CBuffer[1] = (data >> 16) & 0xFF;
	_I2CBuffer[2] = (data >> 8) & 0xFF;
	_I2CBuffer[3] = (data >> 0) & 0xFF;

	if (!i2cdevWrite16(I2C1_DEV, dev, index, 4, (uint8_t *)_I2CBuffer))
  {
    status = VL53L1X_ERROR_CONTROL_INTERFACE;
  }

	return status;
}


uint8_t VL53L1_RdByte(
	uint16_t dev,
	uint16_t      index,
	uint8_t      *pdata)
{
	uint8_t status         = 0;

	if (!i2cdevRead16(I2C1_DEV, dev, index, 1, pdata))
  {
    status = VL53L1X_ERROR_CONTROL_INTERFACE;
  }

	return status;
}

uint8_t VL53L1_RdWord(
	uint16_t dev,
	uint16_t index,
	uint16_t *pdata)
{
	uint8_t status = 0;
	uint8_t _I2CBuffer[2];

	if (!i2cdevRead16(I2C1_DEV, dev, index, 2, (uint8_t *)_I2CBuffer))
	{
		status = VL53L1X_ERROR_CONTROL_INTERFACE;
	}
	*pdata = ((uint16_t)_I2CBuffer[0] << 8) + (uint16_t)_I2CBuffer[1];
	return status;
}

uint8_t VL53L1_RdDWord(
	uint16_t dev,
	uint16_t index,
	uint32_t *pdata)
{
	uint8_t status = 0;
	uint8_t _I2CBuffer[4];
	if (!i2cdevRead16(I2C1_DEV, dev, index, 4, (uint8_t *)_I2CBuffer))
	{
		status = VL53L1X_ERROR_CONTROL_INTERFACE;
	}
	*pdata = ((uint32_t)_I2CBuffer[0] << 24) + ((uint32_t)_I2CBuffer[1] << 16) + ((uint32_t)_I2CBuffer[2] << 8) + (uint32_t)_I2CBuffer[3];
	return status;
}

/*
 * ----------------- HOST TIMING FUNCTIONS -----------------
 */

uint8_t VL53L1_WaitUs(
	uint16_t dev,
	int32_t       wait_us)
{
	uint8_t status         = 0;
	uint32_t delay_ms = (wait_us + 900) / 1000;

	if(delay_ms == 0)
	{
	  delay_ms = 1;
	}

	vTaskDelay(M2T(delay_ms));

	return status;
}


uint8_t VL53L1_WaitMs(
	uint16_t dev,
	int32_t       wait_ms)
{
  vTaskDelay(M2T(wait_ms));

  return 0;
}

// @@ DEBUGGING CODE FROM
// @@ https://community.st.com/t5/imaging-sensors/vl53l1x-data-init-return-error/td-p/156670

//#define I2C_TEST @@ disabling for now,
#ifdef I2C_TEST
int rd_write_verification( VL53L1_Dev_t *dev, uint16_t addr, uint32_t expected_value)
{
	VL53L1X_ERROR Status  = VL53L1X_ERROR_NONE;
	uint8_t bytes[4],mbytes[4];
	uint16_t words[2];
	uint32_t dword;
	int i;
	VL53L1_ReadMulti(dev, addr, mbytes, 4);
	for (i=0; i<4; i++){ VL53L1_RdByte(dev, addr+i, &bytes[i]); }
	for (i=0; i<2; i++){ VL53L1_RdWord(dev, addr+i*2, &words[i]); }
	Status = VL53L1_RdDWord(dev, addr, &dword);
	
	printf("expected   = %8x,\n",expected_value);
	printf("read_multi = %2x, %2x, %2x, %2x\n", mbytes[0],mbytes[1],mbytes[2],mbytes[3]);
	printf("read_bytes = %2x, %2x, %2x, %2x\n", bytes[0],bytes[1],bytes[2],bytes[3]);
	printf("read words = %4x, %4x\n",words[0],words[1]);
	printf("read dword = %8x\n",dword);
	
	if((mbytes[0]<<24 | mbytes[1]<<16 | mbytes[2]<<8 | mbytes[3]) != expected_value) return (-1);
	if((bytes[0]<<24 | bytes[1]<<16 | bytes[2]<<8 | bytes[3]) != expected_value) return (-1);
	if((words[0]<<16 | words[1]) != expected_value) return (-1);
	if(dword != expected_value) return(-1);
	return Status;
	
}

#define REG 0x3A
void i2c_test(VL53L1_Dev_t *dev)
{
	VL53L1X_ERROR Status  = VL53L1X_ERROR_NONE;
	int err_count = 0;
	uint8_t buff[4] = {0x11,0x22,0x33,0x44};
	uint8_t long_out[135] ={0x29, 0x02, 0x10, 0x00, 0x22, 0xBC, 0xCC, 0x81, 0x80, 0x07, 0x16, 0x00, 0xFF, 0xFD,
							0xF7, 0xDE, 0xFF, 0x0F, 0x00, 0x15, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
							0x44, 0x00, 0x2C, 0x00, 0x11, 0x0A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
							0x00, 0x11, 0x02, 0x00, 0x02, 0x08, 0x00, 0x08, 0x10, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0xFF,
							0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x0B, 0x00, 0x00, 0x02, 0x14, 0x21, 0x00, 0x00,
							0x02, 0x00, 0x00, 0x00, 0x00, 0xC8, 0x00, 0x00, 0x38, 0xFF, 0x01, 0x00, 0x01, 0x00, 0x02, 0x00,
							0x9D, 0x07, 0x00, 0xD2, 0x05, 0x01, 0x68, 0x00, 0xC0, 0x08, 0x38, 0x00, 0x00, 0x00, 0x00, 0x03,
							0xB6, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x07, 0x05, 0x06, 0x06, 0x01, 0x00, 0x02,
							0xC7, 0xFF, 0x8B, 0x00, 0x00, 0x00, 0x01, 0x01, 0x40};
	uint8_t long_in[135]= {0xff};
  	int i=0;

	Status = rd_write_verification(dev, 0x10f, 0xeacc10ff);			// verify the Chip ID works

	Status += VL53L1_WriteMulti(dev, 0x01,  long_out, 135);			// check if WriteMulti can write 135 bytes
	Status += VL53L1_ReadMulti(dev, 0x01,  long_in, 135);			// check if WriteMulti can read 135 bytes

	for (i=0; i<135; i++) if(long_in[i] != long_out[i])err_count++;
	if (err_count > 10) Status++;

	Status += VL53L1_WriteMulti(dev, REG,  buff, 4);				// check WriteMulti
	if (rd_write_verification(dev, REG, 0x11223344) <0) err_count++;

	Status += VL53L1_WrDWord(dev, REG, 0xffeeddcc);				// check WrDWord
	if (rd_write_verification(dev, REG, 0xffeeddcc) <0) err_count++;

	Status += VL53L1_WrWord(dev, REG, 0x5566);					// check WrWord
	Status += VL53L1_WrWord(dev, REG+2, 0x7788);
	if (rd_write_verification(dev, REG, 0x55667788) <0) err_count++;

	for (i=0; i<4; i++){ VL53L1_WrByte (dev, REG+i, buff[i]); }
	if (rd_write_verification(dev, REG,0x11223344) <0) Status++;
	if (Status > 0)
	{
		printf("i2c test failed - please check it. Status = %d\n", Status);
	}
}

#endif