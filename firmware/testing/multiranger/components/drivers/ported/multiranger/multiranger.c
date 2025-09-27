/*
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Copyright 2021, Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Foobar is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Foobar.  If not, see <http://www.gnu.org/licenses/>.
 */
/* multiranger.c: Multiranger deck driver */

// @@ ADDED FOR MULTIRANGER DECK


//#include "deck.h" @@ not in ESP-Drone
#include "deck_digital.h"
#include "deck_spi.h"
#include "param.h"
#include "multiranger.h" // @@ 

#define DEBUG_MODULE "MR"

// @@ for debugging nicer.
#define BYTE_TO_BINARY_PATTERN "%c%c%c%c%c%c%c%c"
#define BYTE_TO_BINARY(byte)  \
  ((byte) & 0x80 ? '1' : '0'), \
  ((byte) & 0x40 ? '1' : '0'), \
  ((byte) & 0x20 ? '1' : '0'), \
  ((byte) & 0x10 ? '1' : '0'), \
  ((byte) & 0x08 ? '1' : '0'), \
  ((byte) & 0x04 ? '1' : '0'), \
  ((byte) & 0x02 ? '1' : '0'), \
  ((byte) & 0x01 ? '1' : '0')

#include "system.h"
#include "debug_cf.h" // @@ originally just debug.h
#include "log.h"
#include "pca95x4.h" // @@ already in ESP-Drone
#include "vl53l1x.h"
#include "range.h"
#include "static_mem.h" 

#include "i2cdev.h"

#include "FreeRTOS.h"
#include "task.h"

#include <stdlib.h>

static bool isInit = false;
static bool isTested = false;
static bool isPassed = false;
static uint16_t filterMask = 1 << VL53L1_RANGESTATUS_RANGE_VALID;

#define MR_PIN_UP PCA95X4_P0
#define MR_PIN_FRONT PCA95X4_P4
#define MR_PIN_BACK PCA95X4_P1
#define MR_PIN_LEFT PCA95X4_P6
#define MR_PIN_RIGHT PCA95X4_P2

NO_DMA_CCM_SAFE_ZERO_INIT static VL53L1_Dev_t devFront;
NO_DMA_CCM_SAFE_ZERO_INIT static VL53L1_Dev_t devBack;
NO_DMA_CCM_SAFE_ZERO_INIT static VL53L1_Dev_t devUp;
NO_DMA_CCM_SAFE_ZERO_INIT static VL53L1_Dev_t devLeft;
NO_DMA_CCM_SAFE_ZERO_INIT static VL53L1_Dev_t devRight;

static bool mrInitSensor(VL53L1_Dev_t *pdev, uint32_t pca95pin, char *name)
{
    bool status;
    uint8_t xshut_pins; // @@

    // Bring up VL53 by releasing XSHUT
    pca95x4SetOutput(PCA95X4_DEFAULT_ADDRESS, pca95pin);

    // @@ Check that the output reg is as intended:
    xshut_pins = pca95x4GetOutput(PCA95X4_DEFAULT_ADDRESS);
    //DEBUG_PRINTI("Requested outputs: 0x%x", pca95pin); @@
    //DEBUG_PRINTI("XSHUT UNSTRUCTURED: 0x%x", xshut_pins); @@
    DEBUG_PRINTI("UP: %d BACK: %d RIGHT: %d FRONT: %d LEFT: %d", 0x01 & (xshut_pins >> 0), 0x01 & (xshut_pins >> 1), 0x01 & (xshut_pins >> 2),0x01 & (xshut_pins >> 4), 0x01 & (xshut_pins >> 6));

    // Let VL53 boot
    vTaskDelay(M2T(2)); 
    // Init VL53
    if (vl53l1xInit(pdev, I2C1_DEV))
    {
        DEBUG_PRINTI("Init %s sensor [OK]\n", name); // @@ original: DEBUG_PRINT
        status = true;
    }
    else
    {
        DEBUG_PRINTW("Init %s sensor [FAIL]\n", name); // @@ original: DEBUG_PRINT
        status = false;
    }

    return status;
}

static uint16_t mrGetMeasurementAndRestart(VL53L1_Dev_t *dev)
{
    VL53L1_Error status = VL53L1_ERROR_NONE;
    VL53L1_RangingMeasurementData_t rangingData;
    uint8_t dataReady = 0;
    uint16_t range;

    while (dataReady == 0)
    {
        status = VL53L1_GetMeasurementDataReady(dev, &dataReady);
        vTaskDelay(M2T(1));
    }

    status = VL53L1_GetRangingMeasurementData(dev, &rangingData);

    if (filterMask & (1 << rangingData.RangeStatus))
    {
        range = rangingData.RangeMilliMeter;
    }
    else
    {
        range = 32767;
        //printf("%d\n", rangingData.RangeStatus); // @@
    }

    VL53L1_StopMeasurement(dev);
    status = VL53L1_StartMeasurement(dev);
    status = status;
    //DEBUG_PRINTI("Measurement Status: %d", status); // @@

    return range;
}

static void mrTask(void *param)
{
    VL53L1_Error status = VL53L1_ERROR_NONE;

    systemWaitStart();

    
    // Restart all sensors
    status = VL53L1_StopMeasurement(&devFront);
    DEBUG_PRINTI("Measurement Status: %d", status); // @@
    status = VL53L1_StartMeasurement(&devFront);
    DEBUG_PRINTI("Measurement Status: %d", status); // @@
    status = VL53L1_StopMeasurement(&devBack);
    DEBUG_PRINTI("Measurement Status: %d", status); // @@
    status = VL53L1_StartMeasurement(&devBack);
    i2c_scan(I2C1_DEV);
    DEBUG_PRINTI("Measurement Status: %d", status); // @@
    status = VL53L1_StopMeasurement(&devUp);
    DEBUG_PRINTI("Measurement Status: %d", status); // @@
    status = VL53L1_StartMeasurement(&devUp);
    DEBUG_PRINTI("Measurement Status: %d", status); // @@
    // status = VL53L1_StopMeasurement(&devLeft); 
    // status = VL53L1_StartMeasurement(&devLeft);
    // status = VL53L1_StopMeasurement(&devRight);
    // status = VL53L1_StartMeasurement(&devRight);
    i2c_scan(I2C1_DEV);

    status = status;

    TickType_t lastWakeTime = xTaskGetTickCount();

    while (1)
    {
        // @@ RevEng: rangeSet ONLY publishes the ranges to the ranges array, then
        // this array is accessed by crtp_localization_service.c to send the range 
        // data to the GCS via CRTP. So, could intercept the range data and send via
        // whichever channel we desire.
        vTaskDelayUntil(&lastWakeTime, M2T(100));
        rangeSet(rangeFront, mrGetMeasurementAndRestart(&devFront) / 1000.0f);
        rangeSet(rangeBack, mrGetMeasurementAndRestart(&devBack) / 1000.0f);
        rangeSet(rangeUp, mrGetMeasurementAndRestart(&devUp) / 1000.0f);
        //rangeSet(rangeLeft, mrGetMeasurementAndRestart(&devLeft) / 1000.0f);
        //rangeSet(rangeRight, mrGetMeasurementAndRestart(&devRight) / 1000.0f);
        
        //DEBUG_PRINTI("Range Front: %d", rangeGet(rangeFront));
    }
}

// @@ Removed static
void mrInit()
{
    if (isInit)
    {
        return;
    }

    pca95x4Init();

    // @@ this defines all the USED pins as outputs
    // pca95x4ConfigOutput(PCA95X4_DEFAULT_ADDRESS,
    //                     ~(MR_PIN_UP |
    //                       MR_PIN_RIGHT |
    //                       MR_PIN_LEFT |
    //                       MR_PIN_FRONT |
    //                       MR_PIN_BACK));

    pca95x4ConfigOutput(PCA95X4_DEFAULT_ADDRESS, 0x00); // @@ makes ALL pins outputs

    // @@ 
    // pca95x4ClearOutput(PCA95X4_DEFAULT_ADDRESS,
    //                    MR_PIN_UP |
    //                    MR_PIN_RIGHT |
    //                    MR_PIN_LEFT |
    //                    MR_PIN_FRONT |
    //                    MR_PIN_BACK);
    pca95x4ClearOutput(PCA95X4_DEFAULT_ADDRESS, 0xff); // @@ makes ALL pins low

    rangeSet(rangeFront, 1000 / 1000.0f);
    rangeSet(rangeBack, 1000 / 1000.0f);
    rangeSet(rangeUp, 1000 / 1000.0f);
    rangeSet(rangeLeft, 1000 / 1000.0f);
    rangeSet(rangeRight, 1000 / 1000.0f);
                       
    isInit = true;

    xTaskCreate(mrTask, MULTIRANGER_TASK_NAME, MULTIRANGER_TASK_STACKSIZE, NULL,
                MULTIRANGER_TASK_PRI, NULL);
                
}

// @@ Removed static
bool mrTest()
{
    if (isTested)
    {
        return isPassed;
    }

    isPassed = isInit;

    isPassed &= pca95x4Test(PCA95X4_DEFAULT_ADDRESS); // @@ 
    isPassed &= mrInitSensor(&devFront, MR_PIN_FRONT, "front");
    isPassed &= mrInitSensor(&devBack, MR_PIN_BACK, "back");
    isPassed &= mrInitSensor(&devUp, MR_PIN_UP, "up");
    mrInitSensor(&devLeft, MR_PIN_LEFT, "left"); // @@ isPassed &= mrInitSensor(&devLeft, MR_PIN_LEFT, "left");
    mrInitSensor(&devRight, MR_PIN_RIGHT, "right");// @@ isPassed &= mrInitSensor(&devRight, MR_PIN_RIGHT, "right");

    isTested = true;

    return isPassed;
}

// @@ RevEng: This would be used for identification with the one-wire eeprom (we don't use this).
// static const DeckDriver multiranger_deck = {
//     .vid = 0xBC,
//     .pid = 0x0C,
//     .name = "bcMultiranger",

//     .usedGpio = 0,
//     .usedPeriph = DECK_USING_I2C,

//     .init = mrInit,
//     .test = mrTest,
// };

// DECK_DRIVER(multiranger_deck);



// @@
static uint8_t disable = 0;
#define PARAM_CORE (1<<5)
#define PARAM_PERSISTENT (1 << 8)
#define PARAM_ADD_CORE(TYPE, NAME, ADDRESS) \
  PARAM_ADD(TYPE | PARAM_CORE, NAME, ADDRESS)

PARAM_GROUP_START(deck)

/**
 * @brief Nonzero if [Multi-ranger deck](%https://store.bitcraze.io/collections/decks/products/multi-ranger-deck) is attached
 */
PARAM_ADD_CORE(PARAM_UINT8 | PARAM_RONLY, bcMultiranger, &isInit)

PARAM_GROUP_STOP(deck)

PARAM_GROUP_START(multiranger)
/**
 * @brief Filter mask determining which range measurements is to be let through based on the range status of the VL53L1 chip
 */
PARAM_ADD(PARAM_UINT16, filterMask, &filterMask)

PARAM_GROUP_STOP(multiranger)
