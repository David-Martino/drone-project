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
 * Copyright (C) 2011-2012 Bitcraze AB
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
 * Modifications: 
 *    Copyright (C) 2025 Nathan Mayhew
 *      - Added emergency landing and critical flag system
 *
 * pm.c - Power Management driver and functions.
 */

#include <string.h>
#include <stdbool.h>

#include "FreeRTOS.h"
#include "task.h"

#include "config.h"
#include "system.h"
#include "pm_esplane.h"
#include "adc_esp32.h"
#include "led.h"
#include "log.h"
#include "ledseq.h"
#include "commander.h"
#include "sound.h"
#include "stm32_legacy.h"
#include "range.h" // @@ 
#include "stabilizer.h" // @@
#include "param.h"

#define OFF_DIST 0.05 // @@ distance (mm) to ground at which motors are turned off.
#define DESCEND_SPEED  0.3 // @@ emergency landing speed (m/s)
//#define OVERRIDE_LV_SHUTDOWN // @@ macro for auto landing
//#include "deck.h"
#define DEBUG_MODULE "PM"
#include "debug_cf.h"
#include "static_mem.h"

typedef struct _PmSyslinkInfo
{
  union
  {
    uint8_t flags;
    struct
    {
      uint8_t chg    : 1;
      uint8_t pgood  : 1;
      uint8_t unused : 6;
    };
  };
  float vBat;
  float chargeCurrent;
#ifdef PM_SYSTLINK_INLCUDE_TEMP
  float temp;
#endif
}  __attribute__((packed)) PmSyslinkInfo;

static float     batteryVoltage;
static uint16_t  batteryVoltageMV;
static float     batteryVoltageMin = 6.0;
static float     batteryVoltageMax = 0.0;

static float     extBatteryVoltage;
static uint16_t  extBatteryVoltageMV;
static uint16_t extBatVoltDeckPin;
static bool      isExtBatVoltDeckPinSet = false;
static float     extBatVoltMultiplier;
static float     extBatteryCurrent;
static uint16_t extBatCurrDeckPin;
static bool      isExtBatCurrDeckPinSet = false;
static float     extBatCurrAmpPerVolt;

#ifdef PM_SYSTLINK_INLCUDE_TEMP
// nRF51 internal temp
static float    temp;
#endif

static uint32_t batteryLowTimeStamp;
static uint32_t batteryCriticalLowTimeStamp;
static bool isInit;
static PMStates pmState;
static PmSyslinkInfo pmSyslinkInfo;

static uint8_t batteryLevel;

static uint8_t criticalFlag = 0; // @@ state variable. 1 critical low, so engage/continue landing procedure
static bool landed = false; // @@ state variable. 1 means emergency landing complete, 0 otherwise.

static void pmSetBatteryVoltage(float voltage);

const static float bat671723HS25C[10] =
{
  // @@ Custom SOC values from discharge testing (Green Flower Gaoneng battery 2S 700mAh)
  6.00, // 0
  6.20, // 10
  6.40, // 20%
  7.36, // 30%
  7.46, // 40%
  7.53, // 50%
  7.60, // 60%
  7.70, // 70%
  7.86, // 80%
  8.04  // 90%
};

STATIC_MEM_TASK_ALLOC(pmTask, PM_TASK_STACKSIZE);

void pmInit(void)
{
  if(isInit) {
    return;
  }

    pmEnableExtBatteryVoltMeasuring(CONFIG_ADC1_PIN, 4); // ADC1 PIN is fixed to ADC channel //@@ Modified from 2->4 for our 4:1 voltage divider

    pmSyslinkInfo.pgood = false;
    pmSyslinkInfo.chg = false;
    pmSyslinkInfo.vBat = 7.4f; //@@ changed 3.7f->7.4f
    pmSetBatteryVoltage(pmSyslinkInfo.vBat);

    STATIC_MEM_TASK_CREATE(pmTask, pmTask, PM_TASK_NAME, NULL, PM_TASK_PRI);
    isInit = true;

}

bool pmTest(void)
{
  return isInit;
}

/**
 * Sets the battery voltage and its min and max values
 */
static void pmSetBatteryVoltage(float voltage)
{
  batteryVoltage = voltage;
  batteryVoltageMV = (uint16_t)(voltage * 1000);
  if (batteryVoltageMax < voltage)
  {
    batteryVoltageMax = voltage;
  }
  if (batteryVoltageMin > voltage)
  {
    batteryVoltageMin = voltage;
  }
}

/**
 * Shutdown system
 */
static void pmSystemShutdown(void)
{
  #ifdef OVERRIDE_LV_SHUTDOWN
    DEBUG_PRINTE("WARNING! LOW BATTERY AUTO SHUTDOWN IS OVERRIDDEN, PROCEED WITH CAUTION.");
  #else
    criticalFlag = 1;
  #endif
#ifdef ACTIVATE_AUTO_SHUTDOWN
//TODO: Implement syslink call to shutdown
#endif
}

/**
 * Returns a number from 0 to 9 where 0 is completely discharged
 * and 9 is 90% charged.
 */
static int32_t pmBatteryChargeFromVoltage(float voltage)
{
  int charge = 0;

  if (voltage < bat671723HS25C[0])
  {
    return 0;
  }
  if (voltage > bat671723HS25C[9])
  {
    return 9;
  }
  while (voltage >  bat671723HS25C[charge])
  {
    charge++;
  }

  return charge;
}


float pmGetBatteryVoltage(void)
{
  return batteryVoltage;
}

float pmGetBatteryVoltageMin(void)
{
  return batteryVoltageMin;
}

float pmGetBatteryVoltageMax(void)
{
  return batteryVoltageMax;
}

void pmSyslinkUpdate(SyslinkPacket *slp)
{
  if (slp->type == SYSLINK_PM_BATTERY_STATE) {
    memcpy(&pmSyslinkInfo, &slp->data[0], sizeof(pmSyslinkInfo));
    pmSetBatteryVoltage(pmSyslinkInfo.vBat);
#ifdef PM_SYSTLINK_INLCUDE_TEMP
    temp = pmSyslinkInfo.temp;
#endif
  }
}

void pmSetChargeState(PMChargeStates chgState)
{
  // TODO: Send syslink packafe with charge state
}

PMStates pmUpdateState()
{
  PMStates state;
  bool isCharging = pmSyslinkInfo.chg;
  bool isPgood = pmSyslinkInfo.pgood;
  uint32_t batteryLowTime;

  batteryLowTime = xTaskGetTickCount() - batteryLowTimeStamp;

  if (isPgood && !isCharging)
  {
    state = charged;
  }
  else if (isPgood && isCharging)
  {
    state = charging;
  }
  else if (!isPgood && !isCharging && (batteryLowTime > PM_BAT_LOW_TIMEOUT))
  {
    state = lowPower;
  }
  else
  {
    state = battery;
  }

  return state;
}

void pmEnableExtBatteryCurrMeasuring(uint8_t pin, float ampPerVolt)
{
  extBatCurrDeckPin = pin;
  isExtBatCurrDeckPinSet = true;
  extBatCurrAmpPerVolt = ampPerVolt;
}

float pmMeasureExtBatteryCurrent(void)
{
  float current;

  if (isExtBatCurrDeckPinSet)
  {
    current = analogReadVoltage(extBatCurrDeckPin) * extBatCurrAmpPerVolt;
  }
  else
  {
    current = 0.0;
  }

  return current;
}

void pmEnableExtBatteryVoltMeasuring(uint8_t pin, float multiplier)
{
  extBatVoltDeckPin = pin;
  isExtBatVoltDeckPinSet = true;
  extBatVoltMultiplier = multiplier;
}

float pmMeasureExtBatteryVoltage(void)
{
  float voltage;

  if (isExtBatVoltDeckPinSet)
  {
    voltage = analogReadVoltage(extBatVoltDeckPin) * extBatVoltMultiplier;
  }
  else
  {
    voltage = 0.0;
  }

  return voltage;
}

bool pmIsBatteryLow(void) {
  return (pmState == lowPower);
}

bool pmIsChargerConnected(void) {
  return (pmState == charging) || (pmState == charged);
}

bool pmIsCharging(void) {
  return (pmState == charging);
}
// return true if battery discharging
bool pmIsDischarging(void)
{
  PMStates pmState;
  pmState = pmUpdateState();
  return (pmState == lowPower) || (pmState == battery);
}

/** @@ Emergency Landing procedure
 * @brief Emergency landing procedure
 * @param setpoint setpoint structure that is being passed to controller
 * @return 1 if emergency landing complete, 0 otherwise
 */
bool pmEmergencyLand(setpoint_t *setpoint, state_t *state) {

  if (criticalFlag) {
    if (state->position.z > OFF_DIST) { // if greater than 50mm from the floor, continue sending a DESCEND setpoint
      setpoint->mode.x = modeAbs;
      setpoint->mode.y = modeAbs;
      setpoint->mode.z = modeVelocity;
      setpoint->mode.roll = modeDisable;
      setpoint->mode.pitch = modeDisable;
      setpoint->mode.yaw = modeDisable;
      setpoint->velocity.x = state->position.x; // maintain x-y position
      setpoint->velocity.y = state->position.y;
      setpoint->velocity.z = -DESCEND_SPEED;
      //DEBUG_PRINTI("Landing");
    }
    else {
      //DEBUG_PRINTI("Landing complete");
      landed = true; // Emergency Landing complete, shut down power.

      systemSetArmed(false);
    }
  };
  
  return landed;
}

/** Set Down
 * 
 */
void pmSetCriticalFlag(void) {
  criticalFlag = 1;
}

void pmTask(void *param)
{
  PMStates pmStateOld = battery;
  uint32_t tickCount = 0;

#ifdef configUSE_APPLICATION_TASK_TAG
	#if configUSE_APPLICATION_TASK_TAG == 1
    vTaskSetApplicationTaskTag(0, (void *)TASK_PM_ID_NBR);
    #endif
#endif

  tickCount = xTaskGetTickCount();
  batteryLowTimeStamp = tickCount;
  batteryCriticalLowTimeStamp = tickCount;
  pmSetChargeState(charge300mA);
  systemWaitStart();

  while (1) {
  vTaskDelay(M2T(100));
  extBatteryVoltage = pmMeasureExtBatteryVoltage();
  extBatteryVoltageMV = (uint16_t)(extBatteryVoltage * 1000);
  extBatteryCurrent = pmMeasureExtBatteryCurrent();
  pmSetBatteryVoltage(extBatteryVoltage);
  batteryLevel = pmBatteryChargeFromVoltage(pmGetBatteryVoltage()) * 10;
#ifdef DEBUG_EP2
  DEBUG_PRINTD("batteryLevel=%u extBatteryVoltageMV=%u \n", batteryLevel, extBatteryVoltageMV);
#endif
    tickCount = xTaskGetTickCount();

    if (pmGetBatteryVoltage() > PM_BAT_LOW_VOLTAGE)
    {
      batteryLowTimeStamp = tickCount;
    }
    if (pmGetBatteryVoltage() > PM_BAT_CRITICAL_LOW_VOLTAGE)
    {
      batteryCriticalLowTimeStamp = tickCount;
    }

        pmState = pmUpdateState();

    if (pmState != pmStateOld)
    {
      // Actions on state change
      switch (pmState)
      {
        case charged:
          //ledseqStop(&seq_charging);
          //ledseqRunBlocking(&seq_charged);
          soundSetEffect(SND_BAT_FULL);
          systemSetCanFly(false);
          break;
        case charging:
          //ledseqStop(&seq_lowbat);
          //ledseqStop(&seq_charged);
          ledseqRunBlocking(&seq_charging);
          soundSetEffect(SND_USB_CONN);
          systemSetCanFly(false);
          break;

        case lowPower:
          ledseqRunBlocking(&seq_lowbat);
          soundSetEffect(SND_BAT_LOW);
          systemSetCanFly(true);
          break;
        case battery:
          //ledseqRunBlocking(&seq_charging);
          //ledseqRun(&seq_charged);
          soundSetEffect(SND_USB_DISC);
          systemSetCanFly(true);
          break;
        default:
          systemSetCanFly(true);
          break;
      }
      pmStateOld = pmState;
    }
    // Actions during state
    switch (pmState)
    {
      case charged:
        break;
      case charging:
        {
          // Charge level between 0.0 and 1.0
          float chargeLevel = pmBatteryChargeFromVoltage(pmGetBatteryVoltage()) / 10.0f;
          ledseqSetChargeLevel(chargeLevel);
        }
        break;
      case lowPower:
        {
          uint32_t batteryCriticalLowTime;

          batteryCriticalLowTime = tickCount - batteryCriticalLowTimeStamp;
          if (batteryCriticalLowTime > PM_BAT_CRITICAL_LOW_TIMEOUT)
          {
            pmSystemShutdown();
          }
        }
        break;
      case battery:
        {
          if ((commanderGetInactivityTime() > PM_SYSTEM_SHUTDOWN_TIMEOUT))
          {
            pmSystemShutdown();
          }
        }
        break;
      default:
        break;
    }
  }
}

PARAM_GROUP_START(pm)
PARAM_ADD(PARAM_INT8, critflag, &criticalFlag)
PARAM_ADD(PARAM_INT8, landed, &landed)
PARAM_GROUP_STOP(pm)

LOG_GROUP_START(pm)
LOG_ADD(LOG_FLOAT, vbat, &batteryVoltage)
LOG_ADD(LOG_UINT16, vbatMV, &batteryVoltageMV)
LOG_ADD(LOG_FLOAT, extVbat, &extBatteryVoltage)
LOG_ADD(LOG_UINT16, extVbatMV, &extBatteryVoltageMV)
LOG_ADD(LOG_FLOAT, extCurr, &extBatteryCurrent)
LOG_ADD(LOG_FLOAT, chargeCurrent, &pmSyslinkInfo.chargeCurrent)
LOG_ADD(LOG_INT8, state, &pmState)
LOG_ADD(LOG_UINT8, batteryLevel, &batteryLevel)
#ifdef PM_SYSTLINK_INLCUDE_TEMP
LOG_ADD(LOG_FLOAT, temp, &temp)
#endif
LOG_GROUP_STOP(pm)
