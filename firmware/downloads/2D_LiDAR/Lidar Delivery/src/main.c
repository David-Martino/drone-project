/*
******************************************************************************
* File Name          : main.c
* Description        : Main program body for 2D Lidar Demo using VL53L1X
* Revision			 : 1.0
******************************************************************************
*
* COPYRIGHT(c) 2020 STMicroelectronics
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*   1. Redistributions of source code must retain the above copyright notice,
*   this list of conditions and the following disclaimer.
*   2. Redistributions in binary form must reproduce the above copyright
*   notice, this list of conditions and the following disclaimer in the
*   documentation and/or other materials provided with the distribution.
*   3. Neither the name of STMicroelectronics nor the names of its contributors
*   may be used to endorse or promote products derived from this software
*   without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
* CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*
******************************************************************************
*/

#include "platform.h"
#include "stm32f4xx.h"
#include "stm32f4xx_nucleo.h"
#include "platform_specific.h"
#include <string.h>
#include "VL53L1X_api.h"
#include "math.h"

#define RadarCircleRadius 						(110.0 / 2.0)
#define Pi 										3.1415
#define VHV_TIMER 								200
#define ROI_CONFIG__USER_ROI_CENTRE_SPAD		0x007F
#define NumOfTOFSensors							9
#define TotalWidthOfSPADS						16
#define WidthOfSPADsPerZone						4
#define NumOfSPADsShiftPerZone					1
#define HorizontalFOVofSensor					19.09
#define SingleSPADFOV							(HorizontalFOVofSensor/TotalWidthOfSPADS)
#define NumOfZonesPerSensor						(((TotalWidthOfSPADS - WidthOfSPADsPerZone) / NumOfSPADsShiftPerZone) + 1)
#define StartingZoneAngle						(WidthOfSPADsPerZone / 2 * SingleSPADFOV)
#define ZoneFOVChangePerStep					(SingleSPADFOV * NumOfSPADsShiftPerZone)

/* ----- UART variables, only useful for Serial communication ----- */

extern char Uart_RXBuffer[80];	/* Buffer for continuous RX */
#ifdef USER
extern char UartComm_RXBuffer[80];
#endif
extern int UART_Active;		/* Flag to see if UART is active */
extern int UART_Ready;		/* Flag new command available   */
extern size_t Uart_RxIndex;

/* ----- VL53L1X variables ----- */

uint16_t Dev_init = 0x52;		/* I2C address of device 1 */
float    LidarAngle[117];
uint16_t LidarDistance[117];
uint32_t TimeStamp[117];
uint16_t Devs[9] = {0x62, 0x64, 0x66, 0x68, 0x6A, 0x6C, 0x6E, 0x70, 0x72};
uint16_t Distance;
uint16_t SignalRate;
uint16_t SpadNb;
uint16_t AmbientRate;
uint16_t SignalPerSpad;
uint16_t AmbientPerSpad;
uint8_t RangeStatus;
uint16_t RangeCounter = 0;
//#define Calibrate
#ifdef Calibrate
int16_t  OffsetCal[NumOfTOFSensors*NumOfZonesPerSensor] = {
		0,0,0,0,0,0,0,0,0,0,0,0,0,
		0,0,0,0,0,0,0,0,0,0,0,0,0,
		0,0,0,0,0,0,0,0,0,0,0,0,0,
		0,0,0,0,0,0,0,0,0,0,0,0,0,
		0,0,0,0,0,0,0,0,0,0,0,0,0,
		0,0,0,0,0,0,0,0,0,0,0,0,0,
		0,0,0,0,0,0,0,0,0,0,0,0,0,
		0,0,0,0,0,0,0,0,0,0,0,0,0,
		0,0,0,0,0,0,0,0,0,0,0,0,0
		};
#else
int16_t  OffsetCal[NumOfTOFSensors * NumOfZonesPerSensor] = {
 -40, -44, -29, -24, -21, -18, -14, -15, -14, -13, -17, -22, -32,
 -28, -17, -21, -24, -11, -12, -12,  -9, -11, -26, -25, -36, -38,
 -36, -46, -26, -24, -20, -16, -17, -14, -11, -18, -17, -26, -37,
 -43, -31, -21, -25, -20,  -8, -12, -12, -13, -20, -27, -26, -38,
 -51, -41, -36, -24, -22, -21, -19, -16, -17, -22, -32, -41, -52,
 -44, -38, -26, -26, -29, -17,  -8, -11, -20, -20, -28, -34, -40,
 -50, -41, -37, -39, -23, -11, -21, -18, -20, -18, -23, -34, -48,
 -50, -41, -29, -28, -20, -16, -10, -16, -22, -25, -22, -45, -41,
 -43, -40, -25, -27, -16, -21, -14,  -9, -28, -39, -40, -46, -55
};


#endif





uint16_t   zone_center[]={247,239,231,223,215,207,199,191,183,175,167,159,151, 247, 239, 231, 223, 215};
// Timing Budget Options:  15, 20, 33, 50, 100, 200, 500
uint16_t TimingBudget = 15;

uint16_t current_zone=0;
char BigBuff[4000];
char VL53L1X_BUFFER[60];	/* Create a buffer to get data */

void PlotPolarData(uint8_t SensorNum, uint8_t CurrentZone, uint8_t NumOfZones, uint16_t Distance);
void TurnOnSensor(uint8_t SensorNum);
void ResetAllSensors(void);
void ResetAndInitializeAllSensors(void);
void MX_GPIO_Init(void);
void SystemClock_Config(void);

/* -----Main program ----- */

int main(void)
{
	VL53L1X_ERROR error = 0;
	uint32_t i=0;
	uint8_t Zone, Sensor, Timeout;

	uint32_t TimeStart, TimeEnd, TotalTime, CurrentTime;
	uint8_t Sensorcheck;
	HAL_Init();		/* Reset of all peripherals  */
	SystemClock_Config();
	UART_Init();		/* Initialize UART interface */
	MX_GPIO_Init();
	UART_Reboot(&Uart_RxIndex, &UART_Active, &UART_Ready, Uart_RXBuffer);
	I2C_Init();		/* Initialize I2C interface      */
	//GPIO_Expander_Init();
	ResetAndInitializeAllSensors();
	while (1)
	{
		error = 0;
		TimeStart = HAL_GetTick();
		Timeout = 0;
		for(Zone = 0; Zone < NumOfZonesPerSensor; Zone++)
		{
			for (Sensor=0; Sensor < NumOfTOFSensors ; Sensor++)
			{
				WriteRegister8(Devs[Sensor], ROI_CONFIG__USER_ROI_CENTRE_SPAD, zone_center[Zone+1] - 0);
			}
			i=i+1;
			for (Sensor=0; Sensor < NumOfTOFSensors ; Sensor++)
			{
				error = VL53L1X_CheckForDataReady(Devs[Sensor], &Sensorcheck);
				while ((Sensorcheck == 0) && (Timeout == 0))
				{
					HAL_Delay(1);
					CurrentTime = HAL_GetTick();
					if (CurrentTime > (TimeStart + (NumOfZonesPerSensor + 1) * TimingBudget * 2))
					{
						Timeout = 1;
						Sensor = NumOfTOFSensors;
						Zone = NumOfZonesPerSensor;
					}
					else
					{
						error += VL53L1X_CheckForDataReady(Devs[Sensor], &Sensorcheck);
					}
				}
				if (Timeout == 0)
				{
					WriteRegister8(Devs[Sensor], ROI_CONFIG__USER_ROI_CENTRE_SPAD, zone_center[Zone+1] - 0);
					TimeStamp[Sensor*13+ Zone] = HAL_GetTick();
					VL53L1X_ClearInterrupt(Devs[Sensor]);

					error += VL53L1X_GetDistance(Devs[Sensor], &Distance);
					error += VL53L1X_GetRangeStatus(Devs[Sensor], &RangeStatus);
					if ((RangeStatus== 0) || (RangeStatus == 7))
					{
						if (Distance > 60000)
						{
							Distance = 0;
							PlotPolarData(Sensor, Zone, 13, 0);
						}
						else
						{
							Distance = Distance + OffsetCal[Sensor*13 + Zone];
							if (Distance > 60000)
							{
								Distance = 0;
							}
							PlotPolarData(Sensor, Zone, 13, Distance);
						}
					}
					else
					{
						PlotPolarData(Sensor, Zone, 13, 4000);
					}
				}
			}
		}
		if (Timeout == 1)
		{
			ResetAndInitializeAllSensors();
			Timeout = 0;
			UART_Print("Reset Performed\n");
		}
		else
		{
			HAL_Delay(TimingBudget);
			TimeEnd = HAL_GetTick();;
			TotalTime = (TimeEnd - TimeStart);
			snprintf(BigBuff, sizeof(BigBuff), "Time: %ld\n", TotalTime);
			UART_Print(BigBuff);
		}
		if (error !=0)
		{
			UART_Print("Some Errors seen\n");
		}
	}
}
uint8_t dataRead;
void ResetAndInitializeAllSensors(void)
{
	uint8_t i, Sensor, error = 0;
	uint8_t Bootstate = 0;
	int16_t Offset;
	ResetAllSensors();
	HAL_Delay(10);
	for (i = 0; i < NumOfTOFSensors; i++)
	{
		TurnOnSensor(i);
		HAL_Delay(5);
		error += VL53L1X_BootState(Dev_init, &Bootstate);
		while (Bootstate != 0x03)
		{
			HAL_Delay(5);
			error += VL53L1X_BootState(Dev_init, &Bootstate);
		}
		VL53L1X_SensorInit(Dev_init);	/* Initialize sensor  */
		VL53L1X_SetI2CAddress(Dev_init, Devs[i]);	/* Change i2c address Left is now 0x62 and Dev1 */
		dataRead = ReadRegister8(Devs[i], 0x10f);
		dataRead = ReadRegister8(Devs[i], 0x110);
	}
	UART_Print("All Chips booted\n");

	for (Sensor = 0; Sensor < NumOfTOFSensors; Sensor++)
	{
		VL53L1X_SetDistanceMode(Devs[Sensor], 1);
		VL53L1X_SetTimingBudgetInMs(Devs[Sensor], TimingBudget);
		VL53L1X_SetInterMeasurementInMs(Devs[Sensor], TimingBudget);
		VL53L1X_SetROI(Devs[Sensor], WidthOfSPADsPerZone, 6);
		WriteRegister8(Devs[Sensor], ROI_CONFIG__USER_ROI_CENTRE_SPAD, zone_center[0] - 0);
		error = VL53L1X_GetOffset(Devs[Sensor], &Offset);
		VL53L1X_SetOffset(Devs[Sensor], Offset + 40);
	}
	for (Sensor = 0; Sensor < NumOfTOFSensors; Sensor++)
	{
		VL53L1X_StartRanging(Devs[Sensor]);
		HAL_Delay(1);
	}
	if (error !=0)
	{
		UART_Print("Some Errors seen\n");
	}

}

float OldAngle;
double SystemAngle;
void PlotPolarData(uint8_t SensorNum, uint8_t CurrentZone, uint8_t NumOfZones, uint16_t Distance)
{
	double PartZoneAngle;

	float CorrectedDistance = 0;

	if (Distance > 60000)
	{
		Distance = 0;
	}
	PartZoneAngle = (StartingZoneAngle + ZoneFOVChangePerStep*CurrentZone) - (HorizontalFOVofSensor / 2.0);
	SystemAngle = -80 + 20.0*SensorNum + PartZoneAngle;
	CorrectedDistance = pow(pow(RadarCircleRadius,2) + pow(Distance, 2) - (2 * RadarCircleRadius * Distance * cos((180 - PartZoneAngle)/(180) * Pi)), 0.5);
	if (CorrectedDistance < 55)
	{
		CorrectedDistance = 55;
	}
	LidarAngle[SensorNum*13+CurrentZone] = SystemAngle;
	LidarDistance[SensorNum*13+CurrentZone] = (uint16_t)CorrectedDistance;
}

void TurnOnSensor(uint8_t SensorNum)
{
	switch (SensorNum)
	{
		case 0:
			// GPIO PC2
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET);
			break;
		case 1:
			// GPIO PC3
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET);
			break;
		case 2:
			// GPIO PC4
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET);
			break;
		case 3:
			// GPIO PC5
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET);
			break;
		case 4:
			// GPIO PC6
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
			break;
		case 5:
			// GPIO PC7
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
			break;
		case 6:
			// GPIO PC8
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
			break;
		case 7:
			// GPIO PC9
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
			break;
		case 8:
			// GPIO PC10
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_SET);
			break;
		case 9:
			// GPIO PC11
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_SET);
			break;
		case 10:
			// GPIO PC12
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_SET);
			break;
		case 11:
			// GPIO PB0
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
			break;
		case 12:
			// GPIO PB1
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
			break;
		case 13:
			// GPIO PB2
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
			break;
		case 14:
			// GPIO PB3
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);
			break;
		case 15:
			// GPIO PB4
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
			break;
		case 16:
			// GPIO PB5
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
			break;
		case 17:
			// GPIO PB6
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
			break;
		case 18:
			// GPIO PB7
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
			break;
	}
}

void ResetAllSensors(void)
{
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_RESET);

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
}

void MX_GPIO_Init(void)
{

	GPIO_InitTypeDef GPIO_InitStruct;

	/* GPIO Ports Clock Enable */
	__GPIOC_CLK_ENABLE();
	__GPIOH_CLK_ENABLE();
	__GPIOA_CLK_ENABLE();
	__GPIOB_CLK_ENABLE();

	/*Configure GPIO pin : B1_Pin */
	GPIO_InitStruct.Pin = GPIO_PIN_13;
	GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pin : LD2_Pin */
	GPIO_InitStruct.Pin = GPIO_PIN_5;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	// Reset Pin
	GPIO_InitStruct.Pin = GPIO_PIN_0;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	// PWR EN Pin
	GPIO_InitStruct.Pin = GPIO_PIN_10;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	// CS Pin
	GPIO_InitStruct.Pin = GPIO_PIN_6;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	// Reset Pins for VL53L1 Sensors

	GPIO_InitStruct.Pin = GPIO_PIN_0;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = GPIO_PIN_1;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = GPIO_PIN_2;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = GPIO_PIN_3;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = GPIO_PIN_4;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = GPIO_PIN_5;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = GPIO_PIN_6;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = GPIO_PIN_7;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = GPIO_PIN_2;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = GPIO_PIN_3;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = GPIO_PIN_4;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = GPIO_PIN_5;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = GPIO_PIN_6;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = GPIO_PIN_7;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = GPIO_PIN_8;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = GPIO_PIN_9;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = GPIO_PIN_10;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = GPIO_PIN_11;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = GPIO_PIN_12;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	ResetAllSensors();
}

void SystemClock_Config(void)
{

	RCC_OscInitTypeDef RCC_OscInitStruct;
	RCC_ClkInitTypeDef RCC_ClkInitStruct;

	//Configure the main internal regulator output voltage
	__HAL_RCC_PWR_CLK_ENABLE();

	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

	/**Initializes the CPU, AHB and APB busses clocks
	*/
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 8;
	RCC_OscInitStruct.PLL.PLLN = 336;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
	RCC_OscInitStruct.PLL.PLLQ = 7;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{

	}

	/**Initializes the CPU, AHB and APB busses clocks
	*/
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
							  |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
	{

	}

	/**Configure the Systick interrupt time
	*/
	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

	/**Configure the Systick
	*/
	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

	/* SysTick_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}


