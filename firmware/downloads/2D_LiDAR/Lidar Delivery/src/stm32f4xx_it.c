/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @author  Ac6
  * @version V1.0
  * @date    02-Feb-2015
  * @brief   Default Interrupt Service Routines.
  ******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"
#ifdef USE_RTOS_SYSTICK
#include <cmsis_os.h>
#endif
#include "stm32f4xx_it.h"
#include "string.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            	  	    Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles SysTick Handler, but only if no RTOS defines it.
  * @param  None
  * @retval None
  */
#include "platform_specific.h"

extern UART_HandleTypeDef SERIAL_UART;
extern float    LidarAngle[117];
extern uint16_t LidarDistance[117];
extern char BigBuff[4000];
extern char VL53L1X_BUFFER[60];
extern int16_t  OffsetCal[9][13];
extern uint32_t TimeStamp[117];

void SysTick_Handler(void)
{
	HAL_IncTick();
	HAL_SYSTICK_IRQHandler();
#ifdef USE_RTOS_SYSTICK
	osSystickHandler();
#endif
}

uint8_t InputData, SR_Reg;
void USART2_IRQHandler(void)
{
	uint8_t i, j;

	SR_Reg = SERIAL_UART.Instance->SR & USART_SR_RXNE;
	if (SR_Reg != 0)
	{
		InputData = SERIAL_UART.Instance->DR;
	}
	if ((InputData == 0x53) || (InputData == 0x73)) // "S" to show the all LIDAR Data
	{
		snprintf(BigBuff, sizeof(BigBuff), "Start\n");
		for (i=0; i<117;i++)
		{
			snprintf(VL53L1X_BUFFER, sizeof(VL53L1X_BUFFER),
					"A%6.2f,D%4d\n",
					LidarAngle[i],
					LidarDistance[i]
					);
			strcat(BigBuff, VL53L1X_BUFFER);
		}
		strcat(BigBuff, "END\n");
		UART_Print(BigBuff);
	}
	else if  ((InputData == 0x54) || (InputData == 0x74)) // "T" to show the all LIDAR Data
	{
		snprintf(BigBuff, sizeof(BigBuff), "Start\n");
		for (i=0; i<117;i++)
		{
			snprintf(VL53L1X_BUFFER, sizeof(VL53L1X_BUFFER),
					"A%6.2f,D%4d,T%ld\n",
					LidarAngle[i],
					LidarDistance[i],
					TimeStamp[i]
					);
			strcat(BigBuff, VL53L1X_BUFFER);
		}
		strcat(BigBuff, "END\n");
		UART_Print(BigBuff);

	}
	else if ((InputData == 0x44) || (InputData == 0x64))  // "D" for full data dump
	{


	}
	else if ((InputData == 0x43) || (InputData == 0x63))  // "C" for Calibration Data at 30cm
	{
		snprintf(BigBuff, sizeof(BigBuff), "int16_t  OffsetCal[NumOfTOFSensors * NumOfZonesPerSensor] = {\n");
		for (i=0; i<9;i++)
		{
			for (j=0; j<13; j++)
			{
				if ((i == 8) && (j == 12))
				{
					snprintf(VL53L1X_BUFFER, sizeof(VL53L1X_BUFFER),"%4d",(300 - (LidarDistance[i*13+j] - OffsetCal[i][j] )));
				}
				else
				{
					snprintf(VL53L1X_BUFFER, sizeof(VL53L1X_BUFFER),"%4d,",(300 - (LidarDistance[i*13+j] - OffsetCal[i][j] )));
				}
				strcat(BigBuff, VL53L1X_BUFFER);
			}

			strcat(BigBuff, "\n");

		}
		strcat(BigBuff, "};\n");
		UART_Print(BigBuff);

	}
		//huart2.Instance->SR = 0;

}
