/* ------------------------------ Module Start ------------------------------ */

/**
 * @file imu_uart.c
 * @brief Implements the functionality for the ESP32->Pi UART communication.
 *
 *                  $$$$$$$\            $$\ $$\ $$\
 *                  $$  __$$\           $$ |$$ |$  |
 *                  $$ |  $$ | $$$$$$\  $$ |$$ |\_/$$$$$$$\
 *                  $$ |  $$ |$$  __$$\ $$ |$$ |  $$  _____|
 *                  $$ |  $$ |$$$$$$$$ |$$ |$$ |  \$$$$$$\
 *                  $$ |  $$ |$$   ____|$$ |$$ |   \____$$\
 *                  $$$$$$$  |\$$$$$$$\ $$ |$$ |  $$$$$$$  |
 *                  \_______/  \_______|\__|\__|  \_______/
 *
 *            $$$$$$\                                $$\
 *           $$  __$$\                               $$ |
 *           $$ /  $$ |$$$$$$$\   $$$$$$\   $$$$$$\  $$ | $$$$$$$\
 *           $$$$$$$$ |$$  __$$\ $$  __$$\ $$  __$$\ $$ |$$  _____|
 *           $$  __$$ |$$ |  $$ |$$ /  $$ |$$$$$$$$ |$$ |\$$$$$$\
 *           $$ |  $$ |$$ |  $$ |$$ |  $$ |$$   ____|$$ | \____$$\
 *           $$ |  $$ |$$ |  $$ |\$$$$$$$ |\$$$$$$$\ $$ |$$$$$$$  |
 *           \__|  \__|\__|  \__| \____$$ | \_______|\__|\_______/
 *                               $$\   $$ |
 *                               \$$$$$$  |
 *                                \______/
 * 
 * This file contains the definitions for functions and variables declared in
 * imu_uart.h, providing the core logic for transmitting IMU or state data 
 * from the to ESP32 to the Pi via UART.
 *
 * @author Erwin Bauernschmitt <22964301@student.uwa.edu.au>
 * @date 30 September 2025
 * 
 * @copyright
 *   © 2025 Erwin Bauernschmitt
 *   Licensed under GPLv3.0; see the LICENSE file
 */

/* -------------------------------- Includes -------------------------------- */

// Standard includes
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

// FreeRTOS includes
#include "FreeRTOS.h"
#include "static_mem.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

// App-specific includes
#include "config.h"
#include "imu_uart.h"
#include "uart_esp32.h"
#include "imu_types.h"
#include "uart_helpers.h"
#include "stabilizer.h"
#include "stabilizer_types.h"
#include "sensors_mpu6050_hm5883L_ms5611.h"
#include "param.h"

// ESP-IDF includes
#include "driver/uart.h"

// Debug includes and defines
#define DEBUG_MODULE "IMU_UART"
#include "debug_cf.h"

/* ----------------------------- Configurations ----------------------------- */

// Define size of UART header in bytes
#define HEADER_SIZE (1 + 4 + 8)

// Define size of kalman-filtered state data payload in bytes
#define KALMAN_STATE_DATA_SIZE (4 * 4 + 4 * 4)

// Define CRC16 size in bytes
#define CRC_SIZE 2

// Define max data payload size in bytes
#define MAX_DATA_SIZE (HEADER_SIZE + KALMAN_STATE_DATA_SIZE+ CRC_SIZE)

// Define max frame size after COBS encoding in bytes
#define MAX_FRAME_SIZE (MAX_DATA_SIZE + (1 + ((MAX_DATA_SIZE - 1) / 254)))

// Define max wire packet size after appending packet delimiter
#define MAX_PACKET_SIZE (MAX_FRAME_SIZE + 1)

// Define how often to send IMU data (send one every IMU_TX_FREQ)
#define IMU_TX_FREQ 2

/* -------------------------- Private Declarations -------------------------- */

// Create initialisation flag to be set to true when imuuartInit() completes
static bool isInit = false;

// Preallocate static memory for the IMU_UART task
STATIC_MEM_TASK_ALLOC(imuuartTask, IMU_UART_TASK_STACKSIZE);

// Define an enum for the different UART data types
enum UartDataType_e 
{
	rawImuData = 0,
	processedImuData,
	kalmanStateData,
	uartDataTypeCount,  // Sentinel, not a real UART data type
};
typedef enum UartDataType_e UartDataType;

// Define default form of IMU data to transmit
static const UartDataType defaultDataType = kalmanStateData;

// Select the default UART data type
static UartDataType uartDataType = defaultDataType;

// Define strings for the UART data types
static const char* const uartDataTypeNames[uartDataTypeCount] = 
{
	[rawImuData]       = "rawImuData",
	[processedImuData] = "processedImuData",
	[kalmanStateData]  = "kalmanStateData",
};

// Initialise a counter variable for how often to send IMU data
static uint64_t skipCounter = 0;

/* ----------------------------- Local Helpers ------------------------------ */

/**
 * @brief Returns the string name of the UART data type
 *
 * The function ensures the index is valid.
 *
 * @param[in] t The UART data type to stringify.
 * @return      Non-NULL pointer to a static string (e.g., "rawImuData").
 */
const char* uartDataTypeName(UartDataType t)
{
	// Cast to unsigned so negatives become large and fail the bounds check
	unsigned idx = (unsigned)t;

	// Check if requested index is an actual UART data type
	if (idx < (unsigned)uartDataTypeCount && uartDataTypeNames[idx] != NULL) 
	{
		// Return the string name
		return uartDataTypeNames[idx];
	}

	// If not a valid index, return "unknown"
	return "unknown";
}

/* ----------------------------- FreeRTOS Tasks ----------------------------- */

/**
 * @brief FreeRTOS task that packages IMU data and transmits over UART.
 *
 * The task runs indefinitely. It receives the latest IMU data, builds a frame,
 * and writes to the UART driver. Designed to be created by imuuartInit().
 *
 * @param[in] param Optional context pointer (unused; passed NULL).
 */
static void imuuartTask(void *param)
{ 
	// Initialise memory of the previous UART data type with the default
	UartDataType previousDataType = uartDataType;

	// Initialise memory of the current UART data type with the default
	UartDataType currentDataType = uartDataType;

	// Get the string name of the default UART data type
	const char* dataTypeName = uartDataTypeName(uartDataType);

	// Debug print the UART data type
	DEBUG_PRINTI("UART data type set to default: %s", dataTypeName);

	// Initialise a buffer to hold the flattened data
	uint8_t buffer[MAX_DATA_SIZE] = {0};

	// Initialise a buffer to hold the complete encoded data packet
	uint8_t packet[MAX_PACKET_SIZE] = {0};

	// Initialise a variable to hold the buffer offset 
	size_t offset = 0;

	// Initialise a packet counter variable 
	uint32_t packetCounter = 0;

	// Initialise a boolean to track if all data was successfully received
	bool isDataReceived = true;

	// Initialise a variable to hold the IMU timestamp
	uint64_t imuTimestamp = 0;  // Here since all types need a timestamp

	// Initialise a variable to hold the try write boolean
	bool writeResult = false;

	// Initialise a variable to hold the UART FIFO buffer hint
	UartTxHint hint = 0;

	while(true)
	{
		// Block until the uartReady semaphore can be taken
		uartWaitDataReady();

		// Increment skip variable
		skipCounter += 1;

		// If it is not time to send IMU data yet 
		if (skipCounter < IMU_TX_FREQ)
		{
			// Skip to next loop iteration
			continue;
		}
		// If it is time to send IMU data
		else
		{
			// Reset the skip variable
			skipCounter = 0;
		}

		// Save current value of uartDataType in case its parameter value is changed
		currentDataType = uartDataType;  // Kept constant within each loop

		// Check for change in UART data type by CRTP param change
		if (currentDataType != previousDataType)
		{
			// Update string name of the UART data type
			dataTypeName = uartDataTypeName(currentDataType);

			// Notify of change with debug message
			DEBUG_PRINTI("UART data type changed to: %s", dataTypeName);
			
			// Update memory of the previous UART data type
			previousDataType = currentDataType;
		}

		// Reset the data buffer
		memset(buffer, 0, sizeof(buffer));

		// Reset the packet buffer
		memset(packet, 0, sizeof(packet));
		
		// Reset the boolean for receiving all necessary data
		isDataReceived = true;

		// Receive the most recent IMU timestamp into the timestamp variable
		isDataReceived &= readImuTimestampQueue(&imuTimestamp);

		// If timestamp was not received successfully
		if (isDataReceived == false)
		{
			// Notify user of error via debug
			DEBUG_PRINTE("Failed to receive timestamp for the packet");

			// Skip to next loop iteration
			continue;
		}

		// Reset the buffer offset
		offset = 0;

		// Construct and send the appropriate data
		switch (currentDataType) 
		{
			case rawImuData:
			{
				// Receive data from the raw accel queue
				Axis3i16 accelRaw = {0};
				isDataReceived &= readAccelRawQueue(&accelRaw);

				// Receive data from the raw gyro queue
				Axis3i16 gyroRaw = {0};
				isDataReceived &= readGyroRawQueue(&gyroRaw);

				// Serialise relevant data into block of uint8_t bytes
				if (isDataReceived == true)
				{
					// Serialise header
					offset = put_u8(buffer, offset, (uint8_t)currentDataType);
					offset = put_u32_le(buffer, offset, packetCounter);
					offset = put_u64_le(buffer, offset, imuTimestamp);

					// Serialise data 
					offset = put_i16_le(buffer, offset, accelRaw.x);
					offset = put_i16_le(buffer, offset, accelRaw.y);
					offset = put_i16_le(buffer, offset, accelRaw.z);
					offset = put_i16_le(buffer, offset, gyroRaw.x);
					offset = put_i16_le(buffer, offset, gyroRaw.y);
					offset = put_i16_le(buffer, offset, gyroRaw.z);
				}
				else 
				{
					// If all necessary data for the packet was not received:
					DEBUG_PRINTE("Failed to receive all raw IMU data for the packet");
					continue;
				}
				break;
			}
			case processedImuData:
			{
				// Receive data from the processed accel queue
				Axis3f accelProcessed = {0};
				isDataReceived &= readAccelProcessedQueue(&accelProcessed);

				// Receive data from the processed gyro queue
				Axis3f gyroProcessed = {0};
				isDataReceived &= readGyroProcessedQueue(&gyroProcessed);

				// Serialise relevant data into block of uint8_t bytes
				if (isDataReceived == true)
				{
					// Serialise header
					offset = put_u8(buffer, offset, (uint8_t)currentDataType);
					offset = put_u32_le(buffer, offset, packetCounter);
					offset = put_u64_le(buffer, offset, imuTimestamp);

					// Serialise data
					offset = put_f32_le(buffer, offset, accelProcessed.x);
					offset = put_f32_le(buffer, offset, accelProcessed.y);
					offset = put_f32_le(buffer, offset, accelProcessed.z);
					offset = put_f32_le(buffer, offset, gyroProcessed.x);
					offset = put_f32_le(buffer, offset, gyroProcessed.y);
					offset = put_f32_le(buffer, offset, gyroProcessed.z);
				}
				else 
				{
					// If all necessary data for the packet was not received:
					DEBUG_PRINTE("Failed to receive all processed IMU data for the packet");
					continue;
				}
				break;
			}
			case kalmanStateData:
			{
				// Receive data from the kalman-filtered state queue
				state_t state = {0};
				isDataReceived &= readKalmanStateQueue(&state);

				// Serialise relevant data into block of uint8_t bytes
				if (isDataReceived == true)
				{
					// Serialise header
					offset = put_u8(buffer, offset, (uint8_t)currentDataType);
					offset = put_u32_le(buffer, offset, packetCounter);
					offset = put_u64_le(buffer, offset, imuTimestamp);

					// Serialise data
					offset = put_u32_le(buffer, offset, state.position.timestamp);
					offset = put_f32_le(buffer, offset, state.position.x);
					offset = put_f32_le(buffer, offset, state.position.y);
					offset = put_f32_le(buffer, offset, state.position.z);
					offset = put_u32_le(buffer, offset, state.attitude.timestamp);
					offset = put_f32_le(buffer, offset, state.attitude.roll);
					offset = put_f32_le(buffer, offset, state.attitude.pitch);
					offset = put_f32_le(buffer, offset, state.attitude.yaw);
				}
				else 
				{
					// If all necessary data for the packet was not received:
					DEBUG_PRINTE("Failed to receive kalman-filtered state data for the packet");
					continue;
				}
				break;
			}
			default:
			{
				// If the requested UART data type is not valid:
				DEBUG_PRINTE("Invalid UART data type requested");
				continue;
			}
		}

		// Compute CRC over buffer[0..offset-1]
		uint16_t crc = crc16_ccitt(buffer, offset);

		// Append CRC to data buffer (little-endian)
		offset = put_u16_le(buffer, offset, crc);  // offset now includes CRC

		// COBS-encode into packet buffer
		size_t encLen = cobs_encode(buffer, offset, packet);

		// Append a trailing 0x00 delimiter
		packet[encLen++] = 0x00;

		// If the UART TX mutex is available
		if (uartTxLock() == pdTRUE)
		{
			// Try write to UART FIFO buffer
			writeResult = uartTryWrite(packet, encLen, &hint);

			// Try give the UART TX mutex
			if (uartTxUnlock() == pdFALSE)
			{
				// Notify user via debug if giving mutex fails
				DEBUG_PRINTE("Failed to give UART TX mutex");
			}

			// If the write attempt was successful
			if (writeResult == true)
			{
				// Increment packet counter
				packetCounter += 1;

				// If FIFO buffer did not empty since the last packet
				if (hint == uartBufferHasSpace)
				{
					// Warn user via debug that packets are not clearing fast enough 
					DEBUG_PRINTW("Packet queued, but FIFO buffer was not empty");
				}
			}
			// If the write attempt was unsuccessful
			else
			{
				// If buffer was too full to queue the next packet
				if (hint == uartBufferTooFull)
				{
					// Notify user of the error via debug
					DEBUG_PRINTE("Packet not queued, FIFO buffer too full");
				}
				// If packet is too long to ever fit in the buffer
				else if (hint == uartDataTooLong)
				{
					// Notify user of the error via debug
					DEBUG_PRINTE("Packet not queued, packet longer than FIFO buffer");
				}
			}
		}
		// If the UART TX mutex is unavailable
		else
		{
			// Notify the user via debug
			DEBUG_PRINTE("Failed to take UART TX mutex");
		}
		
	}
}

/* ------------------------------ Public APIs ------------------------------- */

/**
 * @brief Initialize the IMU_UART module and start its FreeRTOS task.
 *
 * Safe to call multiple times; subsequent calls are no-ops once initialised.
 * Creates the task that will package IMU data and send it over UART.
 *
 * @note Must be called after the sensor subsystem is initialized since
 *     the task reads from the accelerometer/gyroscope queues.
 */
void imuuartInit(void)
{
	// Return if imuuartInit() has already been called
	if (isInit) 
	{
		return;
	}

	// Initialise variable to hold potential error
	esp_err_t error;

	// Initialise the UART hardware controller
	error = uartesp32Init();

	// If UART hardware controller fails to initialise successfully
	if (error != ESP_OK) 
	{
		// Notify user of error via debug
		DEBUG_PRINTE("Failed to initialise UART hardware controller");

		// Set the isInit flag to false, since the UART initialisation failed
		isInit = false;
	}
	else
	{
		// Notify user of successful UART hardware initialisation
		DEBUG_PRINTI("Successfully initialised UART hardware controller");

		// Ensure the mutex for UART writes has been initialised
		uartTxLockInit();

		// Create the IMU_UART task (with PRIORITY=2), which runs the imuuartTask() function
		STATIC_MEM_TASK_CREATE(imuuartTask, imuuartTask, IMU_UART_TASK_NAME,NULL, IMU_UART_TASK_PRI);

		// Set the isInit flag to true, since the UART initialisation is complete
		isInit = true;
	}
}

/**
 * @brief Check if the IMU→UART module has initialised successfully.
 *
 * Safe to call multiple times; subsequent calls are no-ops once initialized.
 * Creates the task that will package IMU data and send it over UART.
 *
 */
bool imuuartTest(void)
{
	// Return the current value of the isInit flag
	return isInit;
}

/* ------------------------------- Parameters ------------------------------- */

// Register the UART data type as a parameter of the UART group
PARAM_GROUP_START(uart)
PARAM_ADD(PARAM_UINT8, uartDataType, &uartDataType)
PARAM_GROUP_STOP(uart)

/* ------------------------------- Module End ------------------------------- */