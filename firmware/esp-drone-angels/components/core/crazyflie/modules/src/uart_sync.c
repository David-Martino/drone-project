/* ------------------------------ Module Start ------------------------------ */

/**
 * @file uart_sync.c
 * @brief Implements the functionality for the ESP32->Pi UART sync response.
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
 * uart_sync.h, providing the core logic for responding to the sync requests 
 * sent from the Pi via UART.
 *
 * @author Erwin Bauernschmitt <22964301@student.uwa.edu.au>
 * @date 5 October 2025
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

// App-specific includes
#include "uart_sync.h"
#include "uart_sync_isr.h"
#include "config.h"
#include "imu_uart.h"
#include "uart_helpers.h"
#include "uart_esp32.h"
#include "static_mem.h"

// FreeRTOS includes
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "task.h"

// ESP-IDF includes
#include "esp_timer.h"
#include "driver/gpio.h"

// Debug includes and defines
#define DEBUG_MODULE "UART_SYNC"
#include "debug_cf.h"

/* ----------------------------- Configurations ----------------------------- */

// Define the size of the header
#define HEADER_SIZE 1

// Define the size of the timestamp data
#define TIME_DATA_SIZE (2 * 8)

// Define CRC16 size in bytes
#define CRC_SIZE 2

// Define max data payload size in bytes
#define MAX_DATA_SIZE (HEADER_SIZE + TIME_DATA_SIZE + CRC_SIZE)

// Define max frame size after COBS encoding in bytes
#define MAX_FRAME_SIZE (MAX_DATA_SIZE + (1 + ((MAX_DATA_SIZE - 1) / 254)))

// Define max wire packet size after appending packet delimiter
#define MAX_PACKET_SIZE (MAX_FRAME_SIZE + 1)

/* -------------------------- Private Declarations -------------------------- */

// Create initialisation flag to be set to true when uartsyncInit() completes
static bool isInit = false;

// Preallocate static memory for the UART_SYNC task
STATIC_MEM_TASK_ALLOC(uartsyncTask, UART_SYNC_TASK_STACKSIZE);

/* ----------------------------- FreeRTOS Tasks ----------------------------- */

/**
 * @brief FreeRTOS task that responds to UART sync requests with high priority.
 * 
 * The task runs indefinitely. It receives the latest UART sync request's 
 * start-bit timestamp, builds a frame, and writes to the UART driver. 
 * Designed to be created by uartsyncInit().
 *
 * @param[in] param Optional context pointer (unused; passed NULL).
 */
static void uartsyncTask(void *param)
{
	// Initialise variable for tracking if all data successfully received
	bool isDataReceived = true;

	// Initialise variable for storing UART sync request's start-bit timestamp 
	uint64_t receivedTime = 0;

	// Initialise variable for storing the current time 
	uint64_t currentTime = 0;

	// Initialise a buffer to hold the flattened data
	uint8_t buffer[MAX_DATA_SIZE] = {0};

	// Initialise a buffer to hold the complete encoded data packet
	uint8_t packet[MAX_PACKET_SIZE] = {0};

	// Initialise a variable to hold the try write boolean
	bool writeResult = false;

	// Initialise a variable to hold the buffer offset 
	size_t offset = 0;

	// Initialise a variable to hold the amount of free space in the FIFO buffer
	size_t freeSpace = 0;

	while (true)
	{
		// Receive timestamp from ISR, blocking until one available
		isDataReceived &= readUartSyncTimeQueue(&receivedTime);

		// If the timestamp data was successfully received
		if (isDataReceived == true)
		{
			// Serialise the data type (max value reserved for sync response)
			offset = put_u8(buffer, offset, 255);

			// Serialise the UART sync request's start-bit timestamp
			offset = put_u64_le(buffer, offset, receivedTime);

			// Get the current time as late as possible
			currentTime = (uint64_t)esp_timer_get_time();

			// Serialise the current time
			offset = put_u64_le(buffer, offset, currentTime);

			// Compute CRC over buffer[0..offset-1]
			uint16_t crc = crc16_ccitt(buffer, offset);

			// Append CRC to data buffer (little-endian)
			offset = put_u16_le(buffer, offset, crc);  // offset now includes CRC

			// COBS-encode into packet buffer
			size_t encLen = cobs_encode(buffer, offset, packet);

			// Append a trailing 0x00 delimiter
			packet[encLen++] = 0x00;

			// Get available space in TX FIFO buffer
			(void)uart_get_tx_buffer_free_size(ESP_UART_PORT, &freeSpace);

			// If the TX FIFO buffer is empty
			if (freeSpace == ESP_UART_TX_BUF)
			{
				// If the UART TX mutex is available
				if (uartTxLock() == pdTRUE)
				{
					// Try write to UART FIFO buffer
					writeResult = uartTryWrite(packet, encLen, NULL);

					// Try give the UART TX mutex
					if (uartTxUnlock() == pdFALSE)
					{
						// Notify user via debug if giving mutex fails
						DEBUG_PRINTE("Failed to give UART TX mutex");
					}

					// If the write attempt failed
					if (writeResult != true)
					{
						// Notify the user of the error via debug
						DEBUG_PRINTE("Packet not queued, failed to write to FIFO");
					}
				}
				// If the UART TX mutex is unavailable
				else
				{
					// Notify the user via debug
					DEBUG_PRINTE("Failed to take UART TX mutex");
				}
			}
			// If the FIFO buffer was not empty by write time
			else
			{
				// Notify the user of the error via debug
				DEBUG_PRINTE("Packet not queued, FIFO not empty");
			}
		}
		// If the timestamp data was not successfully received
		else
		{
			// Notify the user of the error via debug
			DEBUG_PRINTE("Failed to receive timestamp data for the packet");
		}

		// Reset the offset variable after sending
		offset = 0;

		// Reset the boolean for receiving all necessary data after sending
		isDataReceived = true;

		// Reset the data buffer after sending
		memset(buffer, 0, sizeof(buffer));

		// Reset the packet buffer after sending
		memset(packet, 0, sizeof(packet));
	}
}

/* ------------------------------ Public APIs ------------------------------- */

/**
 * @brief Initialise the UART sync module and start its FreeRTOS task
 * 
 * Safe to call multiple times; subsequent calls are no-ops once initialised.
 * Creates the task that will respond to UART sync requests from the Pi.
 */
void uartsyncInit(void)
{
	// Return if uartsyncInit() has already been called
	if (isInit) 
	{
		return;
	}

	// Initialise variable to hold potential error
	esp_err_t error;

	// Initialise the UART sync ISR
	error = uartsyncisrInit(ESP_UART_RX_PIN);

	// If the UART sync ISR fails to initialise sucessfully
	if (error != ESP_OK)
	{
		// Notify user of error via debug
		DEBUG_PRINTE("Failed to initialise UART sync ISR");

		// Set the isInit flag to false, since the UART initialisation failed
		isInit = false;
	}
	// If the UART sync ISR initialises successfully
	else
	{
		// Notify user of successful UART sync ISR initialisation
		DEBUG_PRINTI("Successfully initialised UART sync ISR");

		// Ensure the mutex for UART writes has been initialised
		uartTxLockInit();

		// Create the UART_SYNC task (with PRIORITY=8), which runs the uartsyncTask() function
		STATIC_MEM_TASK_CREATE(uartsyncTask, uartsyncTask, UART_SYNC_TASK_NAME,NULL, UART_SYNC_TASK_PRI);

		// Set the isInit flag to true, since the UART sync initialisation is complete
		isInit = true;
	}
}

/**
 * @brief Check if the UART sync module initialised correctly
 * 
 * @retval true		The UART sync module successfully completed running its init
 * @retval false	The UART sync module failed to complete running its init
 */
bool uartsyncTest(void)
{
	// Return the current value of isInit
	return isInit;
}

/* ------------------------------- Module End ------------------------------- */