/* ------------------------------ Module Start ------------------------------ */

/**
 * @file uart_sync.c
 * @brief Implements the functionality of the ISR for UART sync requests.
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
 * uart_sync.h, providing the core logic for the ISR that detects sync request
 * frames sent from the Pi via UART.
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

// App-specific includes
#include "uart_sync_isr.h"

// FreeRTOS includes
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

// ESP-IDF includes
#include "esp_timer.h"
#include "driver/gpio.h"

/* -------------------------- Private Declarations -------------------------- */

// Create queue handle for UART sync request timestamp
static QueueHandle_t uartSyncTimeQueue = NULL;

// Static allocation of FreeRTOS queue storage for UART sync request timestamp
DRAM_ATTR static uint8_t uartSyncTimeQueueStorage[sizeof(uint64_t)];

// Static allocation of FreeRTOS control block for UART sync request timestamp
DRAM_ATTR static StaticQueue_t uartSyncTimeQueueControl;

/* ---------------------------------- ISRs ---------------------------------- */

/**
 * @brief The falling-edge ISR that captures the start-bit timestamp
 * 
 * Kept as short as possible. This saves to the timestamp queue using ISR-safe
 * queue overwriting, with pxHigherPriorityTaskWoken = pdTRUE. The FreeRTOS 
 * task for responding to the sync request has high priority. This ISR requests
 * a context switch to that high-priority task upon completion to minimise the 
 * ESP32's processing time during the response. 
 */
static void IRAM_ATTR uartsyncisr(void)
{
	// Get the current timestamp
	uint64_t timestamp = (uint64_t)esp_timer_get_time();

	// Initialise a "higher priority task is being woken" variable
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	
	// Save the timestamp to the queue, overwriting if not empty
	(void)xQueueOverwriteFromISR(uartSyncTimeQueue, &timestamp, &xHigherPriorityTaskWoken); 
	
	// If the unblocked UART_SYNC task has higher priority than the current task
	if (xHigherPriorityTaskWoken == pdTRUE)
	{
		// Request context switch on ISR exit
		portYIELD_FROM_ISR();
	}
}	

/* ------------------------------ Public APIs ------------------------------- */

/**
 * @brief Initialise a falling-edge ISR on the given UART RX pin.
 * 
 * Call this once before starting the UART_SYNC task.
 * 
 * @param[in] rxPin	The RX pin to attach the ISR to.
 * 
 * @return ESP_OK if succesful, other esp_err_t type if unsuccessful.
 */
esp_err_t uartsyncisrInit(gpio_num_t rxPin)
{
	// If static queue not yet created
  if (uartSyncTimeQueue == NULL)
  {
		// Create the queue for UART sync request timestamp 
		uartSyncTimeQueue = xQueueCreateStatic(1, sizeof(uint64_t), uartSyncTimeQueueStorage, &uartSyncTimeQueueControl);
  }

	// Initialise variable to store potential error
  esp_err_t error;

	// Create the RX pin and interrupt type (negative edge) configuration
	gpio_config_t rxConfig = {
		.pin_bit_mask = (1ULL << rxPin),
		.mode         = GPIO_MODE_INPUT,
		.pull_up_en   = GPIO_PULLUP_ENABLE,
		.pull_down_en = GPIO_PULLDOWN_DISABLE,
		.intr_type    = GPIO_INTR_NEGEDGE,
	};

  // Apply the RX pin configuration to the RX pin being used
  error = gpio_config(&rxConfig);

  // Return if failed
  if (error != ESP_OK) return error;

	// Attach the sync request ISR to the UART RX pin
	error = gpio_isr_handler_add(rxPin, uartsyncisr, NULL);

	/// Return final value of error variable (ESP_OK)
  return error;
}

 /**
 * @brief Receive one UART sync request timestamp from the queue (blocking).
 *
 * Copies the latest UART sync request's start-bit timestamp (in microseconds) 
 * from it's queue into @p timestamp, blocking until one is available.
 *
 * @param[out] timestamp Pointer to the variable that receives the timestamp.
 *
 * @retval true  A timestamp was available and written to @p timestamp.
 * @retval false No timestamp was available; @p timestamp is not modified.
 */
bool readUartSyncTimeQueue(uint64_t *timestamp)
{
	// Receive from the queue into timestamp, blocking until value available
	return (pdTRUE == xQueueReceive(uartSyncTimeQueue, timestamp, portMAX_DELAY));
}

/* ------------------------------- End Module ------------------------------- */