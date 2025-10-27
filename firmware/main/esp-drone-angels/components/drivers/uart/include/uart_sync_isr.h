/* ------------------------------ Module Start ------------------------------ */

/**
 * @file uart_sync_isr.h
 * @brief Public APIs for the UART sync ISR.
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
 * This file contains public APIs for the UART sync ISR.
 *
 * @author Erwin Bauernschmitt <22964301@student.uwa.edu.au>
 * @date 30 September 2025
 * 
 * @copyright
 *   © 2025 Erwin Bauernschmitt
 *   Licensed under GPLv3.0; see the LICENSE file
 */

/* ----------------------------- Include Guard ------------------------------ */

#ifndef __UART_SYNC_ISR_H__
#define __UART_SYNC_ISR_H__

/* -------------------------------- Includes -------------------------------- */

// Standard includes
#include <stdint.h>
#include <stdbool.h>

// ESP-IDF includes
#include "driver/uart.h"
#include "driver/gpio.h"

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
esp_err_t uartsyncisrInit(gpio_num_t rxPin);

 /**
 * @brief Receive one UART sync request timestamp from the queue.
 *
 * Copies the latest UART sync request's start-bit timestamp (in microseconds) 
 * from it's queue into @p timestamp.
 *
 * @note This call consumes one item from the queue; call it at most once per
 *       UART sync request.
 *
 * @param[out] timestamp Pointer to the variable that receives the timestamp.
 *
 * @retval true  A timestamp was available and written to @p timestamp.
 * @retval false No timestamp was available; @p timestamp is not modified.
 */
bool readUartSyncTimeQueue(uint64_t *timestamp);

#endif 

/* ------------------------------- End Module ------------------------------- */
