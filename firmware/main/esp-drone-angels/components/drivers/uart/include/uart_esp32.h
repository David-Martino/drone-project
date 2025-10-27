/* ------------------------------ Module Start ------------------------------ */

/**
 * @file uart_esp32.h
 * @brief Public APIs for the ESP32 UART driver.
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
 * This file contains public APIs for the ESP32 UART driver.
 *
 * @author Erwin Bauernschmitt <22964301@student.uwa.edu.au>
 * @date 30 September 2025
 * 
 * @copyright
 *   © 2025 Erwin Bauernschmitt
 *   Licensed under GPLv3.0; see the LICENSE file
 */

/* ----------------------------- Include Guard ------------------------------ */

#ifndef __UART_ESP32_H__
#define __UART_ESP32_H__

/* -------------------------------- Includes -------------------------------- */

// Standard includes
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

// ESP-IDF includes
#include "driver/uart.h"

/* ----------------------------- Configurations ----------------------------- */

// Define which UART controller to use (UART2)
#ifndef ESP_UART_PORT
#define ESP_UART_PORT  UART_NUM_2
#endif

// Define the baud rate
#ifndef ESP_UART_BAUD
#define ESP_UART_BAUD  1000000
#endif

// Define the size of the TX FIFO ring buffer in bytes (256 min?)
#ifndef ESP_UART_TX_BUF
#define ESP_UART_TX_BUF  256   
#endif

// Define the size of the RX FIFO ring buffer in bytes (256 min?)
#ifndef ESP_UART_RX_BUF
#define ESP_UART_RX_BUF  256   
#endif

// Define which IO pin to use for TX
#ifndef ESP_UART_TX_PIN
#define ESP_UART_TX_PIN  39
#endif

// Define which IO pin to use for RX
#ifndef ESP_UART_RX_PIN
#define ESP_UART_RX_PIN  38
#endif

/* ------------------------------ Public APIs ------------------------------- */

// Define an enum for the different UART TX states
enum UartTxHint_e 
{
	uartBufferIsEmpty = 0,  // The TX FIFO buffer was completely empty 
	uartBufferHasSpace,		// The buffer was not empty but had enough space
	uartBufferTooFull,		// The buffer was not empty and data doesn't fit
	uartDataTooLong,		// Data is longer than the TX FIFO buffer size
	uartTxResultCount,  	// Sentinel, not a real UART TX result
};
typedef enum UartTxHint_e UartTxHint;

/**
 * @brief Initialize the UART used to talk to the ESP32 bridge.
 * 
 * Call this once before starting the IMU UART task.
 * 
 * @return ESP_OK if succesful, other esp_err_t type if unsuccessful.
 */
esp_err_t uartesp32Init(void);

/**
 * @brief Queue a buffer for UART transmission (non-blocking).
 * 
 * Writes @p data[0:len-1] to the UART FIFO buffer for transmission.
 * 
 * @param[in] data	Pointer to buffer containing data to transmit
 * @param[in] len		Number of bytes into buffer to transmit
 * @param[out] hint Pointer to write hint about FIFO buffer state
 * 
 * @retval true  if data was successfully written to the UART FIFO buffer. 
 * @retval false if writing to the UART FIFO buffer failed.
 */
bool uartTryWrite(const uint8_t* data, size_t len, UartTxHint* hint);

#endif

/* ------------------------------- Module End ------------------------------- */