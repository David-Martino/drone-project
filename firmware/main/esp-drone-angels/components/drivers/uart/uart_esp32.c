/* ------------------------------ Module Start ------------------------------ */

/**
 * @file uart_esp32.c
 * @brief Implements the functionality for the ESP32 UART driver.
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
 * uart_esp32.h, providing the core logic for initialising and using an ESP32
 * UART hardware controller.
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
#include <stdbool.h>

// App-specific includes
#include "uart_esp32.h"

// ESP-IDF includes
#include "driver/uart.h"
 
/* -------------------------- Private Declarations -------------------------- */

// Initialise a flag to check if initialisation has already been completed
static bool isInit = false;

// Initialise handle and queue for a UART mutex to prevent concurrent writes
static SemaphoreHandle_t uartMutex;
static StaticSemaphore_t uartMutexBuffer;

/* ------------------------------- Public APIs ------------------------------ */

/**
 * @brief Initialise the UART controller.
 * 
 * Call this once before starting the IMU UART task. If successfull, subsequent
 * calls are no-ops.
 */
esp_err_t uartesp32Init(void)
{
  // Check if UART controller has already been initialised
  if (isInit == true)
  {
    // Return ESP_OK since 
    return ESP_OK;
  }

  // Initialise variable to store potential error
  esp_err_t error;

  // Create the UART configuration
  const uart_config_t cfg = {
    .baud_rate  = ESP_UART_BAUD,
    .data_bits  = UART_DATA_8_BITS,
    .parity     = UART_PARITY_DISABLE,
    .stop_bits  = UART_STOP_BITS_1,
    .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
    .source_clk = UART_SCLK_DEFAULT,
  };

  // Apply the UART configuration to the UART controller being used
  error = uart_param_config(ESP_UART_PORT, &cfg);

  // Return if failed
  if (error != ESP_OK) return error;

  // Set the IO pins being used for the selected UART controller
  error = uart_set_pin(ESP_UART_PORT, ESP_UART_TX_PIN, ESP_UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

  // Return if failed
  if (error != ESP_OK) return error;

  // Install the UART driver
  error = uart_driver_install(ESP_UART_PORT, ESP_UART_RX_BUF, ESP_UART_TX_BUF, 0, NULL, 0);
  
  // Return if failed
  if (error != ESP_OK) return error;

  // Create the UART mutex so that only one task can write packets at a time 
  uartMutex = xSemaphoreCreateMutexStatic(&uartMutexBuffer);

  // Set the isInit flag to prevent re-initialisation
  isInit = true;

  // Return final value of error variable (ESP_OK)
  return error;
}

/**
 * @brief Queue a buffer for UART transmission (non-blocking).
 * 
 * Writes @p data[0:len-1] to the UART FIFO buffer for transmission, with 
 * optional information about the state of the FIFO buffer being written to
 * @p hint (pass NULL if not used).
 * 
 * @param[in]  data	Pointer to buffer containing data to transmit
 * @param[in]  len	Number of bytes into buffer to transmit
 * @param[out] hint Pointer to write hint about FIFO buffer state
 * 
 * @retval true  if data was successfully written to the UART FIFO buffer. 
 * @retval false if writing to the UART FIFO buffer failed.
 */
bool uartTryWrite(const uint8_t* data, size_t len, UartTxHint* hint)
{
  // Take the UART mutex to prevent other tasks from interrupting packet write
  if (xSemaphoreTake(uartMutex, 0) == pdFALSE)
  {
    // Check if hint is being used
    if (hint != NULL)
    {
      // Notify calling function that the UART mutex is unavailable
      *hint = uartWriteClash;
    }
    
    // Return false since data was not successfully written to the buffer
    return false; 
  }

  // Check if data is too long to ever fit in TX FIFO ring buffer
  if (len > ESP_UART_RX_BUF) 
  {
    // Check if hint is being used
    if (hint != NULL)
    {
      // Notify calling function that the data is too long
      *hint = uartDataTooLong;
    }

    // Give back the UART mutex
    xSemaphoreGive(uartMutex);

    // Return false since data was not successfully written to the buffer
    return false;  
  }

  // Initialise variable to store available space in TX FIFO buffer
  size_t free_space = 0;

  // Get available space in TX FIFO buffer
  (void)uart_get_tx_buffer_free_size(ESP_UART_PORT, &free_space);

  // Check if the buffer is too full to accept the full packet
  if (free_space < len) 
  {
    // Check if hint is being used
    if (hint != NULL)
    {
      // Notify calling function that there is not enough space
      *hint = uartBufferTooFull;
    }

    // Give back the UART mutex
    xSemaphoreGive(uartMutex);
    
    // Return false since data was not successfully written to the buffer
    return false;
  }

  // Check if the buffer has completely cleared
  if (free_space == ESP_UART_RX_BUF)
  {
    // Check if hint is being used
    if (hint != NULL)
    {
      // Notify calling function that the buffer is empty
      *hint = uartBufferIsEmpty;  
    }
  } 
  // If the buffer is not empty but has enough space for the data
  else 
  {
    // Check if hint is being used
    if (hint != NULL) 
    {
      // Notify that the buffer js not empty but has enough space
      *hint = uartBufferHasSpace;
    }
  }

  // Initialise variable to store how many bytes were written
  int written = 0;

  // Write the data to the TX FIFO buffer
  written = uart_write_bytes(ESP_UART_PORT, (const char*)data, (size_t)len);

  // Check if writing all of the data bytes failed somehow
  if (written != (int)len) 
  {
    // Check if hint is being used
    if (hint != NULL)
    {
      // Notify calling function that somehow space ran out mid-write
      *hint = uartBufferTooFull;
    }

    // Give back the UART mutex
    xSemaphoreGive(uartMutex);

    // Return false since not all data was successfully written to the buffer
    return false;
  }

  // Give back the UART mutex
  xSemaphoreGive(uartMutex);

  // Return true since all data was successfully written to the buffer
  return true;
}

/* ------------------------------- Module End ------------------------------- */