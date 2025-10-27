/* ------------------------------ Module Start ------------------------------ */

/**
 * @file imu_uart.h
 * @brief Public APIs for the ESP32->Pi UART communication.
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
 * This file contains public APIs for transmitting IMU or state data from the to
 * ESP32 to the Pi via UART.
 *
 * @author Erwin Bauernschmitt <22964301@student.uwa.edu.au>
 * @date 30 September 2025
 * 
 * @copyright
 *   © 2025 Erwin Bauernschmitt
 *   Licensed under GPLv3.0; see the LICENSE file
 */

/* ----------------------------- Include Guard ------------------------------ */

#ifndef __IMU_UART_H__
#define __IMU_UART_H__

/* -------------------------------- Includes -------------------------------- */

// Standard includes
#include <stdbool.h>

/* ------------------------------ Public APIs ------------------------------- */

/**
 * @brief Initialise the IMU→UART module and start its FreeRTOS task.
 *
 * Safe to call multiple times; subsequent calls are no-ops once initialised.
 * Creates the task that will package IMU data and send it over UART.
 */
void imuuartInit(void);

/**
 * @brief Check if the IMU→UART module has initialised successfully.
 * 
 * @retval true		The UART module successfully completed running imuuartInit()
 * @retval false	The UART module failed to complete running imuuartInit()
 */
bool imuuartTest(void);

#endif

/* ------------------------------- Module End ------------------------------- */