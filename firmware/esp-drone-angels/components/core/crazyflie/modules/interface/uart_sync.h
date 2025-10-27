/* ------------------------------ Module Start ------------------------------ */

/**
 * @file uart_sync.h
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

#ifndef __UART_SYNC_H__
#define __UART_SYNC_H__

/* -------------------------------- Includes -------------------------------- */

// Standard includes
#include <stdbool.h>

/* ------------------------------ Public APIs ------------------------------- */

/**
 * @brief Initialise the UART sync module and start its FreeRTOS task
 * 
 * Safe to call multiple times; subsequent calls are no-ops once initialised.
 * Creates the task that will respond to UART sync requests from the Pi.
 */
void uartsyncInit(void);

/**
 * @brief Check if the UART sync module initialised correctly
 * 
 * @retval true		The UART sync module successfully completed running its init
 * @retval false	The UART sync module failed to complete running its init
 */
bool uartsyncTest(void);

/**
 * @brief Notify the UART sync module that the UART sync ISR fired
 * 
 * This is to be called by the UART sync ISR.
 */
void uartsyncNotify(void);

#endif

/* ------------------------------- Module End ------------------------------- */