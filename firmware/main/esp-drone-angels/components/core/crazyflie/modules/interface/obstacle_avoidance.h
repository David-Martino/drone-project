/**
 * @file obstacle_avoidance.c
 * @brief Avoids obstacles, maybe.
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
 * This module sends a hover setpoint if the range to an obstacle
 * in the direction of motion is less than some threshold.
 *
 * @author Nathan Mayhew <23065159@student.uwa.edu.au>
 * @date 12 October 2025
 * 
 * @copyright
 *   © 2025 Nathan Mayhew
 *   Licensed under GPLv3.0; see the LICENSE file
 */



#include "stabilizer.h"

/* Sends hover command if an obstacle is within the threshold*/
void obstacleAvoidanceUpdateSetpoint(setpoint_t *setpoint, state_t *state);

// dry af