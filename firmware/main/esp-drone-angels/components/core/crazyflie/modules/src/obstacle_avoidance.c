/**
 * @file obstacle_avoidance.c
 * @brief Low-level obstacle avoidance from multiranger data and zranger.
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
 * Does NOT account for edge case of obstacles either side of drone simultaneously.
 *
 * @author Nathan Mayhew <23065159@student.uwa.edu.au>
 * @date 12 October 2025
 * 
 * @copyright
 *   © 2025 Nathan Mayhew
 * 
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#include "stabilizer.h"
#include "range.h"
#include "obstacle_avoidance.h"
#include "debug_cf.h"
#include "commander.h"

#define DEBUG_MODULE "LLOA"

/** Hysteresis:
 * 
 * /|           :      :
 * /|           :      :    
 * /|           :      :
 * /|           :      :
 * /|           :      :
 *      THRESH    DHYST
 * 
 * Trigger if range < thresh
 * Detrigger if range > thresh + dhyst
 * 
 */

#define THRESH       300 // OA triggers for ranges below this if velocity > MIN_VEL
#define THRESH_CRIT   200 // OA triggers for ranges below this, without considering velocity.
#define DHYST        50 // OA detriggers for ranges greater than THRESH+DHYST
#define MIN_VEL      0.1f // Minum velocity of OA (m/s)
#define MIN_ALTITUDE   0.1 // Minimum altitude (m)
#define RUN_VEL      0.1f // Speed to back away from object (m/s)
static uint8_t triggered; // State machine variable for OA event (1 = OA active, 0 = OA inactive)

void obstacleAvoidanceSetVelSetpoint(setpoint_t *setpoint, float vx, float vy, float vz);
uint8_t obstacleAvoidanceRangeVelocityCheck(uint8_t *threats, uint32_t limit, float vx, float vy, float vz);
uint8_t obstacleAvoidanceRangeCheck(uint8_t *threats, uint32_t limit);

/**
 * @brief checks for obstacles and will set setpoint to back away if needed
 * @param setpoint pointer to setpoint that will be passed into controller
 * @param state pointer to state output from estimator. Must have a velocity estimate
 */
void obstacleAvoidanceUpdateSetpoint(setpoint_t *setpoint, state_t *state) 
{

    float vxNew = 0.0f;
    float vyNew = 0.0f; // could store these in vec struct if desired.
    float vzNew = 0.0f;
    
    uint8_t threats[5];  // [F, B, L, R, U] Down excluded. Cannot use an enum since multiple objects could be detected (in theory)

    /*       Data collection         */

    /**
     * Just use state velocity data.
     * 
     * Wanted to do some future proofing by using the current setpoint to predict if we were
     * going to hit an obstacle using a higher thresh, but the setpoints may be in either world 
     * or body frame. In the former case, would need to translate the world velocity to a body 
     * frame. I don't fully understand the transformation (Chat says that the inverse of the 
     * attitude quat should be used, but I would have thought that the normal quaternion should 
     * be used.) so don't want to rely on that for testing.
     *              _     _
     * Stretch goal  \_O_/ 
     * 
     */

    // Get state velocity data -- for *detective* avoidance
    float vx = state->velocity.x;
    float vy = state->velocity.y; 
    float vz = state->velocity.z;

    /*                  OA State Machine                */

    /**
     * vxNew, vyNew, vzNew will become the new velocities if an object is detected.
     * 
     * The OA behaviour is to move away from the object at 0.2m/s, and then remain still
     * 
     * TODO - May have to re-enable HL commander if CFlib is being used, but ROS doesn't use HL.
     * ROS not using the HL won't affect anything if we re-enable the HL, since the HL will just get
     * disabled as soon as a non-HL command is sent
     */
    // Only trigger OA if drone is above minimum altitude
    if (state->position.z > MIN_ALTITUDE) {  // TODO: verify that state estimation functions even for rollover condition. Otherwise, up sensor will always trigger OA when rolled over on floor.
        if (triggered) {
            // OA is already active

            if (obstacleAvoidanceRangeCheck(threats, THRESH+DHYST)) {
                // Update state variable (stays the same so not necessary here)
                triggered = 1;

                // Build velocity vector command
                if (threats[0]) {vxNew = -RUN_VEL;}; // Front
                if (threats[1]) {vxNew = RUN_VEL;};  // Back
                if (threats[2]) {vyNew = -RUN_VEL;}; // Left
                if (threats[3]) {vyNew = RUN_VEL;};  // Right
                if (threats[4]) {vzNew = -RUN_VEL;}; // Up

                // Set velocity setpoint
                obstacleAvoidanceSetVelSetpoint(setpoint, vxNew, vyNew, vzNew);  
            }
            else {
                // Update State Variable
                triggered = 0;
                
                // Set hover setpoint
                obstacleAvoidanceSetVelSetpoint(setpoint, 0, 0, 0);

                // commanderRelaxPriority(); // need for HL commander to resume control.
            };
        }
        else {
            // OA is not active

            uint8_t velcheck = obstacleAvoidanceRangeVelocityCheck(threats, THRESH, vx, vy, vz);
            uint8_t critcheck = obstacleAvoidanceRangeCheck(threats, THRESH_CRIT); // called in this order so critcheck overrides threats.

            if (velcheck || critcheck) {
                // Update State variable
                triggered = 1;

                //build velocity vector from threats
                if (threats[0]) {vxNew = -RUN_VEL;}; // Front
                if (threats[1]) {vxNew = RUN_VEL;};  // Back
                if (threats[2]) {vyNew = -RUN_VEL;}; // Left
                if (threats[3]) {vyNew = RUN_VEL;};  // Right
                if (threats[4]) {vzNew = -RUN_VEL;}; // Up

                // Send velocity setpoint
                obstacleAvoidanceSetVelSetpoint(setpoint, vxNew, vyNew, vzNew);
            };
        };
    };
}


/** 
 * @brief Creates velocity setpoint based on input velocity vector
 * @param setpoint pointer to the setpoint structure being passed to the controller
 * @param vx state x velocity (m/s)
 * @param vy state y velocity (m/s)
 * @param vz state z velocity (m/s)
 */
void obstacleAvoidanceSetVelSetpoint(setpoint_t *setpoint, float vx, float vy, float vz)
{
    setpoint->mode.x = modeVelocity;
    setpoint->mode.y = modeVelocity;
    setpoint->mode.z = modeVelocity;
    setpoint->mode.roll = modeDisable;
    setpoint->mode.pitch = modeDisable;
    setpoint->mode.yaw = modeDisable;
    setpoint->velocity.x = vx;
    setpoint->velocity.y = vy;
    setpoint->velocity.z = vz;

    // I don't think I even need to relinquish command back to the high level commander? since I am operating on the setpoint after commander framework anyway.
}

/** 
 * @brief Checks all directions for object
 * @return 1 if triggered, 0 if not triggered
 * @param threats pointer to array storing which direction has been triggered in the order [Front, Back, Left, Right, Up]
 * @param limit any ranges below this will get triggered
 * @param vx state x velocity (m/s)
 * @param vy state y velocity (m/s)
 * @param vz state z velocity (m/s)
 */
uint8_t obstacleAvoidanceRangeVelocityCheck(uint8_t *threats, uint32_t limit, float vx, float vy, float vz) {
    uint8_t localTriggered = 0;

    // x-axis
    if (vx > MIN_VEL) {
        if (rangeGet(rangeFront) < limit) {
            threats[0] = 1;
            localTriggered = 1;
        };
    }
    else if (vx < -MIN_VEL) {
        if (rangeGet(rangeBack) < limit) {
            threats[1] = 1;
            localTriggered = 1;
        };
    };

    // y-axis
    if (vy > MIN_VEL) {
        if (rangeGet(rangeLeft) < limit) {
            threats[2] = 1;
            localTriggered = 1;
        };
    }
    else if (vy < -MIN_VEL) {
        if (rangeGet(rangeRight) < limit) {
            threats[3] = 1;
            localTriggered = 1;

        };
    };
    
    // z-axis
    if (vz > MIN_VEL) {
        if (rangeGet(rangeUp) < limit) {
            threats[4] = 1;
            localTriggered = 1;
        };
    };

    return localTriggered;
};

/** 
 * @brief Checks all directions for object and 
 * @return 1 if triggered, 0 if not triggered
 * @param threats pointer to array storing which direction has been triggered in the order [Front, Back, Left, Right, Up]
 * @param limit any ranges below this will get triggered
 */
uint8_t obstacleAvoidanceRangeCheck(uint8_t *threats, uint32_t limit) {
    uint8_t localTriggered = 0;

    // x-axis
    if (rangeGet(rangeFront) < limit) {
        threats[0] = 1;
        localTriggered = 1;
    };
    if (rangeGet(rangeBack) < limit) {
        threats[1] = 1;
        localTriggered = 1;
    };

    // y-axis
    if (rangeGet(rangeLeft) < limit) {
        threats[2] = 1;
        localTriggered = 1;
    };
    if (rangeGet(rangeRight) < limit) {
        threats[3] = 1;
        localTriggered = 1;

    };

    // z-axis
    if (rangeGet(rangeUp) < limit) {
        threats[4] = 1;
        localTriggered = 1;
    };

    return localTriggered;
};