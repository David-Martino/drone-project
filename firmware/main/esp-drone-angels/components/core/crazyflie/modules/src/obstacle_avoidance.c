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
 * @author Nathan Mayhew <23065159@student.uwa.edu.au>
 * @date 12 October 2025
 * 
 * @copyright
 *   © 2025 Nathan Mayhew
 *   Licensed under GPLv3.0; see the LICENSE file
 */

#include "stabilizer.h"
#include "range.h"
#include "obstacle_avoidance.h"
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include "math3d.h"
#include "debug_cf.h"
#define DEBUG_MODULE "LLOA"


#define THRESH       200 // was for setpoint based avoidance, now for state based avoidance. TODO: make into a parameter later
#define THRESH_CRIT  150 // not used. TODO: make into a parameter later
#define WAIT         3 // [DEPRECATED] time to wait in seconds, drone was going to hover for 3s after each OA event/
#define MIN_VEL      0.1 // OA will only trigger for v > 0.1

static uint8_t triggered; // State machine variable for OA event (1 = OA avtive, 0 = OA inactive)

void obstacleAvoidanceSetVelSetpoint(setpoint_t *setpoint, float vx, float vy, float vz);

void obstacleAvoidanceUpdateSetpoint(setpoint_t *setpoint, state_t *state) 
{
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

    // Get setpoint data -- for *preventative* avoidance [DEPRECATED. I DON'T HAVE TIME TO UNDERSTAND THE TRANSFORMATION AND THAT MAKES IT TOO UNRELIABLE]
    // first check that the velocity is in the body frame.
    // if (setpoint->velocity_body) {
    //     float vx = setpoint->velocity.x;
    //     float vy = setpoint->velocity.y; 
    //     float vz = setpoint->velocity.z;
    // }
    // else {
    //     // Convert to body frame of reference:
    //     struct quat q = state.attitudeQuaternion;
    //     struct quat q_inv = {.x = -q.x, .y = -q.y, .z = -q.z, .w = q.w};

    //     struct vec v_body = qvrot(q_inv, setpoint->velocity);
    //     float vx = v_body.x;
    //     float vy = v_body.y; 
    //     float vz = v_body.z;
    // };



    /*                  OA Decision Making                 */

    /**
     * vxNew, vyNew, vzNew will become the new velocities if an object is detected.
     * 
     * The OA behaviour is to move away from the object at 0.2m/s, and then remain still
     * 
     * TODO - May have to re-enable HL commander if CFlib is being used, but ROS doesn't use HL.
     * ROS not using the HL won't affect anything if we re-enable the HL, since the HL will just get
     * disabled as soon as a non-HL command is sent
     */

    float vxNew = 0.0f;
    float vyNew = 0.0f; // could store these in vec struct if desired.
    float vzNew = 0.0f;
    
    // x-axis
    if (vx > MIN_VEL) {
        if (rangeGet(rangeFront) < THRESH) {
            vxNew = -0.1f; // Move backwards (-x)
            DEBUG_PRINTI("Front TRIGGERED");
            triggered = 1;
        }
    }
    else if (vx < -MIN_VEL) {
        if (rangeGet(rangeBack) < THRESH) {
            vxNew = 0.1f; // Move forwards (+x)
            DEBUG_PRINTI("back TRIGGERED");
            triggered = 1;
        };
    };
    // y-axis
    if (vy > MIN_VEL) {
        if (rangeGet(rangeLeft) < THRESH) {
            vyNew = -0.1f; // Move Right (-y)
            DEBUG_PRINTI("Left TRIGGERED");
            triggered = 1;
        };
    }
    else if (vy < -MIN_VEL) {
        if (rangeGet(rangeRight) < THRESH) {
            vyNew = 0.1f; // Move Left (+y)
            DEBUG_PRINTI("Right TRIGGERED");
            triggered = 1;

        };
    };
    // z-axis
    if (vz > MIN_VEL) {
        if (rangeGet(rangeUp) < THRESH) {
            vzNew = -0.1f; // Move Down (-z)
            DEBUG_PRINTI("Up TRIGGERED");
            triggered = 1;
        };
    };
    

    /*          Take OA Action              */

    /* If an obstacle was detected, then vxNew, vyNew or vzNew are nonzero, 
    *  and an OA velocity setpoint should be sent.
    *
    * Otherwise, if no obstacle is detected, but OA is still triggered (triggered = 1), 
    * then we must stop the drone so it doesn't continue flying off. Hence OA sets a 0 velocity setpoint
    * [Note: maybe this should become a position setpoint with a relative (0,0,0) setpoint?]
    * 
    * If no obstacle is detected, and OA is not active (triggered = 0), then do nothing.
    * 
    * 
    * The hover seems kind of redundant if the drone immediately begins receiving commands from GCS,
    * but this implementation should guarantee that the drone stops moving if no further commands
    * are being sent.
    */
    if ((vxNew != 0.0f) || (vyNew != 0.0f) || (vzNew != 0.0f)) {

        obstacleAvoidanceSetVelSetpoint(setpoint, vxNew, vyNew, vzNew);
        
        triggered = 1;
    }
    else {
        if (triggered == 1) {
            obstacleAvoidanceSetVelSetpoint(setpoint, 0.0f, 0.0f, 0.0f); // could just pass in vxNew, etc because they should already be zero.
        };

        triggered = 0;
    };
}

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