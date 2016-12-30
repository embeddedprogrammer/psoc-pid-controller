/* ========================================
 *
 * Copyright YOUR COMPANY, THE YEAR
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 * CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF your company.
 *
 * ========================================
*/

#ifndef `$INSTANCE_NAME`_SOCCER_MOTOR_H
#define `$INSTANCE_NAME`_SOCCER_MOTOR_H
    
#include "project.h"
    
// Sets up the component. This should be called before any other API fuction for this component.
void `$INSTANCE_NAME`_Start(int period_ms);

// Disengages the motor control by writing a low to the enable pin. 
// This means that each motor terminal will be floating/high-impedance, which allows
// the motor to spin freely. (Of course there will still be a lot of mechanical resistance
// due to the gearbox and so forth, but there will be no electrical resistance)
void `$INSTANCE_NAME`_Disengage();

// Set power (betweeen -255 and +255). Does not use PID control
void `$INSTANCE_NAME`_SetPower(float power);

// Set speed using PID control
void `$INSTANCE_NAME`_SetSpeed(float speed);

// Set PID constants
void `$INSTANCE_NAME`_SetPIDConstants(int fullSpeedCountsPerSecond, int Kp, int Ki, int Kd);

// Read user encoder count. Resets count after read.
int `$INSTANCE_NAME`_ReadEncoderCount();

// Get velocity in counts/second. The velocity is updated each time the tick function is called.
float `$INSTANCE_NAME`_GetVelocity();

// Update velocity calculation, process PID control, etc.
void `$INSTANCE_NAME`_tick();
    
#endif
