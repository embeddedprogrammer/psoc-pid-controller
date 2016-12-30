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

// Set power (betweeen -255 and +255)
void `$INSTANCE_NAME`_SetPower(float power);

// Read user encoder count. Resets count after read.
int `$INSTANCE_NAME`_ReadEncoderCount();

// Get velocity in counts/second. The velocity is updated each time the tick function is called.
int `$INSTANCE_NAME`_GetVelocity();

// Update velocity calculation, process PID control, etc.
void `$INSTANCE_NAME`_tick();
    
#endif
