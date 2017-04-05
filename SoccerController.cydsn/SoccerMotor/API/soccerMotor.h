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
void `$INSTANCE_NAME`_Start(int period_ms, int tau_ms);

// Modify the tick period (for the timer) and tau (velocity filter corner frequency)
void `$INSTANCE_NAME`_SetTickPeriodAndTau(int period_ms, int tau_ms);

// Disengages the motor control by writing a low to the enable pin.
// This means that each motor terminal will be floating/high-impedance, which allows
// the motor to spin freely. (Of course there will still be a lot of mechanical resistance
// due to the gearbox and so forth, but there will be no electrical resistance)
void `$INSTANCE_NAME`_Disengage();

// Set power (betweeen -255 and +255). Does not use PID control
void `$INSTANCE_NAME`_SetPower(float power);

// Set speed using PID control
void `$INSTANCE_NAME`_SetSpeed(float speed);

// Set PID constants. The qpps is the maximum quadrature pulses per second under expected load.
void `$INSTANCE_NAME`_SetPIDConstants(float Kp, float Ki, float qpps);

// Set model and dither constants. offset and dither max are in PWM, period is in seconds
void `$INSTANCE_NAME`_SetAdvancedConstants(float offset, float dither_max, float dither_period);

// Finds an appropriate PWM value to match the desired speed input
float `$INSTANCE_NAME`_getPowerFromLookupTable(float speed);

// Enables/disables the lookup table model
void `$INSTANCE_NAME`_enableLookupTable(float enable);

// Store a pwm/speed pair in the lookup table at the given index
// **NOTE** Assumes that highest values are stored first
void `$INSTANCE_NAME`_storeLookupValue(float pwm, float speed, int index);

// Read user encoder count. Resets count after read.
int `$INSTANCE_NAME`_ReadEncoderCount();

// Get velocity in counts/second. The velocity is updated each time the tick function is called.
float `$INSTANCE_NAME`_GetVelocity();

// Update velocity calculation, process PID control, etc.
void `$INSTANCE_NAME`_tick();

#endif
