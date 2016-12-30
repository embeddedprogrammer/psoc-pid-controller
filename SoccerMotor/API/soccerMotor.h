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
    
#include "cydevice_trm.h"
#include "cyfitter.h"
#include <cytypes.h>
#include "project.h"
    
    
/*
-------------------------------
Start()
----------------------
Sets up the component. This should be called
before any other API fuction for this component.

Arguments:
CPR- Counts Per Revolution. If 0, the encoder is not used.
CallPeriod- The time between UpdateSpeed function calls, in milliseconds.

Returns:
void
-------------------------------
*/
void `$INSTANCE_NAME`_Start(uint32 CPR_new, uint16 CallPeriod_new);
    
    
/*
-------------------------------
SetCPR
----------------------
Sets the counts per revolution of the encoder.

If CPR <= 0, the value is ignored. If CPR > 0, 
a PID controller is used to control the motor speed.

Arguments:
CPR_New- An unsigned 32bit number.


Returns:
void
-------------------------------
*/
void `$INSTANCE_NAME`_SetCPR(uint32 CPR_New);


/*
-------------------------------
UpdateSpeed()
----------------------
Sets motor speed.

If CPR > 0, this uses a PID controller to regulate
the motor speed, using the ecoder for feedback. If
CPR <= 0, the speed is used to set the PWM compare
value.

Arguments:
Speed- A signed 16 bit integer. The sign indicates direction,
        where positive is considered forward, and negative 
        is considered reverse.
        If CPR is <= 0 this value must be between +- 255.


Returns:
void
-------------------------------
*/
void `$INSTANCE_NAME`_UpdateSpeed(float newSpeed);

/*
-------------------------------
GetEncoderCount
----------------------
Gets the current count from the encoder.

The count starts at a value of 8000, and is
decremented or incremented from there.

Arguments:


Returns:
encoder count- an unsigned 16 bit count.
-------------------------------
*/

uint16 `$INSTANCE_NAME`_GetEncoderCount();

/*
-------------------------------
GetPower
----------------------
Gets the power of the motor.

Returns: The current pwm duty cycle.
-------------------------------
*/
float `$INSTANCE_NAME`_GetCurrentPower();
/*
-------------------------------
GetActualSpeed
----------------------
Gets the actual speed from the PID loop.

Arguments:

Returns:
actualSpeed- Unknown units.
-------------------------------
*/
float `$INSTANCE_NAME`_GetActualSpeed();

/*
-------------------------------
GetPTerm
----------------------
Gets the current P term.

Returns:
Pterm- The current Pterm value.
-------------------------------
*/
float `$INSTANCE_NAME`_GetPTerm();

/*
-------------------------------
GetITerm
----------------------
Gets the current I term.

Returns:
iterm- The current iTerm value.
-------------------------------
*/
float `$INSTANCE_NAME`_GetITerm();


/*
-------------------------------
SetPIDConstants
----------------------
Sets the constants of the PID controller.


Arguments:
KpNew- An unsigned 16 bit integer.
KiNew- An unsigned 16 bit integer.
KdNew- An unsigned 16 bit integer.

Returns:
-------------------------------
*/
void `$INSTANCE_NAME`_SetPIDConstants(float KpNew, float KiNew, float KdNew);
    
#endif

/* [] END OF FILE */
