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
// CPR- Counts Per Revolution. If 0, the encoder is not used.
// CallPeriod- The time between UpdateSpeed function calls, in milliseconds.
void `$INSTANCE_NAME`_Start(uint32 CPR_new, uint16 CallPeriod_new);

// Set power (betweeen -255 and +255)
void `$INSTANCE_NAME`_SetPower(float power);
    
#endif
