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

#include "`$INSTANCE_NAME`_soccerMotor.h"
#include "stdbool.h"

void `$INSTANCE_NAME`_Start(uint32 CPR_new, uint16 CallPeriod_new)
{
    //Start the motor output PWM component.
    `$INSTANCE_NAME`_PWM_Start();
    
    //Set up the variables for this motor.
    //`$INSTANCE_NAME`_CPR = CPR_new;
    //`$INSTANCE_NAME`_CallPeriod = CallPeriod_new;
    
    //Start the encoder.
    `$INSTANCE_NAME`_QuadDec_Start();
        
    //This is required for the decoder to actually start. Not sure why.
    //`$INSTANCE_NAME`_QuadDec_TriggerCommand(`$INSTANCE_NAME`_QuadDec_MASK, `$INSTANCE_NAME`_QuadDec_CMD_RELOAD);
}

void `$INSTANCE_NAME`_SetPower(float power)
{
	// Check power bounds
	if(power > 255)
		power = 255;
	else if(power < -255)
		power = -255;
	
    //Set the PWM duty cycle.
    if(power > 0)
    {
        `$INSTANCE_NAME`_PWM_WriteCompare1((uint8) power);
        `$INSTANCE_NAME`_PWM_WriteCompare2(0);
    }
    else if(power < 0)
    {
        `$INSTANCE_NAME`_PWM_WriteCompare1(0);
        `$INSTANCE_NAME`_PWM_WriteCompare2((uint8) -power);
    }
    else
	{
        `$INSTANCE_NAME`_PWM_WriteCompare1(0);
        `$INSTANCE_NAME`_PWM_WriteCompare2(0);
    }
	
	//Set H-bridge enable pin
	`$INSTANCE_NAME`_EN_Write(true);
}
 