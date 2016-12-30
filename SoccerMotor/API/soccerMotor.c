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

#define `$INSTANCE_NAME`_ENCODER_ZERO_COUNT 0x8000

void `$INSTANCE_NAME`_Start(uint32 CPR_new, uint16 CallPeriod_new)
{
    //Start PWM and quadrature decoder
    `$INSTANCE_NAME`_PWM_Start();
    `$INSTANCE_NAME`_QuadDec_Start();
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

int `$INSTANCE_NAME`_userCount = 0;
int `$INSTANCE_NAME`_pidCount = 0;

// Note: The counter only counts up to +/-0x800 (32768). On the newer motors this 
// corresponds to 87.6 rotations and the older motors this corresponds to 1.65
// Therefore to avoid overflow we will reset the encoder counter each time the
// count is read and use software to keep track of the counts.
int `$INSTANCE_NAME`_ProcessEncoderCount()
{
    int count = ((int)`$INSTANCE_NAME`_QuadDec_ReadCounter()) - `$INSTANCE_NAME`_ENCODER_ZERO_COUNT;
	`$INSTANCE_NAME`_userCount += count;
	`$INSTANCE_NAME`_pidCount += count;
	`$INSTANCE_NAME`_QuadDec_TriggerCommand(`$INSTANCE_NAME`_QuadDec_MASK, `$INSTANCE_NAME`_QuadDec_CMD_RELOAD);
}

// Read user encoder count. Resets after read.
int `$INSTANCE_NAME`_ReadEncoderCount()
{
	`$INSTANCE_NAME`_ProcessEncoderCount();
	int count = `$INSTANCE_NAME`_userCount;
	`$INSTANCE_NAME`_userCount = 0;
	return count;
}