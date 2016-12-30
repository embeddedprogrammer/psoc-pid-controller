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

// TICK period
int `$INSTANCE_NAME`_PERIOD_MS = 0;

// Quadrature counting
int `$INSTANCE_NAME`_count = 0;
int `$INSTANCE_NAME`_lastUserCount = 0;
int `$INSTANCE_NAME`_lastInternalCount = 0;
float `$INSTANCE_NAME`_velocity = 0;

//PID control
float `$INSTANCE_NAME`_qpps = 1000;
float `$INSTANCE_NAME`_Kp = 1;
float `$INSTANCE_NAME`_Ki = 0;
float `$INSTANCE_NAME`_Kd = 0;
float `$INSTANCE_NAME`_desiredSpeed = 0;
bool `$INSTANCE_NAME`_pidControlEnabled = false;


// **************** INTERNAL FUNCTIONS (USER FUNCTIONS AT BOTTOM OF FILE) ****************

// Sets PWM duty cycle between -255 to +255.
void `$INSTANCE_NAME`_SetPowerInternal(float power)
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
}

// Note: The counter only counts up to +/-0x800 (32768). On the newer motors this 
// corresponds to 87.6 rotations and the older motors this corresponds to 1.65
// Therefore to avoid overflow we will reset the encoder counter each time the
// count is read and use software to keep track of the counts.
void `$INSTANCE_NAME`_ProcessEncoderCount()
{
    int count = ((int)`$INSTANCE_NAME`_QuadDec_ReadCounter()) - `$INSTANCE_NAME`_ENCODER_ZERO_COUNT;
	`$INSTANCE_NAME`_count += count;
	`$INSTANCE_NAME`_QuadDec_TriggerCommand(`$INSTANCE_NAME`_QuadDec_MASK, `$INSTANCE_NAME`_QuadDec_CMD_RELOAD);
}

// Read internal encoder count. Resets count after read.
int `$INSTANCE_NAME`_ReadInternalCount()
{
	`$INSTANCE_NAME`_ProcessEncoderCount();
	int count = `$INSTANCE_NAME`_count - `$INSTANCE_NAME`_lastInternalCount;
	`$INSTANCE_NAME`_lastInternalCount = `$INSTANCE_NAME`_count;
	return count;
}

void `$INSTANCE_NAME`_updateVelocity()
{
	int diff = `$INSTANCE_NAME`_ReadInternalCount();
	`$INSTANCE_NAME`_velocity = diff * 1000 / `$INSTANCE_NAME`_PERIOD_MS;
}

void `$INSTANCE_NAME`_pidControl()
{
	float power = `$INSTANCE_NAME`_desiredSpeed*`$INSTANCE_NAME`_Kp*255/`$INSTANCE_NAME`_qpps; // + `$INSTANCE_NAME`_Ki + `$INSTANCE_NAME`_Kd;
	`$INSTANCE_NAME`_SetPowerInternal(power);
}

// **************** TOP-LEVEL USER API FUNCTIONS ****************

// Sets up the component. This should be called before any other API fuction for this component.
void `$INSTANCE_NAME`_Start(int period_ms)
{
    //Start PWM and quadrature decoder
    `$INSTANCE_NAME`_PWM_Start();
    `$INSTANCE_NAME`_QuadDec_Start();
	
	//Store period
	`$INSTANCE_NAME`_PERIOD_MS = period_ms;
}

// Disengages the motor control by writing a low to the enable pin. 
// This means that each motor terminal will be floating/high-impedance, which allows
// the motor to spin freely. (Of course there will still be a lot of mechanical resistance
// due to the gearbox and so forth, but there will be no electrical resistance)
void `$INSTANCE_NAME`_Disengage()
{
	//Turn off PID control
	`$INSTANCE_NAME`_pidControlEnabled = false;
	
	//Set H-bridge enable pin
	`$INSTANCE_NAME`_EN_Write(false);
}

// Set power (betweeen -255 and +255). Does not use PID control.
void `$INSTANCE_NAME`_SetPower(float power)
{
	//Turn off PID control
	`$INSTANCE_NAME`_pidControlEnabled = false;
	
	//Set PWM duty cycle
	`$INSTANCE_NAME`_SetPowerInternal(power);
	
	//Set H-bridge enable pin
	`$INSTANCE_NAME`_EN_Write(true);
}

// Set speed
void `$INSTANCE_NAME`_SetSpeed(float speed)
{
	//Turn on PID control
	`$INSTANCE_NAME`_pidControlEnabled = true;
	`$INSTANCE_NAME`_desiredSpeed = speed;
	
	//Set H-bridge enable pin
	`$INSTANCE_NAME`_EN_Write(true);
}

// Set PID constants
void `$INSTANCE_NAME`_SetPIDConstants(int fullSpeedCountsPerSecond, int Kp, int Ki, int Kd)
{
	`$INSTANCE_NAME`_qpps = fullSpeedCountsPerSecond;
	`$INSTANCE_NAME`_Kp = Kp;
	`$INSTANCE_NAME`_Ki = Ki;
	`$INSTANCE_NAME`_Kd = Kd;
}

// Read user encoder count. Resets count after read.
int `$INSTANCE_NAME`_ReadEncoderCount()
{
	`$INSTANCE_NAME`_ProcessEncoderCount();
	int count = `$INSTANCE_NAME`_count - `$INSTANCE_NAME`_lastUserCount;
	`$INSTANCE_NAME`_lastUserCount = `$INSTANCE_NAME`_count;
	return count;
}

// Get velocity in counts/second. The velocity is updated each time the tick function is called.
float `$INSTANCE_NAME`_GetVelocity()
{
	return `$INSTANCE_NAME`_velocity;
}

// Update velocity, process PID control, etc.
void `$INSTANCE_NAME`_tick()
{
	`$INSTANCE_NAME`_updateVelocity();
	if(`$INSTANCE_NAME`_pidControlEnabled)
		`$INSTANCE_NAME`_pidControl();
}
