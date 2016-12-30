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

//Constants.
#define `$INSTANCE_NAME`_ENCODER_ZERO_COUNT 0x8000
#define `$INSTANCE_NAME`_MAX_POWER 255

//Global variables.
uint32 `$INSTANCE_NAME`_CPR = 0; //Counts per revolution of the encoder.
uint16 `$INSTANCE_NAME`_CallPeriod = 10; //How much time between calling UpdateSpeed, in milliseconds.
uint16 `$INSTANCE_NAME`_EncoderCount = 0;   //Last read encoder count.
float `$INSTANCE_NAME`_ActualSpeed = 0;

//Local function prototypes.
void `$INSTANCE_NAME`_PIDController(float DesiredSpeed);
void `$INSTANCE_NAME`_ResetEncoderCount();
void `$INSTANCE_NAME`_SetPower(float power);
float `$INSTANCE_NAME`_CheckPowerBounds(float boundlessPower);
inline uint16 `$INSTANCE_NAME`_ReadEncoderCount();

//PID constants - units of %PWM*sec/rad
float `$INSTANCE_NAME`_Kp = 0.1;
float `$INSTANCE_NAME`_Ki = 0;
float `$INSTANCE_NAME`_Kd = 0;


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
void `$INSTANCE_NAME`_Start(uint32 CPR_new, uint16 CallPeriod_new)
{
    //Start the motor output PWM component.
    `$INSTANCE_NAME`_PWM_Start();
    
    //Set up the variables for this motor.
    `$INSTANCE_NAME`_CPR = CPR_new;
    `$INSTANCE_NAME`_CallPeriod = CallPeriod_new;
    
    //Start the encoder.
    `$INSTANCE_NAME`_QuadDec_Start();
        
    //This is required for the decoder to actually start. Not sure why.
    `$INSTANCE_NAME`_QuadDec_TriggerCommand(`$INSTANCE_NAME`_QuadDec_MASK, `$INSTANCE_NAME`_QuadDec_CMD_RELOAD);
        
    
    return;
}


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
void `$INSTANCE_NAME`_UpdateSpeed(float newSpeed)
{
    if(`$INSTANCE_NAME`_CPR > 0) //Use PID.
    {
        `$INSTANCE_NAME`_PIDController(newSpeed);
    }
    //Otherwise, map the input to an appropriate output.
    else{
        `$INSTANCE_NAME`_SetPower(newSpeed);
        
    }
    
    return;

}

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
void `$INSTANCE_NAME`_SetCPR(uint32 CPR_New)
{
    `$INSTANCE_NAME`_CPR = CPR_New;
    
    return;
}

/*
-------------------------------
PIDController
----------------------
Manages the speed of the motor using a simple PID controller.

This is only called internally. It is not visible outside of
the component.

Arguments:
DesiredSpeed- A signed 16 bit integer.
              The desired rotational speed of the motor in RPM.

Returns:
void
-------------------------------
*/
float `$INSTANCE_NAME`_currentPower = 0;
float `$INSTANCE_NAME`_pTerm = 0;
float `$INSTANCE_NAME`_iTerm = 0;
const int `$INSTANCE_NAME`_hysterisis = 10;
void `$INSTANCE_NAME`_PIDController(float desiredSpeed)
{   
    //Get the current encoder count.
    float count = `$INSTANCE_NAME`_ReadEncoderCount();
    
    //Reset the encoder count.
    `$INSTANCE_NAME`_ResetEncoderCount();
    
    //Figure out how much the motor has moved. Units of rad/sec
    //The literals are for conversion between seconds and milliseconds as well as 2pi.
    float PulsesSinceLastRead = -((float)`$INSTANCE_NAME`_ENCODER_ZERO_COUNT - count);
    `$INSTANCE_NAME`_ActualSpeed = PulsesSinceLastRead*10*1000/(2*3.142*`$INSTANCE_NAME`_CPR);
    
    //Do some PID magic.
    float powerAdjusted = 0;
    float error = 0;
    
    error =  (desiredSpeed - `$INSTANCE_NAME`_ActualSpeed);
    
    //Calculate the various components.
    `$INSTANCE_NAME`_pTerm = `$INSTANCE_NAME`_Kp * error;
    `$INSTANCE_NAME`_iTerm += `$INSTANCE_NAME`_Ki * error;
    
    //Limit the I term.
    if(`$INSTANCE_NAME`_iTerm > 100)
    {
      `$INSTANCE_NAME`_iTerm = 100;   
    }
    else if(`$INSTANCE_NAME`_iTerm < -100)
    {
        `$INSTANCE_NAME`_iTerm = -100;
    }
  
    
    //Calculate the adjustment value.
    powerAdjusted = `$INSTANCE_NAME`_pTerm + `$INSTANCE_NAME`_iTerm;

    //Set the motor power
    `$INSTANCE_NAME`_currentPower = `$INSTANCE_NAME`_CheckPowerBounds(powerAdjusted);
    
    //Don't send the signal if it is shorter than the switching time of the motor drivers.
    if(`$INSTANCE_NAME`_currentPower < `$INSTANCE_NAME`_hysterisis && `$INSTANCE_NAME`_currentPower > -`$INSTANCE_NAME`_hysterisis)
        `$INSTANCE_NAME`_currentPower = 0;
    
    //Set the motor output power.
    `$INSTANCE_NAME`_SetPower(`$INSTANCE_NAME`_currentPower);
    
    return;
}

/*
-------------------------------
GetPTerm
----------------------
Gets the current P term.

Returns:
Pterm- The current Pterm value.
-------------------------------
*/
float `$INSTANCE_NAME`_GetPTerm()
{
    return `$INSTANCE_NAME`_pTerm;
    
}

/*
-------------------------------
GetITerm
----------------------
Gets the current I term.

Returns:
iterm- The current iTerm value.
-------------------------------
*/
float `$INSTANCE_NAME`_GetITerm()
{
    return `$INSTANCE_NAME`_iTerm;
}

/*
-------------------------------
GetActualSpeed
----------------------
Gets the current count from the encoder.

The count starts at a value of 0x8000, and is
decremented or incremented from there.

Arguments:

Returns:
encoder count- an unsigned 16 bit count.
-------------------------------
*/
float `$INSTANCE_NAME`_GetActualSpeed()
{
    return `$INSTANCE_NAME`_ActualSpeed;
}
/*
-------------------------------
GetEncoderCount
----------------------
Gets the current count from the encoder.

The count starts at a value of 0x8000, and is
decremented or incremented from there.

Arguments:

Returns:
encoder count- an unsigned 16 bit count.
-------------------------------
*/
inline uint16 `$INSTANCE_NAME`_GetEncoderCount()
{
    return `$INSTANCE_NAME`_EncoderCount;

}

/*
-------------------------------
ReadEncoderCount
----------------------
Gets the most recent count from the encoder.

The count starts at a value of 0x8000, and is
decremented or incremented from there.

Arguments:

Returns:
encoder count- an unsigned 16 bit count.
-------------------------------
*/
inline uint16 `$INSTANCE_NAME`_ReadEncoderCount()
{
    `$INSTANCE_NAME`_EncoderCount = `$INSTANCE_NAME`_QuadDec_ReadCounter();
    
    return `$INSTANCE_NAME`_EncoderCount;

}

/*
-------------------------------
ResetEncoderCount
----------------------
Resets the encoder count to 0x8000.

This is only called internally. It is not visible outside of
the component.

Arguments:

Returns:
-------------------------------
*/

inline void `$INSTANCE_NAME`_ResetEncoderCount()
{
    `$INSTANCE_NAME`_QuadDec_TriggerCommand(`$INSTANCE_NAME`_QuadDec_MASK, `$INSTANCE_NAME`_QuadDec_CMD_RELOAD);

    return;
}

/*
-------------------------------
SetPower
----------------------
Sets the power of the motor.

This is only called internally. It is not visible outside of
the component.

Arguments:
power- a float. Positive means forward,
       negative means backwards. (Should be between +-255.)

Returns:
-------------------------------
*/
void `$INSTANCE_NAME`_SetPower(float power)
{
    //Do some bounds checking. 
    power = `$INSTANCE_NAME`_CheckPowerBounds(power);
    
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
    else{
        `$INSTANCE_NAME`_PWM_WriteCompare1(0);
        `$INSTANCE_NAME`_PWM_WriteCompare2(0);
    }
    
    return;

}

/*
-------------------------------
GetPower
----------------------
Gets the power of the motor.

Returns: The current pwm duty cycle.
-------------------------------
*/
float `$INSTANCE_NAME`_GetCurrentPower()
{
    return `$INSTANCE_NAME`_currentPower;
    
}

/*
-------------------------------
SetPIDConstants
----------------------
Sets the constants of the PID controller.

Arguments:
KpNew- A float.
KiNew- A float.
KdNew- A float.

Returns:
-------------------------------
*/
void `$INSTANCE_NAME`_SetPIDConstants(float KpNew, float KiNew, float KdNew)
{
    `$INSTANCE_NAME`_Kp = KpNew;
    `$INSTANCE_NAME`_Ki = KiNew;
    `$INSTANCE_NAME`_Kd = KdNew;
    
    return;
    
}

/*
-------------------------------
CheckPowerBounds
----------------------
Boundless power isn't good for anyone.
Make sure that the motors power always
stayes within values for which it is defined!


Arguments:
boundlessPower- int16 value.

Returns:
boundedPower- int16 value between -255 and 255.
-------------------------------
*/
float `$INSTANCE_NAME`_CheckPowerBounds(float boundlessPower)
{
    float boundedPower = 0;
    
    //Do some bounds checking. 
    if(boundlessPower > (float)`$INSTANCE_NAME`_MAX_POWER)
        boundedPower = `$INSTANCE_NAME`_MAX_POWER;
    else if(boundlessPower < (float)-`$INSTANCE_NAME`_MAX_POWER)
        boundedPower = (float)-`$INSTANCE_NAME`_MAX_POWER;
    else
        boundedPower = boundlessPower;
    
    return boundedPower;
    
}



/* [] END OF FILE */
