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
#define MAX_LOOKUP_TABLE_SIZE 255 // Assumes that only the positive half is recorded
																	// and then mirrored to get negative values

// Stores a given pwm_value with a certain speed
struct lookup_entry_t {
	float pwm_val;
	float speed;
}

// TICK period
float `$INSTANCE_NAME`_Ts;
float `$INSTANCE_NAME`_tau;

// Quadrature counting
int `$INSTANCE_NAME`_count = 0;
int `$INSTANCE_NAME`_lastUserCount = 0;
int `$INSTANCE_NAME`_lastInternalCount = 0;
float `$INSTANCE_NAME`_velocity = 0;

//PID control
float `$INSTANCE_NAME`_Kp = 1;
float `$INSTANCE_NAME`_Ki = 10;
float `$INSTANCE_NAME`_qpps = 1000;
float `$INSTANCE_NAME`_integrator = 0;
float `$INSTANCE_NAME`_desiredSpeed = 0;
bool `$INSTANCE_NAME`_pidControlEnabled = false;
float `$INSTANCE_NAME`_model_pwm_offset = 30.0;
float `$INSTANCE_NAME`_dither_pwm_max = 10.0;
float `$INSTANCE_NAME`_dither_sign = 1;
float `$INSTANCE_NAME`_dither_period = 0.001;
float `$INSTANCE_NAME`_dither_timer = 0;

// Lookup table
bool `$INSTANCE_NAME`_lookup_table_enabled = false;
lookup_entry_t `$INSTANCE_NAME`_lookup_table[MAX_LOOKUP_TABLE_SIZE];


// *********************** INTERNAL FUNCTIONS (USER FUNCTIONS AT BOTTOM OF FILE) ***********************

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

// Update velocity. Use the dirty derivative (a discrete-time tustin-approximation of a LPF)
// to smooth the measured wheel velocity. The parameter tau corresponds to the corner frequency of the LPF.
void `$INSTANCE_NAME`_updateVelocity()
{
	int diff = `$INSTANCE_NAME`_ReadInternalCount();
	float tau = `$INSTANCE_NAME`_tau;
	float Ts = `$INSTANCE_NAME`_Ts;
	`$INSTANCE_NAME`_velocity = (2*tau-Ts)/(2*tau+Ts)*`$INSTANCE_NAME`_velocity + 2/(2*tau+Ts)*diff;
	//`$INSTANCE_NAME`_velocity = diff / Ts;
}

// Note: Since we are controlling velocity this is actually a 1st order system,
// thus we only need a PI controller.
void `$INSTANCE_NAME`_pidControl()
{
	// Calc error
	float error = `$INSTANCE_NAME`_desiredSpeed - `$INSTANCE_NAME`_velocity;

	// Limit integrator
	`$INSTANCE_NAME`_integrator += error*`$INSTANCE_NAME`_Ki*`$INSTANCE_NAME`_Ts;
	float integratorMax = `$INSTANCE_NAME`_qpps;
	if(`$INSTANCE_NAME`_integrator > integratorMax)
		`$INSTANCE_NAME`_integrator = integratorMax;
	else if(`$INSTANCE_NAME`_integrator < -integratorMax)
		`$INSTANCE_NAME`_integrator = -integratorMax;

	// Calculate power
	// float power = (`$INSTANCE_NAME`_Kp*`$INSTANCE_NAME`_desiredSpeed + `$INSTANCE_NAME`_integrator)*255/`$INSTANCE_NAME`_qpps;
	float conversion_factor, model_pwm;

	if(`$INSTANCE_NAME`_lookup_table_enabled)
	{
		conversion_factor = `$INSTANCE_NAME`_lookup_table[0].pwm_val/`$INSTANCE_NAME`_lookup_table[0].speed;
		model_pwm = `$INSTANCE_NAME`_getPowerFromLookupTable(`$INSTANCE_NAME`_desiredSpeed);
	}
	else
	{
		conversion_factor = `$INSTANCE_NAME`_lookup_table[0].pwm_val/`$INSTANCE_NAME`_lookup_table[0].speed;
		model_pwm = `$INSTANCE_NAME`_getPowerFromLookupTable(`$INSTANCE_NAME`_desiredSpeed);		
	}

	float PI_pwm = (`$INSTANCE_NAME`_Kp*error + `$INSTANCE_NAME`_integrator)*conversion_factor;

	`$INSTANCE_NAME`_dither_timer += `$INSTANCE_NAME`_Ts;
	if(`$INSTANCE_NAME`_dither_timer >= `$INSTANCE_NAME`_dither_period) {
		`$INSTANCE_NAME`_dither_timer = 0;
		`$INSTANCE_NAME`_dither_sign *= -1;
	}
	float dither_pwm = `$INSTANCE_NAME`_dither_sign*`$INSTANCE_NAME`_dither_pwm_max;
	float power = model_pwm + PI_pwm + dither_pwm;
	`$INSTANCE_NAME`_SetPowerInternal(power);
}

// Uses the lookup table to retrieve an appropriate pwm value for the given desired speed
// -- Assumes that the lookup table goes from high to low speeds
float `$INSTANCE_NAME`_getPowerFromLookupTable(float speed)
{
	float speed_high, speed_low, speed_range, range_proportion, pwm_high, pwm_low, pwm_range, power;
	float abs_speed = fabs(speed);
	int speed_sign = (speed > 0) ? 1 : ((speed < 0) ? -1 : 0);

	if (speed == 0)
	{
			return 0;
	}

	// Find the index just smaller than the desired speed
	int index = 0;
	while (`$INSTANCE_NAME`_lookup_table[index].speed > abs_speed)
	{
		index++;
	}

	// Check for border case
	if(index > 0)
	{
		// Interpolate between points
		speed_high = `$INSTANCE_NAME`_lookup_table[index - 1].speed;
		speed_low = `$INSTANCE_NAME`_lookup_table[index].speed;
		speed_range = speed_high - speed_low;

		range_proportion = (speed - speed_low)/speed_range;

		pwm_high = `$INSTANCE_NAME`_lookup_table[index - 1].pwm_val;
		pwm_low = `$INSTANCE_NAME`_lookup_table[index].pwm_val;
		pwm_range = pwm_high - pwm_low;
		power = pwm_low + range_proportion*pwm_range;
	}
	else
	{
		// If we didn't find a higher speed, just return the highest one recorded
		power = `$INSTANCE_NAME`_lookup_table[0].speed;
	}

	return power*speed_sign;
}

// ******************************** TOP-LEVEL USER API FUNCTIONS ********************************

// Sets up the component. This should be called before any other API fuction for this component.
void `$INSTANCE_NAME`_Start(int period_ms, int tau_ms)
{
    //Start PWM and quadrature decoder
    `$INSTANCE_NAME`_PWM_Start();
    `$INSTANCE_NAME`_QuadDec_Start();

	//Store parameters
	`$INSTANCE_NAME`_SetTickPeriodAndTau(period_ms, tau_ms);
}

// Modify the tick period (for the timer) and tau (velocity filter corner frequency)
void `$INSTANCE_NAME`_SetTickPeriodAndTau(int period_ms, int tau_ms)
{
	//Store parameters
	`$INSTANCE_NAME`_Ts = period_ms/1000.0;
	`$INSTANCE_NAME`_tau = tau_ms/1000.0;
}

void `$INSTANCE_NAME`_enableLookupTable()
{
	`$INSTANCE_NAME`_lookup_table_enabled = true;
}

void `$INSTANCE_NAME`_storeLookupValue(float pwm, float speed, int index)
{
	lookup_entry_t entry;
	entry.pwm_val = pwm;
	entry.speed = speed;

	`$INSTANCE_NAME`_lookup_table[index] = entry;
}

// Disengage the motor control by writing a low to the enable pin.
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
	if(!`$INSTANCE_NAME`_pidControlEnabled)
	{
		// Re-initialize PID control - if integrator and pwm values aren't zero it could cause motor to jerk.
		`$INSTANCE_NAME`_integrator = 0;
		`$INSTANCE_NAME`_pidControlEnabled = true;
		`$INSTANCE_NAME`_pidControl();

		// Set H-bridge enable pin
		`$INSTANCE_NAME`_EN_Write(true);
	}

	// Set desired speed
	`$INSTANCE_NAME`_desiredSpeed = speed;
}

// Set PID constants. The qpps is the maximum quadrature pulses per second under expected load.
void `$INSTANCE_NAME`_SetPIDConstants(float Kp, float Ki, float qpps)
{
	`$INSTANCE_NAME`_Kp = Kp;
	`$INSTANCE_NAME`_Ki = Ki;
	`$INSTANCE_NAME`_qpps = qpps;
}

// Set PID constants. The qpps is the maximum quadrature pulses per second under expected load.
void `$INSTANCE_NAME`_SetAdvancedConstants(float offset, float dither_max, float dither_period)
{
	`$INSTANCE_NAME`_model_pwm_offset = offset;
	`$INSTANCE_NAME`_dither_pwm_max = dither_max;
	`$INSTANCE_NAME`_dither_period = dither_period;
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
