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

#include "project.h"
#include "serial.h"

#define UART_BUFFER_SIZE 20
#define CLOCK_FREQ_KHZ 32

char buffer[UART_BUFFER_SIZE];
int uart_pos = 0;

#define DEBUG_SERIAL

// This macro simply erases all the printd functions if DEBUG_SERIAL is not defined.
// This frees 3% of the flash.
#ifdef DEBUG_SERIAL
	#define printd(...) printf(__VA_ARGS__)
#else
	#define printd(...)
#endif

uint32 unpack(char* ptr, int pos)
{
	return (((uint32)ptr[pos + 0]) << 24) |
			(((uint32)ptr[pos + 1]) << 16) |
			(((uint32)ptr[pos + 2]) << 8) |
			(((uint32)ptr[pos + 3]));
}

void pack(char* ptr, int pos, uint32 val)
{
	ptr[pos + 0] = (val >> 24) & 0xff;
	ptr[pos + 1] = (val >> 16) & 0xff;
	ptr[pos + 2] = (val >> 8) & 0xff;
	ptr[pos + 3] = val & 0xff;
}

// Note: Directly converting uint to float doesn't work for negative numbers so we must first convert it to an int.
// Also: These unpack and pack functions are similar to the python struct.pack and struct.unpack functions
float unpack_f(char* ptr, int pos)
{
	return ((float)(int)unpack(ptr, pos))/1000;
}

void pack_f(char* ptr, int pos, float val)
{
	pack(ptr, pos, (uint32)(int)(val*1000));
}

void sendResponse(char* buffer, int length)
{
	int i;
	for(i = 0; i < length; i++)
		UART_UartPutChar(buffer[i]);
}

void processCommand(char cmd)
{
	float val1, val2, val3, qpps, Kp, Ki, period_ms, tau_ms;
	int motor;
	switch(cmd)
	{
	case 'p': //Set power
		val1 = unpack_f(buffer, 1);
		val2 = unpack_f(buffer, 5);
		val3 = unpack_f(buffer, 9);
		SoccerMotor1_SetPower(val1);
		SoccerMotor2_SetPower(val2);
		SoccerMotor3_SetPower(val3);
		printd("Motor power set to %d, %d, %d\r\n", (int)val1, (int)val2, (int)val3);
		break;
	case 's': //Control with PID values
		val1 = unpack_f(buffer, 1);
		val2 = unpack_f(buffer, 5);
		val3 = unpack_f(buffer, 9);
		SoccerMotor1_SetSpeed(val1);
		SoccerMotor2_SetSpeed(val2);
		SoccerMotor3_SetSpeed(val3);
		printd("Motor speed set to %d, %d, %d counts per second\r\n", (int)val1, (int)val2, (int)val3);
		break;
	case 'k': //Set PID constants
		motor = (buffer[1] - '0');
		Kp = unpack_f(buffer, 2);
		Ki = unpack_f(buffer, 6);
		qpps = unpack_f(buffer, 10);
		if(motor == 1 || motor == 0)
			SoccerMotor1_SetPIDConstants(Kp, Ki, qpps);
		if(motor == 2 || motor == 0)
			SoccerMotor2_SetPIDConstants(Kp, Ki, qpps);
		if(motor == 3 || motor == 0)
			SoccerMotor3_SetPIDConstants(Kp, Ki, qpps);
		printd("PID constants set for motor %d. Kp = %d, Ki = %d, qpps = %d\r\n", motor, (int)Kp, (int)Ki, (int)qpps);
		break;
	case 't': //Set time constants (tick period and velocity filter corner frequency)
		period_ms = unpack_f(buffer, 1);
		tau_ms = unpack_f(buffer, 5);
		Tick_Timer_WritePeriod(CLOCK_FREQ_KHZ * period_ms);
		SoccerMotor1_SetTickPeriodAndTau(period_ms, tau_ms);
		SoccerMotor2_SetTickPeriodAndTau(period_ms, tau_ms);
		SoccerMotor3_SetTickPeriodAndTau(period_ms, tau_ms);
		printd("Time constants set. period = %d ms, tau = %d ms\r\n", (int)period_ms, (int)tau_ms);
		break;
	case 'v': //Get velocity
		val1 = SoccerMotor1_GetVelocity();
		val2 = SoccerMotor2_GetVelocity();
		val3 = SoccerMotor3_GetVelocity();
		pack_f(buffer, 0, val1);
		pack_f(buffer, 4, val2);
		pack_f(buffer, 8, val3);
		sendResponse(buffer, 12);
		printd("Motor velocity: %d %d %d\r\n", (int)val1, (int)val2, (int)val3);
		break;
	case 'e': //Read encoder counts
		val1 = SoccerMotor1_ReadEncoderCount();
		val2 = SoccerMotor2_ReadEncoderCount();
		val3 = SoccerMotor3_ReadEncoderCount();
		pack_f(buffer, 0, val1);
		pack_f(buffer, 4, val2);
		pack_f(buffer, 8, val3);
		sendResponse(buffer, 12);
		printd("Encoder Counts: %d %d %d\r\n", (int)val1, (int)val2, (int)val3);
		break;
	case 'd': // Disengage
		SoccerMotor1_Disengage();
		SoccerMotor2_Disengage();
		SoccerMotor3_Disengage();
		printd("Motors disengaged\r\n", cmd);
		break;
	default:
		printd("'%c' is not a valid command character\r\n", cmd);
		break;
	}
}

int getCommandLength(char cmd)
{
	switch(cmd)
	{
	case 'p': //Set power
		return 13;
		break;
	case 's': //Control with PID values
		return 13;
		break;
	case 'k': //Set PID constants
		return 14;
		break;
	case 't': //Set time constants (tick period and velocity filter corner frequency)
		return 9;
		break;		
	case 'v': //Get velocity
		return 1;
		break;
	case 'e': //Read encoder counts
		return 1;
		break;
	case 'd': // Disengage
		return 1;
		break;
	default:
		return 0;
		break;
	}
}

void serial_recieveChar(char ch)
{
	if(ch == '\n' || ch == '\r')
		uart_pos = 0;
	else
	{
		buffer[uart_pos++] = ch;
		if(uart_pos >= getCommandLength(buffer[0]))
		{
			processCommand(buffer[0]);
			uart_pos = 0;
		}
	}
}
