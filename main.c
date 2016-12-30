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
#include "stdbool.h"

//The tick period in ms.
#define TICK_PERIOD  20

//How many counts per wheel rotation. If 0, doesn't use PID.
#define CPR 512 * 9.7

// Callback for the printf function
void putdata(void* p, char c)
{
	UART_UartPutChar(c);
}

//SERIAL COMMUNICATION (Planning to put into separate file)
int uart_pos = 0;
#define UART_BUFFER_SIZE 10
char buffer[UART_BUFFER_SIZE];

int getCommandLength(char cmd)
{
	switch(cmd)
	{
	case '1': //Set PWM
		return 2;
	default:
		return 0;
	}
}

void processCommand(char cmd)
{
	int val;
	switch(cmd)
	{
	case '1': //Set PWM
		val = (buffer[1] - '0')*25;
		SoccerMotor1_SetPower(val);
		SoccerMotor2_SetPower(val);
		SoccerMotor3_SetPower(val);
		printf("Motor power set to %d\r\n", val);
		break;		
	default:
		printf("'%c' is not a valid command character\r\n", cmd);
	}
}

void recieveChar(char ch)
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

//The system timer ISR.
volatile uint8 tick_Flag = 0;
CY_ISR(Tick_ISR)
{
	uint32 mask = Tick_Timer_GetInterruptSourceMasked();
	tick_Flag = 1;
	Tick_Timer_ClearInterrupt(mask);
}

CY_ISR(UART_ISR)
{
	// If multiple characters are sent at once it is possible for only
	// one interrupt to be generated. In this case we want to completely
	// clear the FIFO, so we use a while loop.
	while(UART_SpiUartGetRxBufferSize())
		recieveChar(UART_UartGetChar());
		
	// Clear interrupt (this avoids an infinite loop!)
	UART_ClearRxInterruptSource(UART_INTR_RX_NOT_EMPTY);
}

int main()
{
    float Kp = 3;
    float Ki = 0.9;

    //Start the motor controllers.
    SoccerMotor1_Start(CPR, TICK_PERIOD);
    SoccerMotor2_Start(CPR, TICK_PERIOD);
    SoccerMotor3_Start(CPR, TICK_PERIOD);
    
    //SoccerMotor1_SetPIDConstants(Kp, Ki, 0);
    //SoccerMotor2_SetPIDConstants(Kp, Ki, 0);
    //SoccerMotor3_SetPIDConstants(Kp, Ki, 0);
    
    //Start the serial port.
    UART_Start();
	UART_INT_StartEx(UART_ISR);
    
    //Initialize the printf function to use the serial port.
    init_printf(NULL, putdata);
    
    //Start the timer and ISR.
//    Tick_Timer_WritePeriod(US_IN_MS * TICK_PERIOD);
//    Tick_Timer_Start();tb
//    Tick_INT_StartEx(Tick_ISR);

    CyDelay(1000);
    printf("Hello there!\n");
    
    //Enable global interrupts.
    CyGlobalIntEnable;
    
    for(;;)
    {
        //Every 10ms.
        if(tick_Flag)
        {
            //Clear the flag.
            tick_Flag = 0;
            
            //Update the speed of all of the motors.
            //SoccerMotor1_UpdateSpeed(0);
            //SoccerMotor2_UpdateSpeed(M2_Speed);
            //SoccerMotor3_UpdateSpeed(M3_Speed);
            
            //Check for timer overflows.
            if(tick_Flag)
                printf("Timer overflow!\n");
        }
    }
}