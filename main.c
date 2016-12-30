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
void putdata( void* p, char c)
{
	UART_UartPutChar(c);
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
	{
		char ch = UART_UartGetChar();
		UART_UartPutChar(ch);
	}
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
    
    SoccerMotor1_SetPIDConstants(Kp, Ki, 0);
    SoccerMotor2_SetPIDConstants(Kp, Ki, 0);
    SoccerMotor3_SetPIDConstants(Kp, Ki, 0);
    
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
            //SoccerMotor1_UpdateSpeed(M1_Speed);
            //SoccerMotor2_UpdateSpeed(M2_Speed);
            //SoccerMotor3_UpdateSpeed(M3_Speed);
            
            //Check for timer overflows.
            if(tick_Flag)
                printf("Timer overflow!\n");
        }
    }
}