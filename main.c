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

#include <stdbool.h>
#include "project.h"
#include "serial.h"

#define CLOCK_FREQ_KHZ 32
#define DEFAULT_TICK_PERIOD_MS 20
#define DEFAULT_TAU_MS 50

// Callback for the printf function
void putdata(void* p, char c)
{
	UART_UartPutChar(c);
}

//The timer ISR.
CY_ISR(Tick_ISR)
{
	// PID control, etc
	SoccerMotor1_tick();
	SoccerMotor2_tick();
	SoccerMotor3_tick();
	
	// Clear interrupt (this avoids an infinite loop!)
	Tick_Timer_ClearInterrupt(Tick_Timer_INTR_MASK_TC);
}

// The UART ISR
CY_ISR(UART_ISR)
{
	// If multiple characters are sent at once it is possible for only
	// one interrupt to be generated. In this case we want to completely
	// clear the FIFO, so we use a while loop.
	while(UART_SpiUartGetRxBufferSize())
		serial_recieveChar(UART_UartGetChar());
		
	// Clear interrupt (this avoids an infinite loop!)
	UART_ClearRxInterruptSource(UART_INTR_RX_NOT_EMPTY);
}

int main()
{
    // Start the motor controllers.
	SoccerMotor1_Start(DEFAULT_TICK_PERIOD_MS, DEFAULT_TAU_MS);
	SoccerMotor2_Start(DEFAULT_TICK_PERIOD_MS, DEFAULT_TAU_MS);
	SoccerMotor3_Start(DEFAULT_TICK_PERIOD_MS, DEFAULT_TAU_MS);
    
    // Start the serial port.
    UART_Start();
	UART_INT_StartEx(UART_ISR);
    
    // Initialize the printf function to use the serial port.
    init_printf(NULL, putdata);
    
    // Start the timer and ISR.
    Tick_Timer_Start();
	Tick_Timer_WritePeriod(CLOCK_FREQ_KHZ * DEFAULT_TICK_PERIOD_MS);
    Tick_INT_StartEx(Tick_ISR);

    // Enable global interrupts.
    CyGlobalIntEnable;
	
	// Loop (all control is handeled by interrupts)
	while(true);
}