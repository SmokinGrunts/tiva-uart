//*****************************************************************************
//
// tiva-uart.c - //
// This is part of revision 2.0.1.11577 of the EK-TM4C123GXL Firmware Package.
// Modificato per illustrare l'utilizzo della seriale nella MCU.
// Permette di usare una funzione composita come la printf, sia con numeri interi che reali
// in singola precisione.
//
//*****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "driverlib/debug.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/uart.h"
#include "uartp/uartstdio.h"
#include "uartp/cmdline.h"

//*****************************************************************************
//
//! \addtogroup example_list
//! <h1>UART Echo (uart_echo)</h1>
//!
//! This example application utilizes the UART to echo text.  The first UART
//! (connected to the USB debug virtual serial port on the evaluation board)
//! will be configured in 115,200 baud, 8-n-1 mode.  All characters received on
//! the UART are transmitted back to the UART.
//
//*****************************************************************************

//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
}
#endif

//*****************************************************************************
//
// The UART interrupt handler.
//
//*****************************************************************************
void
UARTIntHandler(void)
{
    uint32_t ui32Status;

    //
    // Get the interrrupt status.
    //
    ui32Status = ROM_UARTIntStatus(UART0_BASE, true);

    //
    // Clear the asserted interrupts.
    //
    ROM_UARTIntClear(UART0_BASE, ui32Status);

    //
    // Loop while there are characters in the receive FIFO.
    //
    while(ROM_UARTCharsAvail(UART0_BASE))
    {
        //
        // Read the next character from the UART and write it back to the UART.
        //
        ROM_UARTCharPutNonBlocking(UART0_BASE,
                                   ROM_UARTCharGetNonBlocking(UART0_BASE));

        //
        // Blink the LED to show a character transfer is occuring.
        //
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2);

        //
        // Delay for 1 millisecond.  Each SysCtlDelay is about 3 clocks.
        //
        SysCtlDelay(SysCtlClockGet() / (1000 * 3));

        //
        // Turn off the LED
        //
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0);

    }
}

//*****************************************************************************
//
// Send a string to the UART.
//
//*****************************************************************************
void
UARTSend(const uint8_t *pui8Buffer, uint32_t ui32Count)
{
    //
    // Loop while there are more characters to send.
    //
    while(ui32Count--)
    {
        //
        // Write the next character to the UART.
        //
    	UARTCharPut(UART0_BASE, *pui8Buffer++);
    }
}


//*****************************************************************************
//
// Configure the UART and its pins.  This must be called before UARTprintf().
//
//*****************************************************************************
void
ConfigureUART(void)
{
	volatile uint32_t tempo;
    //
    // Enable the GPIO Peripheral used by the UART.
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    //ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

    //
    // Enable UART0
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    // uart1
   //ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);


    //
    // Configure GPIO Pins for UART mode.
    //
    GPIOPinConfigure(GPIO_PA0_U0RX);
    ROM_GPIOPinConfigure(GPIO_PA1_U0TX);
    ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    //
	// Configure GPIO Pins for UART1 mode.
	//
	/*ROM_GPIOPinConfigure(0x1);
	ROM_GPIOPinConfigure(0x401);
	ROM_GPIOPinTypeUART(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1); */

    //
    // Use the internal 16MHz oscillator as the UART clock source.
    //
    //UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);
    //
	// Use the internal 16MHz oscillator as the UART clock source.
	//
	UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);

    tempo = SysCtlClockGet();
    //
    // Initialize the UART0 for console I/O.
    //
    //UARTStdioConfig(0, 115200, 16000000);

    //
	// Initialize the UART1 for console I/O.
	//
	UARTStdioConfig(0, 115200, 16000000);


}


//*****************************************************************************
//
// This example demonstrates how to send a string of data to the UART.
//
//*****************************************************************************
int
main(void)
{

    //
    // Enable lazy stacking for interrupt handlers.  This allows floating-point
    // instructions to be used within interrupt handlers, but at the expense of
    // extra stack usage.
    //
    ROM_FPUEnable();
    ROM_FPULazyStackingEnable();

    //
    // Set the clocking to run directly from the crystal.
    //
    //ROM_SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_OSC | SYSCTL_OSC_MAIN |
    //                   SYSCTL_XTAL_16MHZ);

    //
	// Set the system clock to run at 80Mhz off PLL with external crystal as
	// reference.
	//
    //

	SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ |
					   SYSCTL_OSC_MAIN);
    //
    // Enable the GPIO port that is used for the on-board LED.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

    //
    // Enable the GPIO pins for the LED (PF2).
    //
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_2);


    //
    // Enable processor interrupts.
    //
    ROM_IntMasterEnable();


    //
	// Enable and Initialize the UART.
	//
	ConfigureUART();

    //
    // Enable the UART interrupt.
    //
    ROM_IntEnable(INT_UART0);
    ROM_UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT);

    //ROM_IntEnable(INT_UART1);
    //ROM_UARTIntEnable(UART1_BASE, UART_INT_RX | UART_INT_RT);


    UARTSend((uint8_t *)"\033[2JEnter text: \n", 16);
   UARTprintf("\n");
   UARTprintf("Welcome to the Tiva C Series TM4C123G LaunchPad!\n");
   UARTprintf("Prova stampa numero intero %d\n", 243);

   /// per stampare i float occorre passare attraverso un buffer
	volatile float fSingleRaw, fSingleMultiplier, fSingleResult;
	char pcBuffer[32];

	fSingleRaw = 32767.0f;
	fSingleMultiplier =  0.0011970964f;
	fSingleResult = fSingleRaw * fSingleMultiplier;

	snprintf(pcBuffer, 32, "Raw: %f\n", fSingleRaw);
	snprintf(pcBuffer, 32, "Multiplier %f\n", fSingleMultiplier); ///beve circa 6k di flash!
	UARTprintf(pcBuffer);
	snprintf(pcBuffer, 32, "Result: %f\n", fSingleResult);
	UARTprintf(pcBuffer);

	fSingleResult /= fSingleRaw;

    //
    // Prompt for text to be entered.
    //
    UARTSend((uint8_t *)"\033[2JEnter text: \n", 16);
    UARTSend((uint8_t *)"Ciao a tutti! \n", 15);

    //
    // Loop forever echoing data through the UART.
    //
    while(1)
    {
    }
}
