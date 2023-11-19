#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_nvic.h"
#include "inc/hw_types.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/systick.h"
#include "utils/uartstdio.h"

#define APP_BASE 0x00004000

static bool SysTick_Done = true;

void SysTick_IRQ(void)
{
    SysTickIntDisable();
    SysTick_Done = true;
    SysTickDisable();
    SysTickIntEnable();
}

void Init_Config(void)
{
    uint32_t ClockFreq;
    ClockFreq = SysCtlClockFreqSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_OSC | SYSCTL_OSC_MAIN |
                   SYSCTL_XTAL_16MHZ, 16000000);

    // Enable the peripherals 
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    // Configure the GPIO pin muxing for the UART function.
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);

    // Since GPIO A0 and A1 are used for the UART function, they must be
    // configured for use as a peripheral function (instead of GPIO).
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    UARTStdioConfig(0, 115200, ClockFreq);

    SysTickPeriodSet(16000000);
	HWREG(NVIC_VTABLE) = APP_BASE;
    SysTickIntRegister(&SysTick_IRQ);
    SysTickEnable();
}

/**
 * main.c
 */
void main(void)
{
    Init_Config();
	GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0, GPIO_PIN_0);
	GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_1, 0);
	while (1)
	{
		if(SysTick_Done)
		{
			UARTprintf("Running from application\r\n");
			SysTickPeriodSet(16000000);
			SysTickEnable();
			SysTick_Done = false;
		}
	}
}
