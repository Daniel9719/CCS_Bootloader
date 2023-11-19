#include <stdint.h>
#include <stdbool.h>

#include "inc/hw_memmap.h"
#include "inc/hw_nvic.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/flash.h"
#include "utils/uartstdio.h"

#include "main.h"
#include "bl_commands.h"

#define HWREG(x)   (*((volatile uint32_t *)(x)))

union Data32 {
    struct{
        uint8_t HSB_HN: 8;
        uint8_t HSB_LN: 8;
        uint8_t LSB_HN: 8;
        uint8_t LSB_LN: 8;
    };
    uint32_t u32Data;
};

void Init_Config(void)
{
    uint32_t ClockFreq;
    ClockFreq = SysCtlClockFreqSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_OSC | SYSCTL_OSC_MAIN |
                   SYSCTL_XTAL_16MHZ, 16000000);

    // Enable the peripherals 
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOJ);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    // Configure the GPIO pin muxing for the UART function.
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);

    // Since GPIO A0 and A1 are used for the UART function, they must be
    // configured for use as a peripheral function (instead of GPIO).
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    GPIOPinTypeGPIOInput(GPIO_PORTJ_BASE, GPIO_PIN_0);
    GPIOPadConfigSet(GPIO_PORTJ_BASE, GPIO_PIN_0, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

    UARTStdioConfig(0, 115200, ClockFreq);
    UARTFIFOEnable(UART0_BASE);
    
}


void main_Application(void)
{
    void (*app_reset_handler) (void);
    uint32_t MSP_value = * ( (volatile uint32_t*) FLASH_SECTOR );
    UARTprintf("MSP Value = 0x%8x\n\r", MSP_value);
    if(MSP_value != 0xFFFFFFFF){

        __set_MSP(MSP_value);
        // HWREG(NVIC_VTABLE) = (uint32_t)( (volatile uint32_t*) FLASH_SECTOR );

        app_reset_handler = (void (*)(void)) *((volatile uint32_t*) (FLASH_SECTOR + 4));

        HWREG(NVIC_DIS0) = 0xffffffff;
        HWREG(NVIC_DIS1) = 0xffffffff;
        HWREG(NVIC_DIS2) = 0xffffffff;
        HWREG(NVIC_DIS3) = 0xffffffff;

        app_reset_handler();
    }
    UARTprintf("No application found on 0x%#x\n\r", MSP_value);
}

void main_Bootloader(void)
{
    uint8_t Command = 0;
    static uint8_t prevCommand = 0;
    uint8_t LastCmdSts = COMMAND_RET_SUCCESS;
    union Data32 MemoryData;
    static union Data32 MemoryAddr;
    static union Data32 MemorySize;
    void (*FunctionPtr) (void);

    GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0, 0);
    GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_1, GPIO_PIN_1);
    while(1){
        if (UARTCharsAvail(UART0_BASE))
        {
            Command = (uint8_t) UARTCharGet(UART0_BASE);
            switch (Command)
            {
            case COMMAND_PING:
                UARTCharPut(UART0_BASE, COMMAND_ACK);
                LastCmdSts = COMMAND_RET_SUCCESS;
                break;
            case COMMAND_DOWNLOAD:
                MemoryAddr.HSB_HN = (uint8_t) UARTCharGet(UART0_BASE);
                MemoryAddr.HSB_LN = (uint8_t) UARTCharGet(UART0_BASE);
                MemoryAddr.LSB_HN = (uint8_t) UARTCharGet(UART0_BASE);
                MemoryAddr.LSB_LN = (uint8_t) UARTCharGet(UART0_BASE);
                MemorySize.HSB_HN = (uint8_t) UARTCharGet(UART0_BASE);
                MemorySize.HSB_LN = (uint8_t) UARTCharGet(UART0_BASE);
                MemorySize.LSB_HN = (uint8_t) UARTCharGet(UART0_BASE);
                MemorySize.LSB_LN = (uint8_t) UARTCharGet(UART0_BASE);
                // ASSERT(!(ui32Address & 3));
                // ASSERT(!(ui32Count & 3));

                UARTprintf("Erasing flash memory...this procedure can take a couple of minutes...\n\r");
                if(FlashErase(MemoryAddr.u32Data) == -1){
                    LastCmdSts = COMMAND_RET_FLASH_FAIL;
                    UARTprintf("Memory erased unsuccessfuly\n\r");
                }
                else{
                    LastCmdSts = COMMAND_RET_SUCCESS;
                    UARTprintf("Memory erased successfuly\n\r");
                }
                UARTCharPut(UART0_BASE, COMMAND_ACK);
                prevCommand = COMMAND_DOWNLOAD;
                break;  
            case COMMAND_RUN:
                UARTCharPut(UART0_BASE, COMMAND_ACK);
                MemoryAddr.HSB_HN = (uint8_t) UARTCharGet(UART0_BASE);
                MemoryAddr.HSB_LN = (uint8_t) UARTCharGet(UART0_BASE);
                MemoryAddr.LSB_HN = (uint8_t) UARTCharGet(UART0_BASE);
                MemoryAddr.LSB_LN = (uint8_t) UARTCharGet(UART0_BASE);
                FunctionPtr = (void (*)(void)) MemoryAddr.u32Data;
                LastCmdSts = COMMAND_RET_SUCCESS;
                FunctionPtr();
                break;
            case COMMAND_GET_STATUS:
                UARTCharPut(UART0_BASE, LastCmdSts);
                break;
            case COMMAND_SEND_DATA:
                UARTCharPut(UART0_BASE, COMMAND_ACK);
                if(prevCommand == COMMAND_DOWNLOAD | prevCommand == COMMAND_SEND_DATA){
                    
                    prevCommand = COMMAND_SEND_DATA;
                    UARTprintf("Programming flash memory...this procedure can take a couple of minutes...\n\r");
                    if(FlashProgram((uint32_t*) MemoryData.u32Data, MemoryAddr.u32Data, MemorySize.u32Data) == -1){
                        LastCmdSts = COMMAND_RET_FLASH_FAIL;
                        UARTprintf("Memory programmed unsuccessfuly\n\r");
                    }
                    else{
                        LastCmdSts = COMMAND_RET_SUCCESS;
                        UARTprintf("Memory programmed successfuly\n\r");
                    }
                }
                else{
                    UARTprintf("Invalid command, this command must be followed by a RUN or DOWNLOAD command\n\r");
                    LastCmdSts = COMMAND_RET_INVALID_CMD;
                }
                break;
            case COMMAND_RESET:

                LastCmdSts = COMMAND_RET_SUCCESS;
                UARTCharPut(UART0_BASE, COMMAND_ACK);
                UARTprintf("Saliendo\n");
                SysCtlDelay(10000000);
                HWREG(NVIC_APINT) = (NVIC_APINT_VECTKEY | NVIC_APINT_SYSRESETREQ);
                break;
            default:
                UARTprintf("Unknown command: 0x%2x\n", Command);
                LastCmdSts = COMMAND_RET_UNKNOWN_CMD;
                break;
            }
        }   
        // SysCtlDelay()
    }
}

/**
 * main.c
 */
void main(void)
{
    Init_Config();
    if(GPIOPinRead(GPIO_PORTJ_BASE, GPIO_PIN_0))
    {
        UARTprintf("Button not pressed...Jumping to Application...\n\r");
        main_Application();
    }
    else
    {
        UARTprintf("Button pressed...Jumping to Bootloader...\n\r");
        main_Bootloader();
    }
}

// void UARTStringPut(uint32_t ui32Base, const char *pcString)
// {
//     unsigned int uIdx;

//     //
//     // Send the characters
//     //
//     while(*pcString)
//     {
//         //
//         // If the character to the UART is \n, then add a \r before it so that
//         // \n is translated to \n\r in the output.
//         //
//         if(pcString[uIdx] == '\n')
//         {
//             UARTCharPut(ui32Base, '\r');
//         }

//         //
//         // Send the character to the UART output.
//         //
//         UARTCharPut(ui32Base, *pcString++);
//     }
// }
