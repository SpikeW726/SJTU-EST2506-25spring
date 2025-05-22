//*****************************************************************************
//
// interrupt.c - Example demonstrating how to configure the systick interrupt.
//
// Copyright (c) 2010-2017 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
// 
//   Redistribution and use in source and binary forms, with or without
//   modification, are permitted provided that the following conditions
//   are met:
// 
//   Redistributions of source code must retain the above copyright
//   notice, this list of conditions and the following disclaimer.
// 
//   Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the  
//   distribution.
// 
//   Neither the name of Texas Instruments Incorporated nor the names of
//   its contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// 
// This is part of revision 2.1.4.178 of the Tiva Firmware Development Package.
//
//*****************************************************************************

#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"

//*****************************************************************************
//
// Definition of useful parameters
//
//*****************************************************************************
#define MOTOR_PORT GPIO_PORTF_BASE
#define BUTTON_PORT GPIO_PORTJ_BASE

// 步进电机控制参数
volatile uint8_t current_step = 0;
volatile uint8_t direction = 1; // 1:正转, 0:反转
volatile uint16_t cycle_count = 0;
volatile uint8_t reverse_flag = 0;

// 正向八拍时序表
const uint8_t step_sequence[8][4] = {
    {1, 0, 0, 0},
    {1, 1, 0, 0},
    {0, 1, 0, 0}, 
    {0, 1, 1, 0},
    {0, 0, 1, 0}, 
    {0, 0, 1, 1}, 
    {0, 0, 0, 1}, 
    {1, 0, 0, 1}
};

// const uint8_t step_sequence[4][4] = {
//     {1, 1, 0, 0}, // 节拍1
//     {0, 1, 1, 0}, // 节拍2
//     {0, 0, 1, 1}, // 节拍3
//     {1, 0, 0, 1}, // 节拍4
// };

//*****************************************************************************
//
// This function sets up UART0 to be used for a console to display information
// as the example is running.
//
//*****************************************************************************
void
InitConsole(void)
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF); 
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOJ); 

		
    GPIOPinTypeGPIOInput(GPIO_PORTJ_BASE, GPIO_PIN_0|GPIO_PIN_1);
    GPIOPadConfigSet(GPIO_PORTJ_BASE,GPIO_PIN_0 | GPIO_PIN_1,GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_STD_WPU);
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3);
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, 0);

    // UART Debug
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA); 
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    UARTStdioConfig(0, 115200, 16000000);
    
}

//*****************************************************************************
//
// The interrupt handler for the for Systick interrupt.
//
//*****************************************************************************
void Delay(uint32_t value)
{
	uint32_t ui32Loop;
	for(ui32Loop = 0; ui32Loop < value; ui32Loop++){};
}

void
SysTickIntHandler(void)
{
    if(direction == 0){
                GPIOPinWrite(MOTOR_PORT, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3,
										 step_sequence[current_step][0] << 0 |
										 step_sequence[current_step][1] << 1 |
										 step_sequence[current_step][2] << 2 |
										 step_sequence[current_step][3] << 3);
				current_step = (current_step + 1) % 8;
                // current_step = (current_step + 1) % 4;
    }
    else{
            	GPIOPinWrite(MOTOR_PORT, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3,
										 step_sequence[current_step][0] << 0 |
										 step_sequence[current_step][1] << 1 |
										 step_sequence[current_step][2] << 2 |
										 step_sequence[current_step][3] << 3);
				current_step = (current_step - 1) & 0x7; // 前5位全部置零，后3位实现模8循环
                // current_step = (current_step - 1) & 0x3;
    }

    cycle_count ++;
    cycle_count %= 4096;
    UARTprintf("Number of interrupts: %d\r\n", cycle_count);
    if(cycle_count == 4095){
        // 当中断触发512*8次代表转完一周，查看是否有反向信号
        if(reverse_flag){
            direction ^= 1; // 与1按位异或，就是将direction的最后一位0/1切换
            reverse_flag = 0;
        }
    }
}

//*****************************************************************************
//
// Configure the SysTick and SysTick interrupt with a period of 1 second.
//
//*****************************************************************************
int
main(void)
{
#if defined(TARGET_IS_TM4C129_RA0) ||                                         \
    defined(TARGET_IS_TM4C129_RA1) ||                                         \
    defined(TARGET_IS_TM4C129_RA2)
    uint32_t ui32SysClock;
#endif

    //
    // Set the clocking to run directly from the external crystal/oscillator.
    // TODO: The SYSCTL_XTAL_ value must be changed to match the value of the
    // crystal on your board.
    //
#if defined(TARGET_IS_TM4C129_RA0) ||                                         \
    defined(TARGET_IS_TM4C129_RA1) ||                                         \
    defined(TARGET_IS_TM4C129_RA2)
    ui32SysClock = SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |
                                       SYSCTL_OSC_MAIN |
                                       SYSCTL_USE_OSC), 25000000);
#else
    SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_OSC | SYSCTL_OSC_MAIN |
                   SYSCTL_XTAL_16MHZ);
#endif

    //
    // Set up the serial console to use for displaying messages.  This is
    // just for this example program and is not needed for Systick operation.
    //
    InitConsole();

    //
    // Set up the period for the SysTick timer.  The SysTick timer period will
    // be equal to the system clock, resulting in a period of 1 second.
    //
#if defined(TARGET_IS_TM4C129_RA0) ||                                         \
    defined(TARGET_IS_TM4C129_RA1) ||                                         \
    defined(TARGET_IS_TM4C129_RA2)
    SysTickPeriodSet(ui32SysClock);
#else
    SysTickPeriodSet(234375); // 234375 / 16M = 0.0146484375 这是使用四相八拍满足一分钟转一周需要的中断触发周期
#endif

    // Enable interrupts to the processor.
    IntMasterEnable();

    // Enable the SysTick Interrupt.
    SysTickIntEnable();

    // Enable SysTick.
    SysTickEnable();

    while(1)
    {
        // Check the states of Button 1 & 2
        if(!GPIOPinRead(BUTTON_PORT, GPIO_PIN_0)){
            Delay(SysCtlClockGet() / 100); // 去抖处理
            reverse_flag = 1;
        }
        if(!GPIOPinRead(BUTTON_PORT, GPIO_PIN_1)){
            Delay(SysCtlClockGet() / 100); // 去抖处理
            cycle_count = 0;
        }
    }
}
