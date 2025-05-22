#include <stdint.h>
#include <stdbool.h>
#include "hw_memmap.h"
#include "debug.h"
#include "gpio.h"
#include "hw_types.h"
#include "pin_map.h"
#include "sysctl.h"


#define   FASTFLASHTIME			(uint32_t) 300000
#define   SLOWFLASHTIME			(uint32_t) FASTFLASHTIME*20

uint32_t cnt = 0;
uint32_t ui32SysClock;

void 		Delay(uint32_t value);
void 		S800_GPIO_Init(void);
void		PF_Flash(uint32_t key_value, uint8_t Pin_name);
void        PF_LED_ON(uint32_t key_value, uint8_t Pin_name);
void        EXE_MOD4(uint32_t key_value);

int main(void)
{
	uint32_t read_key_value0, read_key_value1;
	S800_GPIO_Init();
	while(1)
  {
		read_key_value0 = GPIOPinRead(GPIO_PORTJ_BASE,GPIO_PIN_0)	;				//read the PJ0 key value
		read_key_value1 = GPIOPinRead(GPIO_PORTJ_BASE,GPIO_PIN_1)	;				//read the PJ1 key value
		// PF_Flash(read_key_value0, GPIO_PIN_0);
		// PF_LED_ON(read_key_value0, GPIO_PIN_0);
		// PF_LED_ON(read_key_value1, GPIO_PIN_1);
		EXE_MOD4(read_key_value0);
   }
}

void PF_Flash(uint32_t key_value, uint8_t Pin_name)
{
	uint32_t delay_time;
		if (key_value == 0)						//USR_SW1-PJ0 pressed
			delay_time = FASTFLASHTIME;
		else									//USR_SW1-PJ0 released
			delay_time = SLOWFLASHTIME;
		
		GPIOPinWrite(GPIO_PORTF_BASE, Pin_name, Pin_name);			// Turn on the LED.
		Delay(delay_time);
		GPIOPinWrite(GPIO_PORTF_BASE, Pin_name, 0x0);							// Turn off the LED.
		Delay(delay_time);
}

void PF_LED_ON(uint32_t key_value, uint8_t Pin_name){
		if (key_value == 0)						//USR_SW1-PJ0 pressed
			GPIOPinWrite(GPIO_PORTF_BASE, Pin_name, Pin_name);
		else									//USR_SW1-PJ0 released
			GPIOPinWrite(GPIO_PORTF_BASE, Pin_name, 0x0);
}

void EXE_MOD4(uint32_t key_value){
	// static uint32_t cnt = 1;
	if(key_value == 0){
		Delay(FASTFLASHTIME); // 按键消抖
		cnt += 1;
	}

	switch (cnt % 4)
	{
	case 0:
		GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 0x0);
		break;
	case 1:
		PF_Flash(0, GPIO_PIN_0);
		break;
	case 2:
		GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0, 0x0);
		break;
	case 3:
		PF_Flash(0, GPIO_PIN_1);
		break;
	}
}

void Delay(uint32_t value)
{
	uint32_t ui32Loop;
	for(ui32Loop = 0; ui32Loop < value; ui32Loop++){};
}


void S800_GPIO_Init(void)
{
	// Without these lines, the default frequency is 16MHz
	ui32SysClock = SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |
                                       SYSCTL_OSC_MAIN |
                                       SYSCTL_USE_OSC |
																			 SYSCTL_CFG_VCO_480), 120000000);
	
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);						//Enable PortF
	//函数原型：void SysCtlPeripheralEnable(uint32_t ui32Peripheral)
	
	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF));			//Wait for the GPIO moduleF ready
	//函数原型：bool SysCtlPeripheralReady(uint32_t ui32Peripheral)
	//如果指定的外设被使能成功，返回true，否则返回false
	
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOJ);						//Enable PortJ	
	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOJ)){
	};			//Wait for the GPIO moduleJ ready	
	
  	GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_0 | GPIO_PIN_1);			//Set PF0,PF1 as Output pin
	//函数原型：void GPIOPinTypeGPIOOutput(uint32_t ui32Port, uint8_t ui8Pins)
	//配置GPIO端口引脚为输出引脚，如果字符型（uint8_t）参数ui8Pins某位为1，则GPIO端口对应位配置为输出引脚
	
	GPIOPinTypeGPIOInput(GPIO_PORTJ_BASE,GPIO_PIN_0 | GPIO_PIN_1);//Set the PJ0,PJ1 as input pin
	//配置GPIO端口引脚为输入引脚，与GPIOPinTypeGPIOOutput()类似。GPIO_PIN_0 | GPIO_PIN_1 = 00000011b
	
	GPIOPadConfigSet(GPIO_PORTJ_BASE,GPIO_PIN_0 | GPIO_PIN_1,GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_STD_WPU);
	//函数原型：void GPIOPadConfigSet(uint32_t ui32Port, uint8_t ui8Pins, uint32_t ui32Strength, uint32_t ui32PinType)
	//GPIO端口配置。uint32_t ui32Port：GPIO端口基地址
	//ui8Pins：端口引脚位组合表示，如10000001b表示配置端口的D7、D0位
	//ui32Strength：端口的输出驱动能力，对输入设置无效，可选项包括GPIO_STRENGTH_2MA/4MA/8MA/8MA_SC/6MA/10MA/12MA
	//ui32PinType：引脚类型，可选项包括GPIO_PIN_TYPE_STD（推挽）、GPIO_PIN_TYPE_STD_WPU（推挽上拉）、GPIO_PIN_TYPE_STD_WPD（推挽下拉）、
	//GPIO_PIN_TYPE_OD（开漏）、GPIO_PIN_TYPE_ANALOG（模拟）、GPIO_PIN_TYPE_WAKE_HIGH（高电平从冬眠唤醒）、GPIO_PIN_TYPE_WAKE_LOW（低）
}


