
#include <stdint.h>
#include <stdbool.h>
#include "hw_memmap.h"
#include "debug.h"
#include "gpio.h"
#include "hw_i2c.h"
#include "hw_types.h"
#include "i2c.h"
#include "pin_map.h"
#include "sysctl.h"
//*****************************************************************************
//
//I2C GPIO chip address and resigster define
//
//*****************************************************************************
// 芯片在I2C总线上的设备地址
#define TCA6424_I2CADDR 				0x22
#define PCA9557_I2CADDR					0x18

// PCA9557 的寄存器地址，控制LED
#define PCA9557_INPUT					0x00    // 输入寄存器，读取引脚状态
#define	PCA9557_OUTPUT					0x01	// 输出寄存器，设置输出电平
#define PCA9557_POLINVERT				0x02	// 极性反转寄存器，控制是否反转输入信号
#define PCA9557_CONFIG					0x03	// 配置寄存器，设置引脚方向（输入/输出）

// TCA6424 寄存器地址，控制数码管
#define TCA6424_CONFIG_PORT0			0x0c    // 配置寄存器，0为输出，1为输入
#define TCA6424_CONFIG_PORT1			0x0d
#define TCA6424_CONFIG_PORT2			0x0e

#define TCA6424_INPUT_PORT0				0x00	// 输入寄存器
#define TCA6424_INPUT_PORT1				0x01
#define TCA6424_INPUT_PORT2				0x02

#define TCA6424_OUTPUT_PORT0			0x04	// 输出寄存器
#define TCA6424_OUTPUT_PORT1			0x05
#define TCA6424_OUTPUT_PORT2			0x06




void 		Delay(uint32_t value);
void 		S800_GPIO_Init(void);
uint8_t 	I2C0_WriteByte(uint8_t DevAddr, uint8_t RegAddr, uint8_t WriteData);
uint8_t 	I2C0_ReadByte(uint8_t DevAddr, uint8_t RegAddr);
void		S800_I2C0_Init(void);
volatile uint8_t result; // 接收I2C0_WriteByte函数返回的错误类型，0代表无错
uint32_t ui32SysClock;
uint8_t seg7[] = {0x3f,0x06,0x5b,0x4f,0x66,0x6d,0x7d,0x07,0x7f,0x6f,0x77,0x7c,0x58,0x5e,0x079,0x71,0x5c};
uint32_t num_count,time_count = 0;
uint8_t is_100ms_int = 1;

void SysTickIntHandler(void){
	// 数码管
	result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(1<<(num_count)));			//write port 2	控制哪一位数码管亮 1-8
	result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[num_count]);			//write port 1 	控制显示的数字			
	is_100ms_int = 0;
	// LED
	result = I2C0_WriteByte(PCA9557_I2CADDR,PCA9557_OUTPUT,(uint8_t)~(1<<(num_count)));

	num_count++;
	num_count %= 8;

	time_count++;
	time_count %= 10;
	if(time_count == 0){
		is_100ms_int = 1;
	}
}

int main(void)
{
	//use internal 16M oscillator, HSI
    ui32SysClock = SysCtlClockFreqSet((SYSCTL_XTAL_16MHZ |SYSCTL_OSC_INT |SYSCTL_USE_OSC), 16000000);
	SysTickPeriodSet(ui32SysClock/10); // 中断周期为100ms	
	
	S800_GPIO_Init();
	S800_I2C0_Init();


    // Enable interrupts to the processor.
    IntMasterEnable();

    // Enable the SysTick Interrupt.
    SysTickIntEnable();

    // Enable SysTick.
    SysTickEnable();

	while (1)
	{
		// result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(128));					//write port 2	控制哪一位数码管亮 1-8
		// result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[1]);						//write port 1 	控制显示的数字			
		
		
		// result = I2C0_WriteByte(PCA9557_I2CADDR,PCA9557_OUTPUT,0x0); //底板LED低电平有效	

		// GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0, GPIO_PIN_0);												// Turn on the PF0 
		// Delay(800000);
		// GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0, 0x0);														// Turn off the PF0.
		// Delay(800000);	

	}
}

void Delay(uint32_t value)
{
	uint32_t ui32Loop;
	for(ui32Loop = 0; ui32Loop < value; ui32Loop++){};
}

void S800_GPIO_Init(void)
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);						//Enable PortF
	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF));			//Wait for the GPIO moduleF ready
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOJ);						//Enable PortJ	
	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOJ));			//Wait for the GPIO moduleJ ready	
	
  	GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_0);			//Set PF0 as Output pin; LED
	GPIOPinTypeGPIOInput(GPIO_PORTJ_BASE,GPIO_PIN_0 | GPIO_PIN_1); //Set the PJ0,PJ1 as input pin; Button
	GPIOPadConfigSet(GPIO_PORTJ_BASE,GPIO_PIN_0 | GPIO_PIN_1,GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_STD_WPU); // Set Button mode
}


void S800_I2C0_Init(void)
{
	uint8_t result;
  	SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0); // 初始化i2c模块
  	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB); // 使用I2C模块0，引脚配置为I2C0SCL--PB2、I2C0SDA--PB3
	GPIOPinConfigure(GPIO_PB2_I2C0SCL); // 配置PB2为I2C0SCL
  	GPIOPinConfigure(GPIO_PB3_I2C0SDA); // 配置PB3为I2C0SDA
  	GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2); // I2C将GPIO_PIN_2用作SCL
  	GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_3); // I2C将GPIO_PIN_3用作SDA

	I2CMasterInitExpClk(I2C0_BASE,ui32SysClock, true);										//config I2C0 400k ?不是16M吗
	I2CMasterEnable(I2C0_BASE);	

	result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_CONFIG_PORT0,0x0ff);		//config port 0 as input
	result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_CONFIG_PORT1,0x0);			//config port 1 as output 控制数码管段选（显示数字几）
	result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_CONFIG_PORT2,0x0);			//config port 2 as output 控制数码管位选（哪一位显示数字）

	result = I2C0_WriteByte(PCA9557_I2CADDR,PCA9557_CONFIG,0x00);				//config port as output
	result = I2C0_WriteByte(PCA9557_I2CADDR,PCA9557_OUTPUT,0x0ff);				//turn off? the LED1-8, 这里输出的数据为2位16进制数，即8位2进制数，每一位代表一个LED的高/低电平
	
}


uint8_t I2C0_WriteByte(uint8_t DevAddr, uint8_t RegAddr, uint8_t WriteData)
{
	uint8_t rop;
	while(I2CMasterBusy(I2C0_BASE)){};//如果I2C0模块忙，等待
		//
	I2CMasterSlaveAddrSet(I2C0_BASE, DevAddr, false); // 主机往总线上发送从机地址+写标志位
		//设置主机要放到总线上的从机地址。false表示主机写从机，true表示主机读从机
		
	I2CMasterDataPut(I2C0_BASE, RegAddr);//主机写设备寄存器地址
	I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START);//执行重复写入操作
	while(I2CMasterBusy(I2C0_BASE)){};
		
	rop = (uint8_t)I2CMasterErr(I2C0_BASE);//调试用 Gets the error status of the I2C Master

	I2CMasterDataPut(I2C0_BASE, WriteData);
	I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);//执行重复写入操作并结束
	while(I2CMasterBusy(I2C0_BASE)){};

	rop = (uint8_t)I2CMasterErr(I2C0_BASE);//调试用

	return rop;//返回错误类型，无错返回0
}

uint8_t I2C0_ReadByte(uint8_t DevAddr, uint8_t RegAddr)
{
	uint8_t value,rop;
	while(I2CMasterBusy(I2C0_BASE)){};	
	I2CMasterSlaveAddrSet(I2C0_BASE, DevAddr, false);
	I2CMasterDataPut(I2C0_BASE, RegAddr);
//	I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START);		
	I2CMasterControl(I2C0_BASE,I2C_MASTER_CMD_SINGLE_SEND);//执行单词写入操作
	while(I2CMasterBusBusy(I2C0_BASE));
	rop = (uint8_t)I2CMasterErr(I2C0_BASE);
	Delay(1);
	//receive data
	I2CMasterSlaveAddrSet(I2C0_BASE, DevAddr, true);//设置从机地址
	I2CMasterControl(I2C0_BASE,I2C_MASTER_CMD_SINGLE_RECEIVE);//执行单次读操作
	while(I2CMasterBusBusy(I2C0_BASE));
	value = I2CMasterDataGet(I2C0_BASE);//获取读取的数据
	Delay(1);
	return value;
}

