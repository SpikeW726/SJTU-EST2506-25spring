
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
// оƬ��I2C�����ϵ��豸��ַ
#define TCA6424_I2CADDR 				0x22
#define PCA9557_I2CADDR					0x18

// PCA9557 �ļĴ�����ַ������LED
#define PCA9557_INPUT					0x00    // ����Ĵ�������ȡ����״̬
#define	PCA9557_OUTPUT					0x01	// ����Ĵ��������������ƽ
#define PCA9557_POLINVERT				0x02	// ���Է�ת�Ĵ����������Ƿ�ת�����ź�
#define PCA9557_CONFIG					0x03	// ���üĴ������������ŷ�������/�����

// TCA6424 �Ĵ�����ַ�����������
#define TCA6424_CONFIG_PORT0			0x0c    // ���üĴ�����0Ϊ�����1Ϊ����
#define TCA6424_CONFIG_PORT1			0x0d
#define TCA6424_CONFIG_PORT2			0x0e

#define TCA6424_INPUT_PORT0				0x00	// ����Ĵ���
#define TCA6424_INPUT_PORT1				0x01
#define TCA6424_INPUT_PORT2				0x02

#define TCA6424_OUTPUT_PORT0			0x04	// ����Ĵ���
#define TCA6424_OUTPUT_PORT1			0x05
#define TCA6424_OUTPUT_PORT2			0x06




void 		Delay(uint32_t value);
void 		S800_GPIO_Init(void);
uint8_t 	I2C0_WriteByte(uint8_t DevAddr, uint8_t RegAddr, uint8_t WriteData);
uint8_t 	I2C0_ReadByte(uint8_t DevAddr, uint8_t RegAddr);
void		S800_I2C0_Init(void);
void        Power_On_Init(void);
void        Show(uint8_t*);
void        Blank(void);
void        SwitchMode(void);
void        ChooseChangeBit(void);
volatile uint8_t result; // ����I2C0_WriteByte�������صĴ������ͣ�0�����޴�
uint32_t ui32SysClock;
uint8_t num_seg7[] = {0x3f,0x06,0x5b,0x4f,0x66,0x6d,0x7d,0x07,0x7f,0x6f,
					  0x77,0x7c,0x39,0x5e,0x79,0x71,0x3d,0x76,0x0f,0x0e,
					  0x75,0x38,0x37,0x54,0x5c,0x73,0x67,0x31,0x49,0x78,
					  0x3e,0x1c,0x7e,0x64,0x6e,0x59,0x0}; // 0-9+a-z,���һ��ΪϨ��
uint32_t num_count,time_count = 0;
uint8_t is_100ms_int = 1;
uint8_t* Buffer; // ȫ��Ҫͨ���������ʾ�����ݶ�Ҫ�������Buffer����


void SysTickIntHandler(void){
	// �����
	result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(1<<(num_count)));			//write port 2	������һλ������� 1-8
	result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,num_seg7[num_count]);			//write port 1 	������ʾ������			
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
	SysTickPeriodSet(ui32SysClock/10); // �ж�����Ϊ100ms	
	
	S800_GPIO_Init();
	S800_I2C0_Init();
	Power_On_Init();

    // // Enable interrupts to the processor.
    // IntMasterEnable();

    // // Enable the SysTick Interrupt.
    // SysTickIntEnable();

    // // Enable SysTick.
    // SysTickEnable();

	while (1)
	{
		result = I2C0_ReadByte(TCA6424_I2CADDR, TCA6424_INPUT_PORT0); // ���϶�ȡ8��������״̬,���ݶ�ȡֵ��Ϥ˭������,Ȼ�������Ӧ����
		if(result == 0xFE){
			SwitchMode();
		}
		else if (result == 0xFD){
			ChooseChangeBit();
		}
	}
}

void SwitchMode(void){
	uint8_t test[] = {1,1,1,1,1,1,1,1};
	Show(test);
}

void ChooseChangeBit(){
	uint8_t test[] = {1,1,1,1,1,1,1,1};
	Show(test);
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
  	SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0); // ��ʼ��i2cģ��
  	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB); // ʹ��I2Cģ��0����������ΪI2C0SCL--PB2��I2C0SDA--PB3
	GPIOPinConfigure(GPIO_PB2_I2C0SCL); // ����PB2ΪI2C0SCL
  	GPIOPinConfigure(GPIO_PB3_I2C0SDA); // ����PB3ΪI2C0SDA
  	GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2); // I2C��GPIO_PIN_2����SCL
  	GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_3); // I2C��GPIO_PIN_3����SDA

	I2CMasterInitExpClk(I2C0_BASE,ui32SysClock, true);										//config I2C0 400k ?����16M��
	I2CMasterEnable(I2C0_BASE);	

	result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_CONFIG_PORT0,0x0ff);		//config port 0 as input
	result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_CONFIG_PORT1,0x0);			//config port 1 as output ��������ܶ�ѡ����ʾ���ּ���
	result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_CONFIG_PORT2,0x0);			//config port 2 as output ���������λѡ����һλ��ʾ���֣�

	result = I2C0_WriteByte(PCA9557_I2CADDR,PCA9557_CONFIG,0x00);				//config port as output
	result = I2C0_WriteByte(PCA9557_I2CADDR,PCA9557_OUTPUT,0x0ff);				//turn off? the LED1-8, �������������Ϊ2λ16����������8λ2��������ÿһλ����һ��LED�ĸ�/�͵�ƽ
	
}

void Power_On_Init(void){
	uint8_t ID[] = {4,2,9,1,0,0,1,6};
	uint8_t Name[] = {'w'-'a'+10,'z'-'a'+10,'h'-'a'+10,36,36,36,36,36};
	int i,j;
	for(i=0; i<3; i++){ // ѧ�ź��λ��LED��˸3��
		result = I2C0_WriteByte(PCA9557_I2CADDR,PCA9557_OUTPUT,0x0);
		for(j=0; j<20; j++){ // ���ѭ�����Ƶ�����ʾʱ�䳤��
			Show(ID);
		}
		result = I2C0_WriteByte(PCA9557_I2CADDR,PCA9557_OUTPUT,0xFF);
		Blank();
		Delay(800000); // ���Delay���Ƶ�����˸���ʱ��
	}
	for(i=0; i<3; i++){ // ������˸3��
		for(j=0; j<20; j++){
			Show(Name);
		}
		Blank();
		Delay(800000);
	}
}

void Blank(){
	int i;
	for(i=0; i<8; i++){
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(1<<i));
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,0x0);	
	}
}

void Show(uint8_t* Buffer){ // ��Ĭ����������ǰ�λ
	int i = 0;
	for(i=0; i<8; i++){
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(1<<i));
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,num_seg7[Buffer[i]]);
		Delay(10000); // ���Delay���Ƶ���ÿһλ��ʾ֮��ļ��,����̫��,����Ͳ���ͬʱ��ʾ����λ,���������
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,0x0);
	}
}

uint8_t I2C0_WriteByte(uint8_t DevAddr, uint8_t RegAddr, uint8_t WriteData)
{
	uint8_t rop;
	while(I2CMasterBusy(I2C0_BASE)){};//���I2C0ģ��æ���ȴ�
		//
	I2CMasterSlaveAddrSet(I2C0_BASE, DevAddr, false); // �����������Ϸ��ʹӻ���ַ+д��־λ
		//��������Ҫ�ŵ������ϵĴӻ���ַ��false��ʾ����д�ӻ���true��ʾ�������ӻ�
		
	I2CMasterDataPut(I2C0_BASE, RegAddr);//����д�豸�Ĵ�����ַ
	I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START);//ִ���ظ�д�����
	while(I2CMasterBusy(I2C0_BASE)){};
		
	rop = (uint8_t)I2CMasterErr(I2C0_BASE);//������ Gets the error status of the I2C Master

	I2CMasterDataPut(I2C0_BASE, WriteData);
	I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);//ִ���ظ�д�����������
	while(I2CMasterBusy(I2C0_BASE)){};

	rop = (uint8_t)I2CMasterErr(I2C0_BASE);//������

	return rop;//���ش������ͣ��޴���0
}

uint8_t I2C0_ReadByte(uint8_t DevAddr, uint8_t RegAddr)
{
	uint8_t value,rop;
	while(I2CMasterBusy(I2C0_BASE)){};	
	I2CMasterSlaveAddrSet(I2C0_BASE, DevAddr, false);
	I2CMasterDataPut(I2C0_BASE, RegAddr);
//	I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START);		
	I2CMasterControl(I2C0_BASE,I2C_MASTER_CMD_SINGLE_SEND);//ִ�е���д�����
	while(I2CMasterBusBusy(I2C0_BASE));
	rop = (uint8_t)I2CMasterErr(I2C0_BASE);
	Delay(1);
	//receive data
	I2CMasterSlaveAddrSet(I2C0_BASE, DevAddr, true);//���ôӻ���ַ
	I2CMasterControl(I2C0_BASE,I2C_MASTER_CMD_SINGLE_RECEIVE);//ִ�е��ζ�����
	while(I2CMasterBusBusy(I2C0_BASE));
	value = I2CMasterDataGet(I2C0_BASE);//��ȡ��ȡ������
	Delay(1);
	return value;
}

