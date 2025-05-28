#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "hw_memmap.h"
#include "debug.h"
#include "gpio.h"
#include "hw_i2c.h"
#include "hw_types.h"
#include "i2c.h"
#include "pin_map.h"
#include "sysctl.h"
#include "systick.h"
#include "uart.h"
#include "hw_ints.h"
#include "interrupt.h"
#include "hw_nvic.h"
#define DISABLE_INTERRUPTS() HWREG(NVIC_DIS0) = 0xFFFFFFFF
#define ENABLE_INTERRUPTS() HWREG(NVIC_EN0) = 0xFFFFFFFF
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

typedef enum {
    MODE_DATE,
    MODE_TIME,
    MODE_ALARM,
	MODE_MAX
} DisplayMode;

typedef enum {
    SET_MODE,
    SET_VALUE,
    RUN,
	LEFT_FLOAT,
	RIGHT_FLOAT
} State;

// ��������
uint8_t 	I2C0_WriteByte(uint8_t DevAddr, uint8_t RegAddr, uint8_t WriteData);
uint8_t 	I2C0_ReadByte(uint8_t DevAddr, uint8_t RegAddr);
uint8_t     ReadKey(void);
State       CheckStateSwitch(uint8_t,State);
void 		Delay(uint32_t value);
void 		S800_GPIO_Init(void);
void		S800_I2C0_Init(void);
void		S800_UART_Init(void);
void        Power_On_Init(void);
void        Show(uint8_t*);
void        Blank(void);
void        ChooseChangeBit(void);
void        ShowTime(void);
void        ShowDate(void);
void        ShowAlarm(void);

void UARTStringPut(uint8_t *);
void UARTStringPutNonBlocking(const char *);


// ȫ�ֱ���
volatile uint8_t result; // ����I2C0_WriteByte�������صĴ������ͣ�0�����޴�
uint32_t ui32SysClock;
uint32_t ui32SysClock,ui32IntPriorityGroup,ui32IntPriorityMask;
uint32_t ui32IntPrioritySystick,ui32IntPriorityUart0;
uint8_t num_seg7[] = {0x3f,0x06,0x5b,0x4f,0x66,0x6d,0x7d,0x07,0x7f,0x6f,
					  0x77,0x7c,0x39,0x5e,0x79,0x71,0x3d,0x76,0x0f,0x0e,
					  0x75,0x38,0x37,0x54,0x5c,0x73,0x67,0x31,0x49,0x78,
					  0x3e,0x1c,0x7e,0x64,0x6e,0x59,0x0,
					  0x3f|0x80,0x06|0x80,0x5b|0x80,0x4f|0x80,0x66|0x80,0x6d|0x80,0x7d|0x80,0x07|0x80,0x7f|0x80,0x6f|0x80,}; // 0-9+a-z,��36λΪϨ��,���һ���Ǵ�С�����1-9

uint8_t Buffer[8]; // ȫ��Ҫͨ���������ʾ�����ݶ�Ҫ�������Buffer����

// ������ʾ�����ʼ��
uint8_t Time_buffer[] = {0,8,0,0,0,0,36,36};
uint8_t Date_buffer[] = {2,0,2,5,0,5,2,8};
uint8_t Alarm_buffer[] = {0,0,0,5,0,0,36,36};
uint8_t Blank_buffer[] = {36,36,36,36,36,36,36,36};
uint8_t Float_buffer[16] = {0X0};

DisplayMode currentMode = MODE_DATE;
State currentState = SET_MODE;
uint8_t currentKey = 0xFF;
uint8_t lastKeyState = 0xFF;
int float_cnt = 0;
uint16_t time_cnt = 0;
uint8_t left_float_speed = 0;
uint8_t right_float_speed = 0;

void SysTickIntHandler(void){
	// UARTStringPutNonBlocking("\r\nSYSTICK_INT\r\n");
	// UARTStringPut((uint8_t *)"\r\nSYSTICK_INT\r\n");
	switch (currentState){
		case LEFT_FLOAT:{
			int i, tmp;
			time_cnt++;
			if(left_float_speed == 0){
				if(time_cnt % 800 == 0){
					time_cnt = 0;
					for(i = 0; i < 8; i++){
						tmp = (i+float_cnt) >= 16 ? (i+float_cnt-16) : (i+float_cnt);
						Buffer[i] = Float_buffer[tmp];
					}
					float_cnt += 1; 
					if(float_cnt == 16) float_cnt = 0;
				}
			}
			else{
				if(time_cnt % 400 == 0){
					time_cnt = 0;
					for(i = 0; i < 8; i++){
						tmp = (i+float_cnt) >= 16 ? (i+float_cnt-16) : (i+float_cnt);
						Buffer[i] = Float_buffer[tmp];
					}
					float_cnt += 1; 
					if(float_cnt == 16) float_cnt = 0;
				}
			}

			break;
		}
		case RIGHT_FLOAT:{
			int i, tmp;
			time_cnt++;
			if(left_float_speed == 0){
				if(time_cnt % 800 == 0){
					time_cnt = 0;
					for(i = 0; i < 8; i++){
						tmp = (i+float_cnt) >= 16 ? (i+float_cnt-16) : (i+float_cnt);
						Buffer[i] = Float_buffer[tmp];
					}
					float_cnt -= 1; 
					if(float_cnt == 0) float_cnt = 16;
				}
			}
			else{
				if(time_cnt % 400 == 0){
					time_cnt = 0;
					for(i = 0; i < 8; i++){
						tmp = (i+float_cnt) >= 16 ? (i+float_cnt-16) : (i+float_cnt);
						Buffer[i] = Float_buffer[tmp];
					}
					float_cnt -= 1; 
					if(float_cnt == 0) float_cnt = 16;
				}
			}

			break;
		}
	}
}

void UART0_Handler(void)
{	
    uint32_t int_status = UARTIntStatus(UART0_BASE, true);
	UARTStringPutNonBlocking("\r\nUART0_INT\r\n");
	// UARTStringPut((uint8_t *)"\r\nUART0_INT\r\n");
    UARTIntClear(UART0_BASE, int_status);

    // ����������ַ�����������
    while(UARTCharsAvail(UART0_BASE)) { // ���UART����FIFO���Ƿ��п�������,������ʱ����True
        int32_t received = UARTCharGetNonBlocking(UART0_BASE);
        if(received != -1) {
			// �ɿ����ͣ�����ʽ����ÿ���ַ��ȴ�ʱ��̣ܶ�
            while(!UARTCharPutNonBlocking(UART0_BASE, (uint8_t)received)) {
                // �ȴ�����FIFO�пռ�
                // ��115200�������£��ȴ�ʱ��̣ܶ�Լ87?s/�ַ���
            }
        }
    }
}

int main(void){
	//use internal 16M oscillator, HSI
    ui32SysClock = SysCtlClockFreqSet((SYSCTL_XTAL_16MHZ |SYSCTL_OSC_INT | SYSCTL_USE_PLL |SYSCTL_CFG_VCO_480), 20000000);
	SysTickPeriodSet(ui32SysClock/1000);
	SysTickEnable();
	SysTickIntEnable();	
	
	S800_GPIO_Init();
	S800_I2C0_Init();
	S800_UART_Init();
	Power_On_Init();

	IntEnable(INT_UART0);
  	UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT);	//Enable UART0 RX,TX interrupt
	IntMasterEnable();

	// �����ж����ȼ�,UART�жϱ�Systick�ж����ȼ���
	ui32IntPriorityMask	= IntPriorityMaskGet();
	IntPriorityGroupingSet(3);														//Set all priority to pre-emtption priority
	
	IntPrioritySet(INT_UART0,3);													//Set INT_UART0 to highest priority
	IntPrioritySet(FAULT_SYSTICK,0x0e0);									//Set INT_SYSTICK to lowest priority
	
	ui32IntPriorityGroup = IntPriorityGroupingGet();

	ui32IntPriorityUart0 = IntPriorityGet(INT_UART0);
	ui32IntPrioritySystick = IntPriorityGet(FAULT_SYSTICK);
	
	while (1){	
		
		// UARTStringPut((uint8_t *)"\r\nmain-loop\r\n");
		Show(Buffer);
		currentKey = ReadKey();

		switch (currentState){
			case SET_MODE:{
				// ������⣨�½��ش�����
				if(!(currentKey & 0x01) && (lastKeyState & 0x01)) {
					currentMode = (currentMode + 1) % MODE_MAX;
				}
				currentState = CheckStateSwitch(currentKey, currentState);
				lastKeyState = currentKey;

				switch(currentMode) {
				case MODE_TIME:  ShowTime(); break;
				case MODE_DATE:  ShowDate(); break;
				case MODE_ALARM: ShowAlarm(); break;
				}

				break;
			}
			case LEFT_FLOAT:{
				if(!(currentKey & 0x40) && (lastKeyState & 0x40)) {
					left_float_speed = (left_float_speed+1) % 2;
				}

				memcpy(Float_buffer, Date_buffer, sizeof(Date_buffer));
				memcpy(Float_buffer+8, Time_buffer, sizeof(Time_buffer));

				currentState = CheckStateSwitch(currentKey, currentState);
				lastKeyState = currentKey;

				break;
			}
			case RIGHT_FLOAT:{
				if(!(currentKey & 0x20) && (lastKeyState & 0x20)) {
					left_float_speed = (left_float_speed+1) % 2;
				}

				memcpy(Float_buffer, Date_buffer, sizeof(Date_buffer));
				memcpy(Float_buffer+8, Time_buffer, sizeof(Time_buffer));

				currentState = CheckStateSwitch(currentKey, currentState);
				lastKeyState = currentKey;

				break;
			}
		}
	}
}

State CheckStateSwitch(uint8_t key, State lastState){
	if(!(key & 0x01) && (lastKeyState & 0x01)) {
		return SET_MODE; // ��SW1����ģʽ�л�״̬
	}
	else if (!(key & 0x02) && (lastKeyState & 0x02)){
		return SET_VALUE; // ��SW2����������ʾ����״̬
	}
	else if (!(key & 0x80) && (lastKeyState & 0x80)){
		return RUN; // ��SW8��������״̬
	}
	else if (!(key & 0x40) && (lastKeyState & 0x40)){
		return LEFT_FLOAT; // ��SW7��������ˮ��ʾ״̬
	}
	else if (!(key & 0x20) && (lastKeyState & 0x20)){
		return RIGHT_FLOAT; // ��SW6��������ˮ��ʾ״̬
	}
	else{ // ����ԭ״̬
		return lastState;
	}
}

uint8_t ReadKey(){
	result = I2C0_ReadByte(TCA6424_I2CADDR, TCA6424_INPUT_PORT0);
	return result;
}

void ShowTime(){
	int i;
	for(i = 0; i<8; i++){
		Buffer[i] = Time_buffer[i];
	}
	Buffer[1] += 37; // ��С����
	Buffer[3] += 37; // ��С����
}

void ShowAlarm(){
	int i;
	for(i = 0; i<8; i++){
		Buffer[i] = Alarm_buffer[i];
	}
	Buffer[1] += 37; // ��С����
	Buffer[3] += 37; // ��С����
}

void ShowDate(){
	int i;
	for(i = 0; i<8; i++){
		Buffer[i] = Date_buffer[i];
	}
	Buffer[3] += 37; // ��С����
	Buffer[5] += 37; // ��С����
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


void Power_On_Init(void){
	uint8_t ID[] = {4,2,9,1,0,0,1,6};
	uint8_t Name[] = {'w'-'a'+10,'z'-'a'+10,'h'-'a'+10,36,36,36,36,36};
	int i,j;
	for(i = 0; i<8; i++){
		Buffer[i] = Blank_buffer[i];
	}
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
	int i;
    uint8_t local_buffer[8];
    
    // �����ٽ���
    DISABLE_INTERRUPTS();
    memcpy(local_buffer, Buffer, sizeof(local_buffer));
    ENABLE_INTERRUPTS(); // �˳��ٽ���

	for(i=0; i<8; i++){
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(1<<i));
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,num_seg7[Buffer[i]]);
		Delay(10000); // ���Delay���Ƶ���ÿһλ��ʾ֮��ļ��,����̫��,����Ͳ���ͬʱ��ʾ����λ,���������
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,0x0);
	}
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

void UARTStringPut(uint8_t *cMessage)
{
	while(*cMessage!='\0')
		UARTCharPut(UART0_BASE,*(cMessage++));
}
void UARTStringPutNonBlocking(const char *cMessage)
{
	while(*cMessage!='\0')
		UARTCharPutNonBlocking(UART0_BASE,*(cMessage++));
}

void S800_UART_Init(void){
	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
  	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);						//Enable PortA
	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA));			//Wait for the GPIO moduleA ready

	GPIOPinConfigure(GPIO_PA0_U0RX);												// Set GPIO A0 and A1 as UART pins.
  	GPIOPinConfigure(GPIO_PA1_U0TX);    			

  	GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

	// Configure the UART for 115,200, 8-N-1 operation.
  	UARTConfigSetExpClk(UART0_BASE, ui32SysClock,115200,(UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |UART_CONFIG_PAR_NONE));
	UARTStringPut((uint8_t *)"\r\nHello,world!\r\n");
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
	Delay(100);
	//receive data
	I2CMasterSlaveAddrSet(I2C0_BASE, DevAddr, true);//���ôӻ���ַ
	I2CMasterControl(I2C0_BASE,I2C_MASTER_CMD_SINGLE_RECEIVE);//ִ�е��ζ�����
	while(I2CMasterBusBusy(I2C0_BASE));
	value = I2CMasterDataGet(I2C0_BASE);//��ȡ��ȡ������
	Delay(100);
	return value;
}

