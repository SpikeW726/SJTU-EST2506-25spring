#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
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
#include "hw_pwm.h"
#include "driverlib/pin_map.h"
#include "driverlib/pwm.h"

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

// ��������ַ����
#define BEEP_PWM_BASE   				PWM0_BASE        // PWM0 ģ��
#define BEEP_PWM_GEN    				PWM_GEN_3        // PWM �������� 3����Ӧͨ�� 6-7��
#define BEEP_PWM_OUT    				PWM_OUT_7        // PWM ͨ�� 7
#define BEEP_PWM_OUT_BIT 				PWM_OUT_7_BIT   // ͨ��λ����
#define BEEP_GPIO_PORT  				GPIO_PORTK_BASE  // GPIO �˿� K
#define BEEP_GPIO_PIN   				GPIO_PIN_5       // GPIO ���� PK5

// ���ڽ���ָ����󳤶�
#define SERIAL_LENGTH_MAX               128

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

typedef struct {
	uint8_t hour;
	uint8_t min;
	uint8_t sec;
} _time;

typedef struct {
	uint16_t year;
	uint8_t mon;
	uint8_t day;
} _date;

// UART ����״̬�ṹ��
typedef struct {
    volatile char rxBuffer[SERIAL_LENGTH_MAX];  // ���ջ�����
	char cmdBuffer[SERIAL_LENGTH_MAX];          // ���������
    volatile uint16_t rxIndex;                  // ��ǰ����λ��
    volatile bool cmdReady;                     // ���������־
} UART_RxState;

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
void  		Beep_Init(void);
void        Buzzer_On(void);
void        Buzzer_Off(void);
void 		Buzzer_SetFrequency(uint32_t);
void        Show(uint8_t*);
void        Blank(void);
void        ShowTime(void);
void        ShowDate(void);
void        ShowAlarm(void);
void        Load_date(uint8_t *, _date);
void        Load_time(uint8_t *, _time);

void 		UARTStringPut(uint8_t *);
void 		UARTStringPutNonBlocking(const char *);
void 		UART0_ProcessCommands(void);
void		ProcessCommand(const char* cmd);
void 		RemoveSpaces(char *buffer);
uint8_t* 	Uint8ToString(uint8_t value, const char* suffix);
uint8_t* 	Uint16ToString(uint16_t value, const char* suffix);

// ȫ�ֱ���
volatile uint8_t result; // ����I2C0_WriteByte�������صĴ������ͣ�0�����޴�
uint32_t ui32SysClock;
uint32_t ui32SysClock,ui32IntPriorityGroup,ui32IntPriorityMask;
uint32_t ui32IntPrioritySystick,ui32IntPriorityUart0;
uint8_t num_seg7[] = {0x3f,0x06,0x5b,0x4f,0x66,0x6d,0x7d,0x07,0x7f,0x6f,
					  0x77,0x7c,0x39,0x5e,0x79,0x71,0x3d,0x76,0x0f,0x0e,
					  0x75,0x38,0x37,0x54,0x5c,0x73,0x67,0x31,0x49,0x78,
					  0x3e,0x1c,0x7e,0x64,0x6e,0x59,0x0,
					  0x3f|0x80,0x06|0x80,0x5b|0x80,0x4f|0x80,0x66|0x80,0x6d|0x80,0x7d|0x80,0x07|0x80,0x7f|0x80,0x6f|0x80}; // 0-9+a-z,��36λΪϨ��,���һ���Ǵ�С�����1-9

// ���õ�ȫ�ֱ���
DisplayMode currentMode = MODE_DATE;
State currentState = SET_MODE;
uint8_t currentKey = 0xFF;
uint8_t lastKeyState = 0xFF;
uint8_t Buffer[8]; // ȫ��Ҫͨ���������ʾ�����ݶ�Ҫ�������Buffer����

// ������ʾ�����ʼ��
_time Time_buffer = {17,23,58};
_date Date_buffer = {2025,5,29};
_time Alarm_buffer = {0,0,15};
uint8_t Blank_buffer[] = {36,36,36,36,36,36,36,36};
uint8_t Float_buffer[16] = {0X0};

// ����ˮ��ʹ�õ�ȫ�ֱ���
int float_cnt = 0;
uint16_t time_cnt = 0;
uint8_t left_float_speed = 0;
uint8_t right_float_speed = 0;
// ��������ʾ���ݡ�ʹ�õ�ȫ�ֱ���
uint8_t date_blink_start_bit = 0;
uint8_t date_blink_end_bit = 3;
uint8_t blink_start_bit = 0;
uint8_t blink_end_bit = 1;
uint8_t bit_cnt = 0; 
uint16_t blink_time_cnt = 0;
// �����С�ʹ�õ�ȫ�ֱ���
uint16_t run_time_cnt = 0;
bool time_sec_carry = false;
bool time_min_carry = false;
bool alarm_sec_carry = false;
bool alarm_min_carry = false;
bool beep_flag = false;

// �����ڹ��ܡ�ʹ�õ�ȫ�ֱ���
UART_RxState UART0_Rx = {{0}, 0, false, {0}};
uint8_t txBuffer[SERIAL_LENGTH_MAX];

void SysTickIntHandler(void){
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
			if(right_float_speed == 0){
				if(time_cnt % 600 == 0){
					time_cnt = 0;
					for(i = 0; i < 8; i++){
						tmp = (i+float_cnt) >= 16 ? (i+float_cnt-16) : (i+float_cnt);
						Buffer[i] = Float_buffer[tmp];
					}
					float_cnt -= 1; 
					if(float_cnt <= 0) float_cnt = 16;
				}
			}
			else{
				if(time_cnt % 300 == 0){
					time_cnt = 0;
					for(i = 0; i < 8; i++){
						tmp = (i+float_cnt) >= 16 ? (i+float_cnt-16) : (i+float_cnt);
						Buffer[i] = Float_buffer[tmp];
					}
					float_cnt -= 1; 
					if(float_cnt <= 0) float_cnt = 16;
				}
			}
			break;
		}

		case SET_VALUE:{
			int i;
			static bool is_show = true;
			blink_time_cnt++;

			if(blink_time_cnt % 200 == 0){
				blink_time_cnt = 0;
				is_show = !is_show;
				switch (currentMode){
					case MODE_DATE:
						if(!is_show){
							for(i=date_blink_start_bit; i<=date_blink_end_bit; i++){
								Buffer[i] = 36;
							}
						}
						else ShowDate();
						break;
					case MODE_TIME:
						if(!is_show){
							for(i=blink_start_bit; i<=blink_end_bit; i++){
								Buffer[i] = 36;
							}
						}
						else ShowTime();
						break;
					case MODE_ALARM:
						if(!is_show){
							for(i=blink_start_bit; i<=blink_end_bit; i++){
								Buffer[i] = 36;
							}
						}
						else ShowAlarm();
						break;
				}
			}
			break;
		}

		case RUN:{
			run_time_cnt ++;
			if(beep_flag == true){
				Buzzer_On();
			}

			if(run_time_cnt % 1000 == 0){
				switch(currentMode){
					case MODE_TIME: 
						Time_buffer.sec = (Time_buffer.sec+1)%60;
						if(Time_buffer.sec == 0) time_sec_carry = true;
						if(time_sec_carry){
							time_sec_carry = false; 
							Time_buffer.min++;
							if(Time_buffer.min == 60) time_min_carry = true;
							Time_buffer.min %= 60;
						}
						if(time_min_carry){
							time_min_carry = false; 
							Time_buffer.hour = (Time_buffer.hour+1)%100;
						}
						ShowTime();
						break;
					case MODE_ALARM: 
						Alarm_buffer.sec--; 
						if(Alarm_buffer.sec == 0xff){
							if(Alarm_buffer.min == 0 && Alarm_buffer.hour == 0){beep_flag = true; Alarm_buffer.sec++; break;}
							else{
								Alarm_buffer.sec = 59;
								alarm_sec_carry = true;
							}
						}
						if(alarm_sec_carry){
							alarm_sec_carry = false; 
							Time_buffer.min--;
							if(Time_buffer.min == 0xff) alarm_min_carry = true;
							Time_buffer.min = 59;
						}
						if(alarm_min_carry){
							alarm_min_carry = false; 
							Time_buffer.hour--;
						}
						ShowAlarm();
						break;
				}
			}
				
			break;
		}
	}
}

void UART0_Handler(void)
{	
    uint32_t int_status = UARTIntStatus(UART0_BASE, true);
	// UARTStringPutNonBlocking("\r\nUART0_INT\r\n");
    UARTIntClear(UART0_BASE, int_status);

	// �������жϻ���ճ�ʱ�ж�
    if(int_status & (UART_INT_RX | UART_INT_RT)) { // ���int_status�Ƿ�����������жϱ�־λ
        // �������п����ַ�
        while(UARTCharsAvail(UART0_BASE)) {
            int32_t received = UARTCharGetNonBlocking(UART0_BASE);
            if(received == -1) break;
            
            // �س���� (0x0D) - Windows ���
            if(received == '\n') {
				// UARTStringPut((uint8_t *)"\r\n1111111111111\r\n");
                if(UART0_Rx.rxIndex > 0) {
                    // ����ַ�����ֹ��
                    UART0_Rx.rxBuffer[UART0_Rx.rxIndex] = '\0';
                    UART0_Rx.cmdReady = true;
                }
                // ����������ȷ����һ�ν��ո��Ǿ����ݣ�
                UART0_Rx.rxIndex = 0;
                continue;
            }
			
			// ��ͨ�ַ�����
            if(UART0_Rx.rxIndex < (SERIAL_LENGTH_MAX - 1)) {
                UART0_Rx.rxBuffer[UART0_Rx.rxIndex++] = (char)received;
            } 
			else {
                // ������������,��������ַ�
                memmove((void*)UART0_Rx.rxBuffer, 
                        (void*)&UART0_Rx.rxBuffer[1], 
                        SERIAL_LENGTH_MAX - 2);
                UART0_Rx.rxBuffer[SERIAL_LENGTH_MAX - 2] = (char)received;
                UART0_Rx.rxIndex = SERIAL_LENGTH_MAX - 1;
            }
        }
    }
}

int main(void){
    ui32SysClock = SysCtlClockFreqSet((SYSCTL_XTAL_16MHZ |SYSCTL_OSC_INT | SYSCTL_USE_PLL |SYSCTL_CFG_VCO_480), 20000000);
	SysTickPeriodSet(ui32SysClock/1000);
	SysTickEnable();
	SysTickIntEnable();	
	
	S800_GPIO_Init();
	S800_I2C0_Init();
	S800_UART_Init();
	Beep_Init();
	Power_On_Init();

	IntEnable(INT_UART0);
  	UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT);	//Enable UART0 RX,TX interrupt
	IntMasterEnable();

	// �����ж����ȼ�,UART�жϱ�Systick�ж����ȼ���
	ui32IntPriorityMask	= IntPriorityMaskGet();
	IntPriorityGroupingSet(3);														//Set all priority to pre-emtption priority
	
	IntPrioritySet(INT_UART0,3);													//Set INT_UART0 to highest priority
	IntPrioritySet(FAULT_SYSTICK,0x0e0);											//Set INT_SYSTICK to lowest priority
	
	ui32IntPriorityGroup = IntPriorityGroupingGet();

	ui32IntPriorityUart0 = IntPriorityGet(INT_UART0);
	ui32IntPrioritySystick = IntPriorityGet(FAULT_SYSTICK);

	while (1){	
		
		Show(Buffer);
		currentKey = ReadKey();
		UART0_ProcessCommands();

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
				Load_date(Float_buffer, Date_buffer);
				Load_time(Float_buffer+8, Time_buffer);

				currentState = CheckStateSwitch(currentKey, currentState);
				lastKeyState = currentKey;

				break;
			}

			case RIGHT_FLOAT:{
				if(!(currentKey & 0x20) && (lastKeyState & 0x20)) {
					right_float_speed = (right_float_speed+1) % 2;
				}

				Load_date(Float_buffer, Date_buffer);
				Load_time(Float_buffer+8, Time_buffer);

				currentState = CheckStateSwitch(currentKey, currentState);
				lastKeyState = currentKey;

				break;
			}

			case SET_VALUE:{
				switch (currentMode){
					case MODE_DATE:
						if(!(currentKey & 0x02) && (lastKeyState & 0x02)) {
							switch (bit_cnt){
								case 0:
									date_blink_start_bit += 4;
									date_blink_end_bit += 2;
									break;
								case 1:
									date_blink_start_bit += 2;
									date_blink_end_bit += 2;
									break;
								case 2:
									date_blink_start_bit = 0;
									date_blink_end_bit = 3;
									break;
							}
							bit_cnt = (bit_cnt+1)%3;
						}
						else if(!(currentKey & 0x08) && (lastKeyState & 0x08)) { // SW4Ϊ������˸λ��ֵ
							switch (bit_cnt){
								case 0: Date_buffer.year = (Date_buffer.year+1)%10000; break;							
								case 1: Date_buffer.mon++; if(Date_buffer.mon==13) Date_buffer.mon=1; break;
								case 2: Date_buffer.day++; if(Date_buffer.day==32) Date_buffer.day=1; break;
							}
						}
						else if(!(currentKey & 0x04) && (lastKeyState & 0x04)) { // SW3Ϊ��С��˸λ��ֵ
							switch (bit_cnt){
								case 0: Date_buffer.year--; if(Date_buffer.year==0xffff) Date_buffer.year=9999; break;							
								case 1: Date_buffer.mon--; if(Date_buffer.mon==0xff) Date_buffer.mon=12; break;
								case 2: Date_buffer.day--; if(Date_buffer.day==0xff) Date_buffer.day=31; break;
							}
						}
						break;
					
					case MODE_TIME:
						if(!(currentKey & 0x02) && (lastKeyState & 0x02)) {
							blink_start_bit = (blink_start_bit+2)%6;
							blink_end_bit = (blink_end_bit+2)%6;
							bit_cnt = (bit_cnt+1)%3;
						}
						else if(!(currentKey & 0x08) && (lastKeyState & 0x08)) { // SW4Ϊ������˸λ��ֵ
							switch (bit_cnt){
								case 0: Time_buffer.hour = (Time_buffer.hour+1)%24; break;							
								case 1: Time_buffer.min = (Time_buffer.min+1)%60; break;
								case 2: Time_buffer.sec = (Time_buffer.sec+1)%60; break;
							}
						}
						else if(!(currentKey & 0x04) && (lastKeyState & 0x04)) { // SW3Ϊ��С��˸λ��ֵ
							switch (bit_cnt){
								case 0: Time_buffer.hour--; if(Time_buffer.hour==0xff) Time_buffer.hour=23; break;							
								case 1: Time_buffer.min--; if(Time_buffer.min==0xff) Time_buffer.min=59; break;
								case 2: Time_buffer.sec--; if(Time_buffer.sec==0xff) Time_buffer.sec=59; break;
							}
						}
						break;

					case MODE_ALARM:
						if(!(currentKey & 0x02) && (lastKeyState & 0x02)) {
							blink_start_bit = (blink_start_bit+2)%6;
							blink_end_bit = (blink_end_bit+2)%6;
							bit_cnt = (bit_cnt+1)%3;
						}
						else if(!(currentKey & 0x08) && (lastKeyState & 0x08)) { // SW4Ϊ������˸λ��ֵ
							switch (bit_cnt){
								case 0: Alarm_buffer.hour = (Alarm_buffer.hour+1)%100; break;							
								case 1: Alarm_buffer.min = (Alarm_buffer.min+1)%60; break;
								case 2: Alarm_buffer.sec = (Alarm_buffer.sec+1)%60; break;
							}
						}
						else if(!(currentKey & 0x04) && (lastKeyState & 0x04)) { // SW3Ϊ��С��˸λ��ֵ
							switch (bit_cnt){
								case 0: Alarm_buffer.hour--; if(Alarm_buffer.hour==0xff) Alarm_buffer.hour=99; break;							
								case 1: Alarm_buffer.min--; if(Alarm_buffer.min==0xff) Alarm_buffer.min=59; break;
								case 2: Alarm_buffer.sec--; if(Alarm_buffer.sec==0xff) Alarm_buffer.sec=59; break;
							}
						}
						break;
					}

				currentState = CheckStateSwitch(currentKey, currentState);
				lastKeyState = currentKey;

				break;
			}

			case RUN:{

				currentState = CheckStateSwitch(currentKey, currentState);
				lastKeyState = currentKey;

				break;
			}
		}
	}
}

State CheckStateSwitch(uint8_t key, State lastState){
	if(!(key & 0x01) && (lastKeyState & 0x01)) {
		Buzzer_Off();
		return SET_MODE; // ��SW1����ģʽ�л�״̬
	}
	else if (!(key & 0x02) && (lastKeyState & 0x02)){
		Buzzer_Off();
		return SET_VALUE; // ��SW2����������ʾ����״̬
	}
	else if (!(key & 0x80) && (lastKeyState & 0x80)){
		Buzzer_Off();
		return RUN; // ��SW8��������״̬
	}
	else if (!(key & 0x40) && (lastKeyState & 0x40)){
		Buzzer_Off();
		return LEFT_FLOAT; // ��SW7��������ˮ��ʾ״̬
	}
	else if (!(key & 0x20) && (lastKeyState & 0x20)){
		Buzzer_Off();
		return RIGHT_FLOAT; // ��SW6��������ˮ��ʾ״̬
	}
	else{ // ����ԭ״̬
		return lastState;
	}
}

// �������������ѭ���е��ã�
void UART0_ProcessCommands(void) {
    // ������������־
    if(UART0_Rx.cmdReady) {
        // �����ٽ����������жϣ�
        uint32_t int_state = IntMasterDisable();
		// UARTStringPut((uint8_t *)"\r\n22222222222222\r\n");        
        // ���ƽ��ջ��������������
        strncpy(UART0_Rx.cmdBuffer, (const char*)UART0_Rx.rxBuffer, SERIAL_LENGTH_MAX);
        
        // �����־
        UART0_Rx.cmdReady = false;
        
        // �˳��ٽ������ָ��ж�״̬��
        if(!int_state) IntMasterEnable();
        
        // ����������ж���ִ�У�
		// UARTStringPut((uint8_t *)UART0_Rx.cmdBuffer);
		// UARTStringPut((uint8_t *)"\r\n");

		RemoveSpaces(UART0_Rx.cmdBuffer);
        ProcessCommand(UART0_Rx.cmdBuffer);
    }
}

void ProcessCommand(const char* cmd){
	// UARTStringPut((uint8_t *)cmd);
	// UARTStringPut((uint8_t *)"\r\n");

	if(strncmp(cmd, "*GET:", 5) == 0){
		if(strncmp(cmd+5, "DATE", 4) == 0){
			if(strcmp(cmd+9, "YEAR") == 0) {UARTStringPut(Uint16ToString(Date_buffer.year, "%u\r\n"));}
			else if(strcmp(cmd+9, "MONTH") == 0) {UARTStringPut(Uint8ToString(Date_buffer.mon, "%u\r\n"));}
			else if(strcmp(cmd+9, "DATE") == 0) {UARTStringPut(Uint8ToString(Date_buffer.day, "%u\r\n"));}
			else if(strcmp(cmd+9, "YEARMONTH") == 0) {UARTStringPut(Uint16ToString(Date_buffer.year, "%u ")); UARTStringPut(Uint8ToString(Date_buffer.mon, "%u\r\n"));}
			else if(strcmp(cmd+9, "MONTHDATE") == 0) {UARTStringPut(Uint8ToString(Date_buffer.mon, "%u ")); UARTStringPut(Uint8ToString(Date_buffer.day, "%u\r\n"));}
			else if(strcmp(cmd+9, "YEARDATE") == 0) {UARTStringPut(Uint16ToString(Date_buffer.year, "%u ")); UARTStringPut(Uint8ToString(Date_buffer.day, "%u\r\n"));}
			else if(strcmp(cmd+9, "YEARMONTHDATE") == 0) {UARTStringPut(Uint16ToString(Date_buffer.year, "%u ")); UARTStringPut(Uint8ToString(Date_buffer.mon, "%u ")); UARTStringPut(Uint8ToString(Date_buffer.day, "%u\r\n"));}
		}
		else if(strncmp(cmd+5, "TIME", 4) == 0){
			if(strcmp(cmd+9, "HOUR") == 0) {UARTStringPut(Uint8ToString(Time_buffer.hour, "%u\r\n"));}
			else if(strcmp(cmd+9, "MIN") == 0) {UARTStringPut(Uint8ToString(Time_buffer.min, "%u\r\n"));}
			else if(strcmp(cmd+9, "SEC") == 0) {UARTStringPut(Uint8ToString(Time_buffer.sec, "%u\r\n"));}
			else if(strcmp(cmd+9, "HOURMIN") == 0) {UARTStringPut(Uint8ToString(Time_buffer.hour, "%u ")); UARTStringPut(Uint8ToString(Time_buffer.min, "%u\r\n"));}
			else if(strcmp(cmd+9, "MINSEC") == 0) {UARTStringPut(Uint8ToString(Time_buffer.min, "%u ")); UARTStringPut(Uint8ToString(Time_buffer.sec, "%u\r\n"));}
			else if(strcmp(cmd+9, "HOURSEC") == 0) {UARTStringPut(Uint8ToString(Time_buffer.hour, "%u ")); UARTStringPut(Uint8ToString(Time_buffer.sec, "%u\r\n"));}
			else if(strcmp(cmd+9, "HOURMINSEC") == 0) {UARTStringPut(Uint8ToString(Time_buffer.hour, "%u ")); UARTStringPut(Uint8ToString(Time_buffer.min, "%u ")); UARTStringPut(Uint8ToString(Time_buffer.sec, "%u\r\n"));}
		}
		else if(strncmp(cmd+5, "ALARM", 5) == 0){
			UARTStringPut(Uint8ToString(Alarm_buffer.hour, "%u "));
			UARTStringPut(Uint8ToString(Alarm_buffer.min, "%u "));
			UARTStringPut(Uint8ToString(Alarm_buffer.sec, "%u\r\n"));
		}
	}

	else if(strncmp(cmd, "*SET:", 5) == 0){
		if (currentState != SET_VALUE) UARTStringPut((uint8_t *)"\r\nPress SW2 to switch to SET_VALUE mode first!!!\r\n");
		else{
			
		}
	}
}

void RemoveSpaces(char *buffer) {
    char *read = buffer;  // ��ָ��,����ԭʼ�ַ���
    char *write = buffer; // дָ��,д��ǿո��ַ�   
	
	if (buffer == NULL) return;

    while (*read != '\0') {
        if (*read != ' ') {*write = *read; write++; read++;}
        else read++;
    }

    *write = '\0';
}

uint8_t ReadKey(){
	result = I2C0_ReadByte(TCA6424_I2CADDR, TCA6424_INPUT_PORT0);
	return result;
}

void Load_time(uint8_t *target, _time source_buffer){
	target[0] = source_buffer.hour / 10;
	target[1] = source_buffer.hour % 10;
	target[2] = source_buffer.min / 10;
	target[3] = source_buffer.min % 10;
	target[4] = source_buffer.sec / 10;
	target[5] = source_buffer.sec % 10;
	target[6] = target[7] = 36;
}

void ShowTime(){
	Load_time(Buffer, Time_buffer);
	Buffer[1] += 37; // ��С����
	Buffer[3] += 37; // ��С����
}

void ShowAlarm(){
	Load_time(Buffer, Alarm_buffer);
	Buffer[1] += 37; // ��С����
	Buffer[3] += 37; // ��С����
}

void Load_date(uint8_t *target, _date source_buffer){
	uint16_t tmp_year = source_buffer.year;
	int i;
	for(i=3; i>=0; i--){
		target[i] = tmp_year % 10;
		tmp_year /= 10;
	}
	target[4] = source_buffer.mon / 10;
	target[5] = source_buffer.mon % 10;
	target[6] = source_buffer.day / 10;
	target[7] = source_buffer.day % 10;
}

void ShowDate(){
	Load_date(Buffer, Date_buffer);
	Buffer[3] += 37; // ��С����
	Buffer[5] += 37; // ��С����
}

void Delay(uint32_t value)
{
	uint32_t ui32Loop;
	for(ui32Loop = 0; ui32Loop < value; ui32Loop++){};
}

// �򿪷�����
void Buzzer_On(void) {
    PWMOutputState(BEEP_PWM_BASE, BEEP_PWM_OUT_BIT, true);
}

// �رշ�����
void Buzzer_Off(void) {
    PWMOutputState(BEEP_PWM_BASE, BEEP_PWM_OUT_BIT, false);
}

// ���÷���Ƶ��
void Buzzer_SetFrequency(uint32_t frequency) {
    uint32_t ui32Load = (SysCtlClockGet() / frequency) - 1;
    PWMGenPeriodSet(BEEP_PWM_BASE, BEEP_PWM_GEN, ui32Load);
    PWMPulseWidthSet(BEEP_PWM_BASE, BEEP_PWM_OUT, ui32Load / 2); // ���� 50% ռ�ձ�
}

void Beep_Init(void){
	uint32_t ui32PWMClock = SysCtlClockGet();       // ��ȡϵͳʱ�ӣ�20 MHz��
    uint32_t ui32Load = (ui32PWMClock / 523) - 1; // ��������ֵ��20 MHz / 2000 = 10000��

    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);   // ʹ�� PWM0 ģ��
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOK);  // ʹ�� PORTK

    GPIOPinTypePWM(BEEP_GPIO_PORT, BEEP_GPIO_PIN);  // �� PK5 ����Ϊ PWM ���
    GPIOPinConfigure(GPIO_PK5_M0PWM7);              // ӳ�䵽 M0PWM7

    PWMGenConfigure(BEEP_PWM_BASE, BEEP_PWM_GEN,
                    PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);

    PWMGenPeriodSet(BEEP_PWM_BASE, BEEP_PWM_GEN, ui32Load);

    PWMPulseWidthSet(BEEP_PWM_BASE, BEEP_PWM_OUT, ui32Load / 2); 

    PWMGenEnable(BEEP_PWM_BASE, BEEP_PWM_GEN);
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

uint8_t* Uint8ToString(uint8_t value, const char* suffix) {
    sprintf((char*)txBuffer, suffix, value);
    return txBuffer;
}
uint8_t* Uint16ToString(uint16_t value, const char* suffix) {
    sprintf((char*)txBuffer, suffix, value);
    return txBuffer;
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

