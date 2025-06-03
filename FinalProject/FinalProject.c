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

// 蜂鸣器地址配置
#define BEEP_PWM_BASE   				PWM0_BASE        // PWM0 模块
#define BEEP_PWM_GEN    				PWM_GEN_3        // PWM 生成器组 3（对应通道 6-7）
#define BEEP_PWM_OUT    				PWM_OUT_7        // PWM 通道 7
#define BEEP_PWM_OUT_BIT 				PWM_OUT_7_BIT   // 通道位掩码
#define BEEP_GPIO_PORT  				GPIO_PORTK_BASE  // GPIO 端口 K
#define BEEP_GPIO_PIN   				GPIO_PIN_5       // GPIO 引脚 PK5

// 串口接收指令最大长度
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

// UART 接收状态结构体
typedef struct {
    volatile char rxBuffer[SERIAL_LENGTH_MAX];  // 接收缓冲区
	char cmdBuffer[SERIAL_LENGTH_MAX];          // 命令处理缓冲区
    volatile uint16_t rxIndex;                  // 当前接收位置
    volatile bool cmdReady;                     // 命令就绪标志
} UART_RxState;

// 函数声明
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

// 全局变量
volatile uint8_t result; // 接收I2C0_WriteByte函数返回的错误类型，0代表无错
uint32_t ui32SysClock;
uint32_t ui32SysClock,ui32IntPriorityGroup,ui32IntPriorityMask;
uint32_t ui32IntPrioritySystick,ui32IntPriorityUart0;
uint8_t num_seg7[] = {0x3f,0x06,0x5b,0x4f,0x66,0x6d,0x7d,0x07,0x7f,0x6f,
					  0x77,0x7c,0x39,0x5e,0x79,0x71,0x3d,0x76,0x0f,0x0e,
					  0x75,0x38,0x37,0x54,0x5c,0x73,0x67,0x31,0x49,0x78,
					  0x3e,0x1c,0x7e,0x64,0x6e,0x59,0x0,
					  0x3f|0x80,0x06|0x80,0x5b|0x80,0x4f|0x80,0x66|0x80,0x6d|0x80,0x7d|0x80,0x07|0x80,0x7f|0x80,0x6f|0x80}; // 0-9+a-z,第36位为熄灭,最后一行是带小数点的1-9

// 公用的全局变量
DisplayMode currentMode = MODE_DATE;
State currentState = SET_MODE;
uint8_t currentKey = 0xFF;
uint8_t lastKeyState = 0xFF;
uint8_t Buffer[8]; // 全局要通过数码管显示的内容都要进入这个Buffer数组

// 各种显示数组初始化
_time Time_buffer = {17,23,58};
_date Date_buffer = {2025,5,29};
_time Alarm_buffer = {0,0,15};
uint8_t Blank_buffer[] = {36,36,36,36,36,36,36,36};
uint8_t Float_buffer[16] = {0X0};

// “流水”使用的全局变量
int float_cnt = 0;
uint16_t time_cnt = 0;
uint8_t left_float_speed = 0;
uint8_t right_float_speed = 0;
// “设置显示数据”使用的全局变量
uint8_t date_blink_start_bit = 0;
uint8_t date_blink_end_bit = 3;
uint8_t blink_start_bit = 0;
uint8_t blink_end_bit = 1;
uint8_t bit_cnt = 0; 
uint16_t blink_time_cnt = 0;
// “运行”使用的全局变量
uint16_t run_time_cnt = 0;
bool time_sec_carry = false;
bool time_min_carry = false;
bool alarm_sec_carry = false;
bool alarm_min_carry = false;
bool beep_flag = false;

// “串口功能”使用的全局变量
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

	// 检查接收中断或接收超时中断
    if(int_status & (UART_INT_RX | UART_INT_RT)) { // 检测int_status是否包含这两个中断标志位
        // 处理所有可用字符
        while(UARTCharsAvail(UART0_BASE)) {
            int32_t received = UARTCharGetNonBlocking(UART0_BASE);
            if(received == -1) break;
            
            // 回车检测 (0x0D) - Windows 风格
            if(received == '\n') {
				// UARTStringPut((uint8_t *)"\r\n1111111111111\r\n");
                if(UART0_Rx.rxIndex > 0) {
                    // 添加字符串终止符
                    UART0_Rx.rxBuffer[UART0_Rx.rxIndex] = '\0';
                    UART0_Rx.cmdReady = true;
                }
                // 重置索引（确保下一次接收覆盖旧数据）
                UART0_Rx.rxIndex = 0;
                continue;
            }
			
			// 普通字符处理
            if(UART0_Rx.rxIndex < (SERIAL_LENGTH_MAX - 1)) {
                UART0_Rx.rxBuffer[UART0_Rx.rxIndex++] = (char)received;
            } 
			else {
                // 如果缓冲区溢出,则丢弃最旧字符
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

	// 配置中断优先级,UART中断比Systick中断优先级高
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
				// 按键检测（下降沿触发）
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
						else if(!(currentKey & 0x08) && (lastKeyState & 0x08)) { // SW4为增加闪烁位的值
							switch (bit_cnt){
								case 0: Date_buffer.year = (Date_buffer.year+1)%10000; break;							
								case 1: Date_buffer.mon++; if(Date_buffer.mon==13) Date_buffer.mon=1; break;
								case 2: Date_buffer.day++; if(Date_buffer.day==32) Date_buffer.day=1; break;
							}
						}
						else if(!(currentKey & 0x04) && (lastKeyState & 0x04)) { // SW3为减小闪烁位的值
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
						else if(!(currentKey & 0x08) && (lastKeyState & 0x08)) { // SW4为增加闪烁位的值
							switch (bit_cnt){
								case 0: Time_buffer.hour = (Time_buffer.hour+1)%24; break;							
								case 1: Time_buffer.min = (Time_buffer.min+1)%60; break;
								case 2: Time_buffer.sec = (Time_buffer.sec+1)%60; break;
							}
						}
						else if(!(currentKey & 0x04) && (lastKeyState & 0x04)) { // SW3为减小闪烁位的值
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
						else if(!(currentKey & 0x08) && (lastKeyState & 0x08)) { // SW4为增加闪烁位的值
							switch (bit_cnt){
								case 0: Alarm_buffer.hour = (Alarm_buffer.hour+1)%100; break;							
								case 1: Alarm_buffer.min = (Alarm_buffer.min+1)%60; break;
								case 2: Alarm_buffer.sec = (Alarm_buffer.sec+1)%60; break;
							}
						}
						else if(!(currentKey & 0x04) && (lastKeyState & 0x04)) { // SW3为减小闪烁位的值
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
		return SET_MODE; // 按SW1进入模式切换状态
	}
	else if (!(key & 0x02) && (lastKeyState & 0x02)){
		Buzzer_Off();
		return SET_VALUE; // 按SW2进入设置显示内容状态
	}
	else if (!(key & 0x80) && (lastKeyState & 0x80)){
		Buzzer_Off();
		return RUN; // 按SW8进入运行状态
	}
	else if (!(key & 0x40) && (lastKeyState & 0x40)){
		Buzzer_Off();
		return LEFT_FLOAT; // 按SW7进入左流水显示状态
	}
	else if (!(key & 0x20) && (lastKeyState & 0x20)){
		Buzzer_Off();
		return RIGHT_FLOAT; // 按SW6进入右流水显示状态
	}
	else{ // 保持原状态
		return lastState;
	}
}

// 命令处理函数（在主循环中调用）
void UART0_ProcessCommands(void) {
    // 检查命令就绪标志
    if(UART0_Rx.cmdReady) {
        // 进入临界区（禁用中断）
        uint32_t int_state = IntMasterDisable();
		// UARTStringPut((uint8_t *)"\r\n22222222222222\r\n");        
        // 复制接收缓冲区到命令缓冲区
        strncpy(UART0_Rx.cmdBuffer, (const char*)UART0_Rx.rxBuffer, SERIAL_LENGTH_MAX);
        
        // 清除标志
        UART0_Rx.cmdReady = false;
        
        // 退出临界区（恢复中断状态）
        if(!int_state) IntMasterEnable();
        
        // 处理命令（在中断外执行）
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
    char *read = buffer;  // 读指针,遍历原始字符串
    char *write = buffer; // 写指针,写入非空格字符   
	
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
	Buffer[1] += 37; // 加小数点
	Buffer[3] += 37; // 加小数点
}

void ShowAlarm(){
	Load_time(Buffer, Alarm_buffer);
	Buffer[1] += 37; // 加小数点
	Buffer[3] += 37; // 加小数点
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
	Buffer[3] += 37; // 加小数点
	Buffer[5] += 37; // 加小数点
}

void Delay(uint32_t value)
{
	uint32_t ui32Loop;
	for(ui32Loop = 0; ui32Loop < value; ui32Loop++){};
}

// 打开蜂鸣器
void Buzzer_On(void) {
    PWMOutputState(BEEP_PWM_BASE, BEEP_PWM_OUT_BIT, true);
}

// 关闭蜂鸣器
void Buzzer_Off(void) {
    PWMOutputState(BEEP_PWM_BASE, BEEP_PWM_OUT_BIT, false);
}

// 设置蜂鸣频率
void Buzzer_SetFrequency(uint32_t frequency) {
    uint32_t ui32Load = (SysCtlClockGet() / frequency) - 1;
    PWMGenPeriodSet(BEEP_PWM_BASE, BEEP_PWM_GEN, ui32Load);
    PWMPulseWidthSet(BEEP_PWM_BASE, BEEP_PWM_OUT, ui32Load / 2); // 保持 50% 占空比
}

void Beep_Init(void){
	uint32_t ui32PWMClock = SysCtlClockGet();       // 获取系统时钟（20 MHz）
    uint32_t ui32Load = (ui32PWMClock / 523) - 1; // 计算周期值（20 MHz / 2000 = 10000）

    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);   // 使能 PWM0 模块
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOK);  // 使能 PORTK

    GPIOPinTypePWM(BEEP_GPIO_PORT, BEEP_GPIO_PIN);  // 将 PK5 配置为 PWM 输出
    GPIOPinConfigure(GPIO_PK5_M0PWM7);              // 映射到 M0PWM7

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
	for(i=0; i<3; i++){ // 学号后八位和LED闪烁3次
		result = I2C0_WriteByte(PCA9557_I2CADDR,PCA9557_OUTPUT,0x0);
		for(j=0; j<20; j++){ // 这个循环控制的是显示时间长短
			Show(ID);
		}
		result = I2C0_WriteByte(PCA9557_I2CADDR,PCA9557_OUTPUT,0xFF);
		Blank();
		Delay(800000); // 这个Delay控制的是闪烁间隔时长
	}
	for(i=0; i<3; i++){ // 名字闪烁3次
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

void Show(uint8_t* Buffer){ // 先默认输出内容是八位
	int i;
	for(i=0; i<8; i++){
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(1<<i));
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,num_seg7[Buffer[i]]);
		Delay(10000); // 这个Delay控制的是每一位显示之间的间隔,不能太大,否则就不是同时显示所有位,而是走马灯
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
	Delay(100);
	//receive data
	I2CMasterSlaveAddrSet(I2C0_BASE, DevAddr, true);//设置从机地址
	I2CMasterControl(I2C0_BASE,I2C_MASTER_CMD_SINGLE_RECEIVE);//执行单次读操作
	while(I2CMasterBusBusy(I2C0_BASE));
	value = I2CMasterDataGet(I2C0_BASE);//获取读取的数据
	Delay(100);
	return value;
}

