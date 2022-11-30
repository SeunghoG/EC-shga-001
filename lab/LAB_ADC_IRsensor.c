/**
******************************************************************************
* @author  SSSLAB
* @Mod		 2021-8-12 by YKKIM  	
* @brief   Embedded Controller:  LAB ___
*					 - _________________________________
* 
******************************************************************************
*/
#include "stm32f411xe.h"
#include "ecGPIO.h"
#include "ecRCC.h"
#include "ecTIM.h"
#include "ecSysTick.h"
#include "ecUART.h"
#include "ecADC.h"
#include "ecPWM.h"
#define A 0
#define B 1
//IR parameter//
uint32_t IR1, IR2;
int flag = 0;
int seqCHn[16] = {8,9,};
#define END_CHAR 13
#define MAX_BUF 100

uint8_t mcu2Data = 0;
uint8_t pcData = 0;
uint8_t btData = 0;
uint8_t buffer[MAX_BUF] = {0, };
int bReceive = 0;
int idx = 0;

float x=0.7;
float y=0.7;

uint32_t ovf_cnt = 0;
float distance = 0;

float timeInterval = 0;
float timestart = 0;
float timeend = 0;

void setup(void);


typedef struct{
   GPIO_TypeDef *port;
   int pin;
   
} _Pin;

_Pin dcPwmPin[2] = {
	{GPIOC, 9}, // TIM3 Ch3
	{GPIOC, 8}	// TIM3 Ch4
};

PWM_t dcPwm[2];

_Pin dcDirPin[2] = {
	{GPIOB, 8}, {GPIOC, 6}	
};
	
int main(void) { 

	// Initialiization --------------------------------------------------------
	setup();
	
	// Inifinite Loop ----------------------------------------------------------
	while(1){
		
		distance = (float) timeInterval / 58.0;
		printf("\r%f [cm]",distance);
		if(distance < 7.0){
				
				PWM_duty(&dcPwm[A], 0.0);
				PWM_duty(&dcPwm[B], 0.0);
				GPIO_write(dcDirPin[A].port, dcDirPin[A].pin, LOW);
				GPIO_write(dcDirPin[B].port, dcDirPin[B].pin, LOW);
		
		}
		
		else{
			printf("IR1 = %d \r\n",IR1);
			printf("IR2 = %d \r\n",IR2);
			printf("\r\n");
		
			if (IR1 > 1000){
				if(IR2 >1000) USART_write(USART1, "Error\r\n", 10);
				else{
				USART_write(USART1, "GO RIGHT\r\n", 10);
				PWM_duty(&dcPwm[A], 0.1);
				PWM_duty(&dcPwm[B], 0.9);
				GPIO_write(dcDirPin[A].port, dcDirPin[A].pin, LOW);
				GPIO_write(dcDirPin[B].port, dcDirPin[B].pin, LOW);
				}
			}
			else if (IR2 > 1000){
				if(IR1 >1000) USART_write(USART1, "Error\r\n", 10);
				else{
					USART_write(USART1, "GO LEFT\r\n", 10);
					PWM_duty(&dcPwm[A], 0.9);
					PWM_duty(&dcPwm[B], 0.1);
					GPIO_write(dcDirPin[A].port, dcDirPin[A].pin, LOW);
					GPIO_write(dcDirPin[B].port, dcDirPin[B].pin, LOW);
				}
			}
			else{
				if(IR2 >1000) printf("error");
				else{
					USART_write(USART1, "GO STRAIGHT\r\n", 10);
					PWM_duty(&dcPwm[A], 0.6);
					PWM_duty(&dcPwm[B], 0.6);
					GPIO_write(dcDirPin[A].port, dcDirPin[A].pin, LOW);
					GPIO_write(dcDirPin[B].port, dcDirPin[B].pin, LOW);
				}
			}
		}
		delay_ms(30);
	}
}

// Initialiization 
void setup(void){	
	RCC_PLL_init();                         // System Clock = 84MHz
	UART2_init();
	SysTick_init();
	
	USART_init(USART2, 9600);
	USART_begin(USART1, GPIOA, 9, GPIOA, 10, 9600); 	// PA9: TXD , PA10: RXD
	PWM_init(&dcPwm[A], dcPwmPin[A].port, dcPwmPin[A].pin);
	PWM_init(&dcPwm[B], dcPwmPin[B].port, dcPwmPin[B].pin);
	
	PWM_period_us(&dcPwm[A], 100);
	PWM_period_us(&dcPwm[B], 100);

	for (int i = 0; i < 2; i++){
		GPIO_init(dcDirPin[i].port, dcDirPin[i].pin, OUTPUT);
		GPIO_pupd(dcDirPin[i].port, dcDirPin[i].pin, PD);
		GPIO_otype(dcDirPin[i].port, dcDirPin[i].pin, OUTPP);
		GPIO_ospeed(dcDirPin[i].port, dcDirPin[i].pin, HIGHSPEED);
	}
	
	GPIO_write(dcDirPin[A].port, dcDirPin[A].pin, LOW);
	GPIO_write(dcDirPin[B].port, dcDirPin[B].pin, LOW);
	
	
	// PWM configuration ---------------------------------------------------------------------------------	
	PWM_t trig;												// PWM1 for trig
	PWM_init(&trig,GPIOA,6);			 		// PA_6: Ultrasonic trig pulse
	PWM_period_us(&trig, 50000);    	// PWM of 50ms period. Use period_us()
	PWM_pulsewidth_us(&trig, 10);   	// PWM pulse width of 10us
	
// Input Capture configuration -----------------------------------------------------------------------	
	IC_t echo;												// Input Capture for echo
	ICAP_init(&echo,GPIOB,6);    		// PB10 as input caputre
 	ICAP_counter_us(&echo, 10);   		// ICAP counter step time as 10us
	ICAP_setup(&echo, 1, IC_RISE);   	// TIM4_CH1 as IC1 , rising edge detect
	ICAP_setup(&echo, 2, IC_FALL);   	// TIM4_CH1 as IC2 , falling edge detect
	
	TIM_INT_enable(TIM4);           // TIM4 Interrupt Enable
	
	// ADC setting
  ADC_init(GPIOB, 0, TRGO);
	//ADC_TRGO(TIM2, 10, RISE);
	ADC_init(GPIOB, 1, TRGO);
	//ADC_TRGO(TIM2, 10, RISE);
	

	// ADC channel sequence setting
	ADC_sequence(2, seqCHn);
	
	// ADON, SW Trigger enable
	ADC_start();
}
void USART1_IRQHandler(){         //USART1 INT 
	if(is_USART_RXNE(USART1)){
		
		btData = USART_getc(USART1);
		USART_write(USART1,(uint8_t*) "BT sent : ", 10);
		USART_write(USART1, &btData, 1);
		USART_write(USART1, "\r\n", 2);
		printf("NUCLEO got : %c (from BT)\r\n",btData);
	
		
		if(btData == 'u' || btData == 'U')
		{
			x=x+0.01;y=y+0.01;
		}
		if(btData == 'd' || btData == 'D')
		{
			x=x-0.01;y=y-0.01;
		}
		if(btData == 'l' || btData == 'L')
		{
			x=x+0.05; 
		}
		if(btData == 'r' || btData == 'R')
		{
			y=y+0.05;
		}
		if(btData == 's' || btData == 'S')
		{
			x=0.0; y = 0.0;
		}
		
		//printf("NUCLEO got : %c (from BT)\r\n",btData);
		
	}
}

void TIM4_IRQHandler(void){
	if(is_UIF(TIM4)){                     	// Update interrupt
		ovf_cnt++;													  // overflow count
		clear_UIF(TIM4);  							    	// clear update interrupt flag
	}
	if(is_CCIF(TIM4, 1)){ 									  // TIM4_Ch1 (IC1) Capture Flag. Rising Edge Detect
		timestart = TIM4->CCR1;									// Capture TimeStart
		clear_CCIF(TIM4, 1);                 	  // clear capture/compare interrupt flag 
	}								                      
	else if(is_CCIF(TIM4,2)){ 								// TIM4_Ch1 (IC2) Capture Flag. Falling Edge Detect
		timeend = TIM4->CCR2;		// Capture TimeEnd
		clear_CCIF(TIM4, 2);
		
		timeInterval = 10*(-(timeend-timestart)+(0XFFFF*ovf_cnt)); 					// Total time of echo pulse (10us * counter pulses -> [msec] unit)
		ovf_cnt = 0;                        	// overflow reset
										  // clear capture/compare interrupt flag 
	}
}
void USART2_IRQHandler(){         //USART2 INT 
	if(is_USART_RXNE(USART2)){
		
		pcData = USART_getc(USART2);
		USART_write(USART1, &pcData, 1);
		
		printf("%c", pcData);
		
		if (pcData == END_CHAR)
			printf("\r\n");
		
	}
}
void ADC_IRQHandler(void){
	if((is_ADC_OVR())){
		clear_ADC_OVR();
	}
	
	if(is_ADC_EOC()){       //after finishing sequence
			if (flag==0){
				IR1 = ADC_read();
			}  
			else if (flag==1){
				IR2 = ADC_read();
			}
		flag =! flag;
	}
}