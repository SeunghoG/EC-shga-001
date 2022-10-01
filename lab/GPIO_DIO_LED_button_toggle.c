/**
******************************************************************************
* @author	seungho ga
* @Mod		2022.10.02
* @brief	Embedded Controller:  LAB Digital In/Out
*					 - Toggle multiple LEDs by Button B1 pressing
* 
******************************************************************************
*/


#include "stm32f4xx.h"
#include "ecRCC.h"
#include "ecGPIO.h"
#include "ecSysTick.h"


#define LED_PIN 	5
#define BUTTON_PIN 13

void setup(void);
	
int main(void) { 
	// Initialiization
	setup();
	int but_val=0;
	
	// Inifinite Loop 
	while(1){
		but_val=GPIO_read(GPIOC, BUTTON_PIN);
		
		if(but_val==0) 
		{toggle(GPIOA, LED_PIN);
		delay_ms(30);}
	}
}


// Initialiization 
void setup(void)
{
	RCC_HSI_init();	
	SysTick_init();
	
	GPIO_init(GPIOC, BUTTON_PIN, INPUT);  // calls RCC_GPIOC_enable()
	GPIO_pupd(GPIOC ,BUTTON_PIN , 0UL);   // 10: Pull-down
	
	GPIO_init(GPIOA, LED_PIN, OUTPUT);    // calls RCC_GPIOA_enable()
	GPIO_otype(GPIOA,LED_PIN, 0UL);       // 0:Push-Pull
	GPIO_pupd(GPIOA,LED_PIN, 0UL);        // 00: none
	GPIO_ospeed(GPIOA,LED_PIN, 2UL);      //10:Fast Speed
	
	GPIO_init(GPIOA, 6, OUTPUT);    // calls RCC_GPIOA_enable()
	GPIO_otype(GPIOA,6, 0UL);       // 0:Push-Pull
	GPIO_pupd(GPIOA,6, 0UL);        // 00: none
	GPIO_ospeed(GPIOA,6, 2UL);      //10:Fast Speed
	
	GPIO_init(GPIOA, 7, OUTPUT);    // calls RCC_GPIOA_enable()
	GPIO_otype(GPIOA,7, 0UL);       // 0:Push-Pull
	GPIO_pupd(GPIOA,7, 0UL);        // 00: none
	GPIO_ospeed(GPIOA,7, 2UL);      //10:Fast Speed
	
}
