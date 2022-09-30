/**
******************************************************************************
* @author	your name
* @Mod		date
* @brief	Embedded Controller:  LAB Digital In/Out
*					 - Toggle multiple LEDs by Button B1 pressing
* 
******************************************************************************
*/


#include "stm32f4xx.h"
#include "ecRCC.h"
#include "ecGPIO.h"
#include "ecSysTick.h"


void setup(void);
  int but_val=0;
	int cnt=0;
	int i=0;
	
int main(void) { 
	// Initialiization
	setup();
	
	// Inifinite Loop 
	while(1){
		
		
		but_val=GPIO_read(GPIOC, BUTTON_PIN);
		if(but_val==0){ 
			delay_ms(70);
			cnt++;
			i=cnt%4;
			switch (i){
				case 1:
					toggle(GPIOA, PA5);
				if(cnt>4){
				toggle(GPIOB, PB6);}
				break;
				
				case 2:
					toggle(GPIOA, PA5);
				  toggle(GPIOA, PA6);
				break;
				
				case 3:
					toggle(GPIOA, PA6);
					toggle(GPIOA, PA7);
				break;
					
				default:
					toggle(GPIOA, PA7);
					toggle(GPIOB, PB6);
				break;
				
				}
		}
	}
}

// Initialiization 
void setup(void)
{
	RCC_HSI_init();	
	SysTick_init();
	
	GPIO_init(GPIOC, BUTTON_PIN, INPUT);  // calls RCC_GPIOC_enable()
	GPIO_pupd(GPIOC ,BUTTON_PIN , 1UL);   // 01: Pull-up
	
	GPIO_init(GPIOA, PA5, OUTPUT);    // calls RCC_GPIOA_enable()
	GPIO_otype(GPIOA,PA5, 0UL);       // 0:Push-Pull
	GPIO_pupd(GPIOA,PA5, 0UL);        // 00: none
	GPIO_ospeed(GPIOA,PA5, 2UL);      //10:Fast Speed
	
	GPIO_init(GPIOA, PA6, OUTPUT);    // calls RCC_GPIOA_enable()
	GPIO_otype(GPIOA,PA6, 0UL);       // 0:Push-Pull
	GPIO_pupd(GPIOA,PA6, 0UL);        // 00: none
	GPIO_ospeed(GPIOA,PA6, 2UL);      //10:Fast Speed
	
	GPIO_init(GPIOA, PA7, OUTPUT);    // calls RCC_GPIOA_enable()
	GPIO_otype(GPIOA,PA7, 0UL);       // 0:Push-Pull
	GPIO_pupd(GPIOA,PA7, 0UL);        // 00: none
	GPIO_ospeed(GPIOA,PA7, 2UL);      //10:Fast Speed
	
	GPIO_init(GPIOB, PB6, OUTPUT);    // calls RCC_GPIOB_enable()
	GPIO_otype(GPIOB,PB6, 0UL);       // 0:Push-Pull
	GPIO_pupd(GPIOB,PB6, 0UL);        // 00: none
	GPIO_ospeed(GPIOB,PB6, 2UL);      //10:Fast Speed
	
	GPIO_write(GPIOA, PA5,LOW);
	GPIO_write(GPIOA, PA6,LOW);
	GPIO_write(GPIOA, PA7,LOW);
	GPIO_write(GPIOB, PB6,LOW);
	
}