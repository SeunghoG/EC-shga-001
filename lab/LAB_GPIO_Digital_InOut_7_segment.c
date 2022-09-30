#include "stm32f4xx.h"
#include "ecGPIO.h"
#include "ecRCC.h"
#include "ecSysTick.h"
//PA5, PA6, PA7, PB6, PC7, PA9, PA8, PB10



#define BUTTON_PIN 13

void setup(void);
	
int main(void) { 
	// Initialiization --------------------------------------------------------
	//setup();
	sevensegment_init();
	unsigned int cnt = 0;
	
	// Inifinite Loop ----------------------------------------------------------
	while(1){
		sevensegment_decode(cnt % 10);
		if(GPIO_read(GPIOC, BUTTON_PIN) == 0) {
			cnt++; 
		}
		
		for(int i = 0; i < 500000;i++){}
	}
}


// Initialiization 
void setup(void)
{
	GPIO_init(GPIOC, BUTTON_PIN, INPUT);  // calls RCC_GPIOC_enable()
	GPIO_pupd(GPIOC ,BUTTON_PIN , 1UL);   // 01: Pull-up

}