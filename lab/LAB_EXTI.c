#include "stm32f411xe.h"
#include "ecGPIO.h"
#include "ecRCC.h"
#include "ecEXTI.h"

int cnt = 0;
// Initialiization 
void setup(void);
void EXTI15_10_IRQHandler(void);

int main(void) { 
	// Initialiization --------------------------------------------------------
	setup();
	// Inifinite Loop ----------------------------------------------------------
	while(1){}
}

void setup(void){
	RCC_HSI_init();
	LED_init();
	
	EXTI_init(GPIOC, BUTTON_PIN, FALL, 0);
	GPIO_init(GPIOC, BUTTON_PIN, INPUT);
	GPIO_pupd(GPIOC, BUTTON_PIN, 1);
}

void EXTI15_10_IRQHandler(void) {  
	if (is_pending_EXTI(BUTTON_PIN)) {
		cnt++;
		LED_toggle(cnt);
		clear_pending_EXTI(BUTTON_PIN); // cleared by writing '1'
	}
}