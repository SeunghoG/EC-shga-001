/**
******************************************************************************
* @author  SSSLAB
* @Mod		 2021-8-30 by YKKIM  	
* @brief   Embedded Controller:  LAB Systick&EXTI with API
*					 - 7 segment
* 
******************************************************************************
*/

#include "stm32f411xe.h"
#include "ecGPIO.h"
#include "ecRCC.h"
#include "ecSysTick.h"
#include "ecEXTI.h"

int count = 0;
// Initialiization 
void setup(void);
void EXTI15_10_IRQHandler(void);

int main(void) { 
	// Initialiization --------------------------------------------------------
		setup();
	
	// Inifinite Loop ----------------------------------------------------------
	while(1){
		sevensegment_decode(count);
		delay_ms(1000);
		count++;
		if (count >9) count =0;
		SysTick_reset();
	}
}

void setup(void)
{
	RCC_PLL_init();
	SysTick_init();
	sevensegment_init();
	
	EXTI_init(GPIOC, BUTTON_PIN, FALL, 0);
	GPIO_init(GPIOC, BUTTON_PIN, INPUT);
	GPIO_pupd(GPIOC, BUTTON_PIN, 1);
}

void EXTI15_10_IRQHandler(void) {  
	if (is_pending_EXTI(BUTTON_PIN)) {
		count=-1;
		clear_pending_EXTI(BUTTON_PIN); // cleared by writing '1'
	}
}