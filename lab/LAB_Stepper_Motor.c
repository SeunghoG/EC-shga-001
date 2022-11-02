#include "stm32f411xe.h"
#include "ecGPIO.h"
#include "ecRCC.h"
#include "ecSysTick.h"
#include "ecEXTI.h"
#include "ecStepper.h"

// Initialiization 
void setup(void);
void EXTI15_10_IRQHandler(void);



int main(void) { 
	// Initialiization --------------------------------------------------------
		setup();
		Stepper_step(204800,0,FULL);
	
	// Inifinite Loop ----------------------------------------------------------
	while(1){;}
	
}
void setup(void)
{
	// Timer initialization
	RCC_PLL_init();
	SysTick_init();
	
	//EXTI button pin initialization
	button_int();
	EXTI_init(GPIOC, BUTTON_PIN, FALL, 0);
	
	Stepper_init(GPIOB,10,GPIOB,4,GPIOB,5,GPIOB,3);
	Stepper_setSpeed(2);
}

void EXTI15_10_IRQHandler(void) {  
	if (is_pending_EXTI(BUTTON_PIN)) {
		Stepper_stop();
		clear_pending_EXTI(BUTTON_PIN); // cleared by writing '1'
	}
}
