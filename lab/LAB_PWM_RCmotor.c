#include "stm32f411xe.h"
#include "ecGPIO.h"
#include "ecRCC.h"
#include "ecSysTick.h"
#include "ecEXTI.h"
#include "ecTIM.h"
#include "ecPWM.h"

PWM_t pwm;
int idx = 0;
int dir = 1;
double duty =0.0;
// Initialiization 
void setup(void);
void EXTI15_10_IRQHandler(void);
void TIM3_IRQHandler(void);


int main(void) { 
	// Initialiization --------------------------------------------------------
		setup();
	
	// Inifinite Loop ----------------------------------------------------------
	while(1){
		
		}
	
}


void setup(void)
{
	// Timer initialization
	RCC_PLL_init();
	SysTick_init();
	
	// PWM Pin initialization
	GPIO_init(GPIOA, PA1, AF);
	GPIO_ospeed(GPIOA,PA1, 2UL);      // 10:fast Speed
	GPIO_pupd(GPIOA,PA1, 1UL);        // 01:pull-up
	GPIO_otype(GPIOA,PA1, 0UL);       // 00:push-pull
	
	//EXTI button pin initialization
	button_int();
	EXTI_init(GPIOC, BUTTON_PIN, FALL, 0);
	
	//TIM_interrupt_init
	TIM_INT_init_msec(TIM3, 500);
	
	
	PWM_init(&pwm,GPIOA, 1);
	PWM_period_ms(&pwm,20);

}

void EXTI15_10_IRQHandler(void) {  
	if (is_pending_EXTI(BUTTON_PIN)) {
		PWM_duty(&pwm, 0.5/20.0 );
		idx=0;
		clear_pending_EXTI(BUTTON_PIN); // cleared by writing '1'
	}
}

void TIM3_IRQHandler(void) {  
	if (is_UIF(TIM3)) {
		
		if(dir == 1) idx++;
		else if(dir == -1) idx--;
		
		dir = update_dir(dir,idx);
		duty = update_duty(idx);
		PWM_duty(&pwm,duty);

		clear_UIF(TIM3); // cleared by writing '1'
	}
}

