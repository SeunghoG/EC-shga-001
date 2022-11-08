

#ifndef __EC_TIM_H 
#define __EC_TIM_H
#include "stm32f411xe.h"

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */


/* Timer Configuration */
void TIM_init_usec(TIM_TypeDef *timerx, uint32_t usec);
void TIM_init_msec(TIM_TypeDef *timerx, uint32_t msec);

void TIM_period_us(TIM_TypeDef* timx, uint32_t usec);  
void TIM_period_ms(TIM_TypeDef* timx, uint32_t msec);

void TIM_INT_init_usec(TIM_TypeDef* timerx, uint32_t usec); 
void TIM_INT_init_msec(TIM_TypeDef* timerx, uint32_t msec);

void TIM_INT_enable(TIM_TypeDef* timx);
void TIM_INT_disable(TIM_TypeDef* timx);

uint32_t is_UIF(TIM_TypeDef *TIMx);
void clear_UIF(TIM_TypeDef *TIMx);


/* Input Capture*/

// Edge Type
#define IC_RISE 0
#define IC_FALL 1
#define IC_BOTH 2

//Input Capture

typedef struct{
	GPIO_TypeDef *port;
	int pin;   
	TIM_TypeDef *timer;
	int ch;  		//int Timer Channel
	int ICnum;  //int IC number
} IC_t;

void ICAP_init(IC_t *ICx, GPIO_TypeDef *port, int pin);
void ICAP_setup(IC_t *ICx, int IC_number, int edge_type);
void ICAP_counter_us(IC_t *ICx, int usec);

uint32_t is_CCIF(TIM_TypeDef *TIMx, uint32_t ccNum);
void clear_CCIF(TIM_TypeDef *TIMx, uint32_t ccNum);

void ICAP_pinmap(IC_t *timer_pin);


#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif
