/*----------------------------------------------------------------\
@ Embedded Controller by Young-Keun Kim - Handong Global University
Author           : SSS LAB
Created          : 05-03-2021
Modified         : 09-20-2022
Language/ver     : C++ in Keil uVision

Description      : Distributed to Students for LAB_GPIO
/----------------------------------------------------------------*/



#include "stm32f4xx.h"
#include "stm32f411xe.h"
#include "ecGPIO.h"



void GPIO_init(GPIO_TypeDef *Port, int pin, int mode){     
	// mode  : Input(0), Output(1), AlterFunc(2), Analog(3)   
	if (Port == GPIOA)
		RCC_GPIOA_enable();
	if (Port == GPIOB)
		RCC_GPIOB_enable();
	if (Port == GPIOC)
		RCC_GPIOC_enable();

	// You can also make a more general function of
	// void RCC_GPIO_enable(GPIO_TypeDef *Port); 

	GPIO_mode(Port, pin, mode);
	
}


// GPIO Mode          : Input(00), Output(01), AlterFunc(10), Analog(11)
void GPIO_mode(GPIO_TypeDef *Port, int pin, int mode){
   Port->MODER &= ~(3UL<<(2*pin));     
   Port->MODER |= mode<<(2*pin);    
}


// GPIO Speed          : Low speed (00), Medium speed (01), Fast speed (10), High speed (11)
void GPIO_ospeed(GPIO_TypeDef *Port, int pin, int speed){
	//[TO-DO] YOUR CODE GOES HERE
	Port->OSPEEDR &= ~(3UL<<(2*pin));
	//[TO-DO] YOUR CODE GOES HERE
	Port->OSPEEDR |= speed<<(2*pin);
}

// GPIO Output Type: Output push-pull (0, reset), Output open drain (1)
void GPIO_otype(GPIO_TypeDef *Port, int pin, int type){
  //[TO-DO] YOUR CODE GOES HERE
	//[TO-DO] YOUR CODE GOES HERE
	Port->OTYPER &= ~(1UL<<(pin));
	Port->OTYPER |= type<<(pin);
	
}

// GPIO Push-Pull    : No pull-up, pull-down (00), Pull-up (01), Pull-down (10), Reserved (11)
void GPIO_pupd(GPIO_TypeDef *Port, int pin, int pupd){
  
	//[TO-DO] YOUR CODE GOES HERE
	Port->PUPDR &= ~(3<<(pin*2));
	
	//[TO-DO] YOUR CODE GOES HERE
	Port->PUPDR |= pupd<<(pin*2);
}

int GPIO_read(GPIO_TypeDef *Port, int pin){
  int bitVal = ((Port->IDR)>>pin)&1;
	return bitVal;
}

void GPIO_write(GPIO_TypeDef *Port, int pin,  int output){
	Port->ODR &= ~(1UL<<(pin));
	Port->ODR |=(output << pin);
}

void toggle(GPIO_TypeDef *Port, int pin){
	Port->ODR ^= (1<<pin);
}

void sevensegment_init(void){
	RCC_HSI_init();	
	GPIO_init(GPIOA, PA5, OUTPUT);
	GPIO_otype(GPIOA,PA5, 0UL);       // 0:Push-Pull
	GPIO_pupd(GPIOA,PA5, 0UL);        // 00: none
	GPIO_ospeed(GPIOA,PA5, 1UL);      // 01:Medium Speed
	
	GPIO_init(GPIOA, PA6, OUTPUT);
	GPIO_otype(GPIOA,PA6, 0UL);       // 0:Push-Pull
	GPIO_pupd(GPIOA,PA6, 0UL);        // 00: none
	GPIO_ospeed(GPIOA,PA6, 1UL);      // 01:Medium Speed
	
	GPIO_init(GPIOA, PA7, OUTPUT);
	GPIO_otype(GPIOA,PA7, 0UL);       // 0:Push-Pull
	GPIO_pupd(GPIOA,PA7, 0UL);        // 00: none
	GPIO_ospeed(GPIOA,PA7, 1UL);      // 01:Medium Speed
	
	GPIO_init(GPIOA, PA8, OUTPUT);
	GPIO_otype(GPIOA,PA8, 0UL);       // 0:Push-Pull
	GPIO_pupd(GPIOA,PA8, 0UL);        // 00: none
	GPIO_ospeed(GPIOA,PA8, 1UL);      // 01:Medium Speed
	
	GPIO_init(GPIOA, PA9, OUTPUT);
	GPIO_otype(GPIOA,PA9, 0UL);       // 0:Push-Pull
	GPIO_pupd(GPIOA,PA9, 0UL);        // 00: none
	GPIO_ospeed(GPIOA,PA9, 1UL);      // 01:Medium Speed
	
	GPIO_init(GPIOB, PB6, OUTPUT);
	GPIO_otype(GPIOB,PB6, 0UL);       // 0:Push-Pull
	GPIO_pupd(GPIOB,PB6, 0UL);        // 00: none
	GPIO_ospeed(GPIOB,PB6, 1UL);      // 01:Medium Speed
	
	GPIO_init(GPIOB, PB10, OUTPUT);
	GPIO_otype(GPIOB,PB10, 0UL);       // 0:Push-Pull
	GPIO_pupd(GPIOB,PB10, 0UL);        // 00: none
	GPIO_ospeed(GPIOB,PB10, 1UL);      // 01:Medium Speed
	
	GPIO_init(GPIOC, PC7, OUTPUT);
	GPIO_otype(GPIOC,PC7, 0UL);       // 0:Push-Pull
	GPIO_pupd(GPIOC,PC7, 0UL);        // 00: none
	GPIO_ospeed(GPIOC,PC7, 1UL);      // 01:Medium Speed
	
}

void sevensegment_decode(int num){
	int number[10][7]={
                    {0,0,0,0,0,0,1},          //zero
                    {1,0,0,1,1,1,1},          //one
                    {0,0,1,0,0,1,0},          //two
                    {0,0,0,0,1,1,0},          //three
                    {1,0,0,1,1,0,0},          //four
                    {0,1,0,0,1,0,0},          //five
                    {0,1,0,0,0,0,0},          //six
                    {0,0,0,1,1,1,1},          //seven
                    {0,0,0,0,0,0,0},          //eight
                    {0,0,0,1,1,0,0},          //nine
                  };

  //display shows the number in this case 6
        
	GPIO_write(GPIOA,PA8,number[num][0]);  //a
	GPIO_write(GPIOB,PB10,number[num][1]); //b
	GPIO_write(GPIOA,PA7,number[num][2]);  //c
	GPIO_write(GPIOA,PA6,number[num][3]);  //d
	GPIO_write(GPIOA,PA5,number[num][4]);  //e
	GPIO_write(GPIOA,PA9,number[num][5]);  //f
	GPIO_write(GPIOC,PC7,number[num][6]);  //g
  GPIO_write(GPIOC,PB6,HIGH);            //g	
					
					
	//the digit after "number" is displayed
	//before it gets tired
	
}