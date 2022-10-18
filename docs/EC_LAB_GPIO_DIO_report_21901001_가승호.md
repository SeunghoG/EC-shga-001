# LAB: GPIO Digital InOut

**Date:** 2022-09-20

**Author/Partner:**가승호 / 허다빈

**Github:** [repository link](https://github.com/SeunghoG/EC-shga-001)

**Demo Video:** [Youtube link](https://youtu.be/D1bWu2sCk1U)

###  

## Requirement

###  Hardware

- MCU
  - NUCLEO-F401RE
- Actuator/Sensor/Others:
  - LEDs x 3
  - Resistor 330 ohm x 3, breadboard

### Software

- Keil uVision, CMSIS, EC_HAL library



## Problem 1: Toggle LED with Button

List of functions for Digital_In and Out .



library file github link : [github link](https://github.com/SeunghoG/EC-shga-001/tree/main/include)

##### ecGPIO.h

```c
void GPIO_init(GPIO_TypeDef *Port, int pin, int mode);
void GPIO_write(GPIO_TypeDef *Port, int pin, int Output);
int  GPIO_read(GPIO_TypeDef *Port, int pin);
void GPIO_mode(GPIO_TypeDef* Port, int pin, int mode);
void GPIO_ospeed(GPIO_TypeDef* Port, int pin, int speed);
void GPIO_otype(GPIO_TypeDef* Port, int pin, int type);
void GPIO_pupd(GPIO_TypeDef* Port, int pin, int pupd);
void toggle(GPIO_TypeDef *Port, int pin);
void sevensegment_init(void);
void sevensegment_decode(int num);
```



##### ecGPIO.c

```c
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
```





## Problem 2: Toggle LED with Button

### Procedure

1. Create a new project under the directory `\repos\EC\LAB\`

- The project name is “**LAB_GPIO_DIO_LED”.**
- Name the source file as “**LAB_GPIO_DIO_LED.c”**



2. Include your library **ecGPIO.h, ecGPIO.c** in `\repos\EC\lib\`.



3. Toggle the LED by pushing button.

- Pushing button (LED ON), Pushing Button (LED OFF) and repeat

### Configuration

| Button (B1) |                LED                |
| :---------: | :-------------------------------: |
| Digital In  |            Digital Out            |
| GPIOC, Pin  |           GPIOA, Pin 5            |
|   PULL-UP   | Open-Drain, Pull-up, Medium Speed |



### Code

Your code goes here: [github_link](https://github.com/SeunghoG/EC-shga-001/blob/main/lab/GPIO_DIO_LED_button_toggle.c)

```c
/**
******************************************************************************
* @author	seungho_Ga
* @Mod		2022.09.30
* @brief	Embedded Controller:  LAB Digital In/Out
*					 - Toggle multiple LEDs by Button B1 pressing
* 
******************************************************************************
*/

#include "stm32f4xx.h"
#include "ecRCC.h"
#include "ecGPIO.h"
#include "ecSysTick.h" // time delay header file 


#define LED_PIN 	5
#define BUTTON_PIN 13

void setup(void);
	
int main(void) { 
	// Initialiization
	setup();
	int but_val=0;
	
	// Inifinite Loop 
	while(1){
		but_val=GPIO_read(GPIOC, BUTTON_PIN);  // read input value (from button)
		
		if(but_val==0) {
            toggle(GPIOA, LED_PIN);      // toggle function 
		    delay_ms(30);                // delay fucntion
        }
	}
}


// Initialiization 
void setup(void)
{
	RCC_HSI_init();	
	SysTick_init();                       // initialize to use time delay header file
	
	GPIO_init(GPIOC, BUTTON_PIN, INPUT);  // calls RCC_GPIOC_enable()
	GPIO_pupd(GPIOC ,BUTTON_PIN , 0UL);   // 10: Pull-down
	
	GPIO_init(GPIOA, LED_PIN, OUTPUT);    // calls RCC_GPIOA_enable()
	GPIO_otype(GPIOA,LED_PIN, 0UL);       // 0:Push-Pull
	GPIO_pupd(GPIOA,LED_PIN, 0UL);        // 00: none
	GPIO_ospeed(GPIOA,LED_PIN, 2UL);      //10:Fast Speed
	
	GPIO_init(GPIOA, 6, OUTPUT);          // calls RCC_GPIOA_enable()
	GPIO_otype(GPIOA,6, 0UL);             // 0:Push-Pull
	GPIO_pupd(GPIOA,6, 0UL);              // 00: none
	GPIO_ospeed(GPIOA,6, 2UL);            //10:Fast Speed
	
	GPIO_init(GPIOA, 7, OUTPUT);          // calls RCC_GPIOA_enable()
	GPIO_otype(GPIOA,7, 0UL);             // 0:Push-Pull
	GPIO_pupd(GPIOA,7, 0UL);              // 00: none
	GPIO_ospeed(GPIOA,7, 2UL);            //10:Fast Speed
	
}
```

### Discussion

1. Find out a typical solution for software debouncing and hardware debouncing. 

   * software debouncing : we have to use time delay function. the reason is that the board 

   ​       need a time to go to steady state value, and we want to get that.

   * hardware debouncing : Schmitt Trigger, with this trigger we can remove noise from human

     hand shivering.

2. What method of debouncing did this NUCLEO board used for the push-button(B1)?

   ​		we choose software debouncing (time delay function method)







## Problem 3: Toggle LED with Button

### Procedure

1. Create a new project under the directory `\repos\EC\LAB\`

- The project name is “**LAB_GPIO_DIO_multiLED”.**
- Name the source file as “**LAB_GPIO_DIO_multiLED.c”**





1. Include your library **ecGPIO.h, ecGPIO.c** in `\repos\lib\`.
2. Connect 4 LEDs externally with necessary load resistors.

- As Button B1 is Pressed, light one LED at a time, in sequence.
- Example: LED0--> LED1--> …LED3--> …LED0….



### Configuration

|    Button     |               LED                |
| :-----------: | :------------------------------: |
|  Digital In   |           Digital Out            |
| GPIOC, Pin 13 |        PA5, PA6, PA7, PB6        |
|    PULL-UP    | Push-Pull, Pull-up, Medium Speed |



### Circuit Diagram

Circuit diagram

![circuit (1)](https://user-images.githubusercontent.com/113995124/193419578-8f01aceb-1e5b-4e3e-9cdc-11fdc5798a10.png)

### Code

Your code goes here: [github_link](https://github.com/SeunghoG/EC-shga-001/blob/main/lab/LAB_GPIO_DIO_multiLED.c)

```c
/**
******************************************************************************
* @author	Seungho_Ga
* @Mod		2022.10.01
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
		but_val=GPIO_read(GPIOC, BUTTON_PIN);     // read button value
		if(but_val==0){                           
			delay_ms(70);
			cnt++;
			i=cnt%4;
			switch (i){
				case 1:                           // cnt=1 toggle PA5
					toggle(GPIOA, PA5);
				if(cnt>4){                        // if cnt number 5 we have to toggle                                                                         // PA5 and PB6
				toggle(GPIOB, PB6);}
				break;
				
				case 2:                           // cnt=2 toggle PA5
					toggle(GPIOA, PA5);
				  toggle(GPIOA, PA6);
				break;
				
				case 3:                           // cnt=3 toggle PA5
					toggle(GPIOA, PA6);
					toggle(GPIOA, PA7);
				break;
					
				default:                           // cnt=4 toggle PA5 i will zero so                                                           deflault 
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
```

### Results

LED_Toggle_Multi : [Youtube link](https://youtu.be/D1bWu2sCk1U)



### Discussion

1. Find out a typical solution for software debouncing and hardware debouncing. What method of debouncing did this NUCLEO board used for the push-button(B1)?
   * first, software debouncing this NUCLEO board is time delay with ecsystick header file.
   * second, hardware debouncing in this NUCLEO board is this RC circuit![Nucleo_hardware_debouncing](https://user-images.githubusercontent.com/113995124/193418905-d62d6121-5ec5-4065-ada9-6e2a40e42c48.jpg)





### Reference

* Schmitt Trigger / 슈미트 트리거 . (2021-11-13). http://www.ktword.co.kr/test/view/view.php?m_temp1=4566.





### Troubleshooting

1.  I decided to use switch function but i don't know about default value. So i couldn't define when the rest 0.

   Use default, we can solve this problem and when we use switch function, Most of them recommended use 

   default value

   

2. ```c
   switch (i){
   				case 1:                           // cnt=1 toggle PA5
   					toggle(GPIOA, PA5);
   				if(cnt>4){                        // if cnt number 5 we have to toggle    
                       toggle(GPIOA, PA5);               // PA5 and PB6
   				toggle(GPIOB, PB6);}
   				break;
   				
   ```

   This is my first attempt to multi toggle LED when I operated this code PA5 LED on all time.

   ```c
   switch (i){
   				case 1:
   					toggle(GPIOA, PA5);
   				if(cnt>4){
   				toggle(GPIOB, PB6);}
   				break;
   ```

   This is last attempt code. when cnt pass 5, we have to toggle two LED pin so we use 'if'. In if statement we have to toggle only last LEDpin. so I can provided to LED on all time.