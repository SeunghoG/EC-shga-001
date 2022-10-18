# LAB: EXTI_SysTick

**Date:** 2022-10-15

**Author/Partner:** Seungho-Ga / Dabin-Heo

**Github:**  [Github repository](https://github.com/SeunghoG/EC-shga-001)

**Demo Video:** [youtube_link](https://www.youtube.com/channel/UC3ic6GFpWs-oOpynFHUq8Qg)





## Introduction

In this lab, you are required to create two simple programs: toggling multiple LEDs with a push-button input and displaying the number counting from 0 to 9 at 1 second rate on a 7-segment display.                                                                                   



## Requirement

### Hardware

- MCU
  - NUCLEO-F411RE
- Actuator/Sensor/Others:
  - 4 LEDs and load resistance
  - 7-segment display(5101ASR)
  - Array resistor (330 ohm)
  - breadboard

### **Software**

- Keil uVision, CMSIS, EC_HAL library





## Problem 1: LED Toggle with EXTI Button

A program that toggles multiple LEDs with a push-button input using external interrupt.

#### Create HAL library

Declare and Define the following functions in your library. You must

update your header files located in the directory `EC \lib\`.

**ecEXTI.h**

```c
void EXTI_init(GPIO_TypeDef *port, int pin, int trig_type, int priority);
void EXTI_enable(uint32_t pin);  // mask in IMR
void EXTI_disable(uint32_t pin);  // unmask in IMR
uint32_t  is_pending_EXTI(uint32_t pin);
void clear_pending_EXTI(uint32_t pin);
```



#### Procedure

1. Create a new project under the directory `\repos\EC\LAB\LAB_EXTI`

- The project name is “**LAB_EXTI”.**
- Create a new source file named as “**LAB_EXTI.c”**

> You MUST write your name on the source file inside the comment section.

2. Include your updated library in `\repos\EC\lib\` to your project.

- **ecGPIO.h, ecGPIO.c**
- **ecRCC.h, ecRCC.c**
- **ecEXTI.h, ecEXTI.c**

1. Connect 4 LEDs externally on a breadboard.
2. Toggle LEDs, turning on one LED at a time by pressing the push button.
   - Example: LED0--> LED1--> …LED3--> …LED0….
3. You must use your library function of EXTI.
4. Refer to the [sample code](https://ykkim.gitbook.io/ec/firmware-programming/example-code#button-interrupt)

#### Configuration

|    Button     |               LED                |
| :-----------: | :------------------------------: |
|  Digital In   |           Digital Out            |
| GPIOC, Pin 13 |        PA5, PA6, PA7, PB6        |
|    PULL-UP    | Push-Pull, Pull-up, Medium Speed |

#### Circuit Diagram

![circuit (1)](https://user-images.githubusercontent.com/113995124/193419578-8f01aceb-1e5b-4e3e-9cdc-11fdc5798a10.png)

#### Discussion

1. To detect an external signal we can use two different methods: polling and interrupt. What are the advantages and disadvantages of each approach?

   Polling  -> advantage : simple Implementation, good for single I/O cases, doesn't need extra hardware

   ​              -> disadvantage : Inefficient for complex systems, may not be fast enough for requirements

   Interrupts  -> advantage :Efficient for complex systems, can be ignored, can be prioritized

   ​                    -> disadvantage : Trade off of hardware complexity, can make debugging difficult       				                                                 	                                               due to unanticipated random occurrences

2. What would happen if the EXTI interrupt handler does not clear the interrupt pending flags? Check with your code

   ```c
   void EXTI15_10_IRQHandler(void) {  
   	if (is_pending_EXTI(BUTTON_PIN)) {
   		cnt++;
   		LED_toggle(cnt);
   		clear_pending_EXTI(BUTTON_PIN); // cleared by writing '1'
   	}
   }
   
   uint32_t is_pending_EXTI(uint32_t pin){
   	uint32_t EXTI_PRx = 1<<pin;     	// check  EXTI pending 	
   	return ((EXTI->PR & EXTI_PRx) == EXTI_PRx);
   }
   
   void clear_pending_EXTI(uint32_t pin){
   	EXTI->PR |= 1<<pin;     // clear EXTI pending 
   }
   ```


* If you don't clear the interrupt pending flags, pending register always be '0'. This means that we can only try interrupt at once.

  (Interrupts function operates at pending flags 0 - falling edge trigger- )

  #### Code

  [Source Code](https://github.com/SeunghoG/EC-shga-001/blob/main/lab/LAB_EXTI.c)

  

  ```c
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
  	
      // Use 16Mz system clock
      RCC_HSI_init();
      // LED initialization
      LED_init();
  	// Button initialization & interrupts initialization
  	EXTI_init(GPIOC, BUTTON_PIN, FALL, 0);
  	GPIO_init(GPIOC, BUTTON_PIN, INPUT);
  	GPIO_pupd(GPIOC, BUTTON_PIN, 1);
  }
  // Interrupt function
  void EXTI15_10_IRQHandler(void) {  
  	if (is_pending_EXTI(BUTTON_PIN)) {
  		cnt++;
  		LED_toggle(cnt);
  		clear_pending_EXTI(BUTTON_PIN); // cleared by writing '1'
  	}
  }
  ```

  

  #### Results

  Add demo video link: [youtube_link](https://youtu.be/tjvy--yOQos)

  

## Problem 2: Counting number on 7-Segment

Display the number 0 to 9 on the 7-segment LED at the rate of 1 sec. After displaying up to 9, then it should display ‘0’ and continue counting.

When the button is pressed, the number should be reset ‘0’ and start counting again.

#### Create HAL library

Declare and Define the following functions in your library. You must

update your header files located in the directory `EC \lib\`.

**ecSysTick.h**

```c
void SysTick_init(uint32_t msec);
void delay_ms(uint32_t msec);
uint32_t SysTick_val(void);
void SysTick_reset (void);
void SysTick_enable(void);
void SysTick_disable (void)
```



### Procedure

1. Create a new project under the directory

   `\repos\EC\LAB\LAB_EXTI_SysTick`

- The project name is “**LAB_EXTI_SysTick”.**
- Create a new source file named as “**LAB_EXTI_SysTick.c”**



2. Include your updated library in `\repos\EC\lib\` to your project.

   * **ecGPIO.h, ecGPIO.c**

   * **ecRCC.h, ecRCC.c**

   * **ecEXTI.h, ecEXTI.c**

   * **ecSysTick.h, ecSysTick.c**

     

     1. First, check if every number, 0 to 9, can be displayed properly on the 7-segment.

     2. Then, create a code to display the number counting from 0 to 9 and repeat at the rate of 1 second.

     3. When the button is pressed, it should start from '0' again. Use EXTI for this reset.

     4. Refer to the [sample code](https://ykkim.gitbook.io/ec/stm32-m4-programming/example-code#systick-interrupt)

     



### Configuration

![임베핀표](https://user-images.githubusercontent.com/113995124/194503833-06774ce0-b962-47c1-9163-9265b3ce8f4c.PNG)

### Circuit Diagram

![KakaoTalk_20220930_160536835](https://user-images.githubusercontent.com/113995124/196032486-e639dee3-ab09-4614-b6f3-47a655954020.png)







### Code

 [SourceCode](https://github.com/SeunghoG/EC-shga-001/blob/main/lab/LAB_EXTI_SysTick.c)

* SourceCode

```c
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
	RCC_PLL_init();    // Use 84Mhz
	SysTick_init();
	sevensegment_init();
	
	EXTI_init(GPIOC, BUTTON_PIN, FALL, 0); // Use Interrupt
	
    // Button initialization
    GPIO_init(GPIOC, BUTTON_PIN, INPUT);
	GPIO_pupd(GPIOC, BUTTON_PIN, 1);
}

void EXTI15_10_IRQHandler(void) {  
	if (is_pending_EXTI(BUTTON_PIN)) {
		count=-1;
		clear_pending_EXTI(BUTTON_PIN); // cleared by writing '1'
	}
}
```

### Results

[demo video link](https://youtu.be/3ZuAnkebpzg)



## Reference

> https://slideplayer.com/slide/4609666/









