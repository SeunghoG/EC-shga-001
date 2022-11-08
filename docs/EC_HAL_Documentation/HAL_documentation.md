[TOC]





# HAL documentation

Written by:   Seung-Ho-Ga

Course:  Embedded Controller

Program: C/C++

IDE/Compiler: Keil uVision 5

OS: WIn10

MCU:  STM32F411RE (Nucleo-64)

## **Terminology**

1. Register : Fast, small memory for specific purpose
   * Peripherals : Configuration I/O memory
   * CPU
   * GPIO
2. Voltaile : Variables that are not automatically cleared
3. Stack : Temporary data storage like local variables
4. Heap : Dynamic data storage

## **Bitwise**

* **x>>y :** shift x, (->) direction as y value

- **check a_k:** val = (a>>6) & 1

- **check a_k, a_k+1:** val = (a >> 5) & (3)

- **set a_k:** a |= (1 << k)

- **set a_k, a_k+1:**  a |= (3 << k)

- **clear a_k:** a &= ~(1 << k),  PA &= ~(5 << 1): clear port 2, 4

- **Toggle a_k:** a ^= 1 << k (^= : XOR)

## **ecRCC.c**

### Internal Clock (HSI) for GPIO

1. **Enable HSI and choose as SYSCLK source**
   - Enable HSI: **(RCC->CR: HSION=1)** 
   - Wait until HSI is stable and ready: **(RCC->CR: HSIRDY? 1)** 
   - Choose the system clock switch : **(RCC->CFGR: SW = 00)** 
   - Check if the selected source is correct: **(RCC->CFGR: SWS ? 00)**

2. Configure HSI (optional) 
   - Calibration for RC oscillator: (RCC->CR: HSICAL, HSITRIM)
3. Configure APB/AHB Prescaler (optional) 
   - Change Prescaler: RCC->CFGR: HPRE, PPRE

4. **Enable GPIOx clock(AHB1ENR )** 
   - Enable (RCC_AHB1ENR) for PORTx

#### void RCC_HSI_init()

```c
#include "stm32f4xx.h"
#include "ecRCC.h"
volatile int EC_SYSCLK=16000000;
volatile int EC_SYSCLK=16000000;                                       // Question
```



```c
void RCC_HSI_init() {
  // Enable High Speed Internal Clock (HSI = 16 MHz)
  //RCC->CR |= ((uint32_t)RCC_CR_HSION);                                // Enable HSI
	RCC->CR |= 0x00000001U;
	
  // wait until HSI is ready
  //while ( (RCC->CR & (uint32_t) RCC_CR_HSIRDY) == 0 ) {;}
	while ( (RCC->CR & 0x00000002U) == 0 ) {;}                          // Wait until HSI is stable and                                                                                ready
	
  // Select HSI as system clock source 
  RCC->CFGR &= (uint32_t)(~RCC_CFGR_SW); 								// not essential
  RCC->CFGR |= (uint32_t)RCC_CFGR_SW_HSI; 								//00: Choose the system clock switch

  // Wait till HSI is used as system clock source
  while ((RCC->CFGR & (uint32_t)RCC_CFGR_SWS) != 0 );                   //check if the selected source is                                                                             correct
		   
  //EC_SYSTEM_CLK=16000000;
  //EC_SYSCLK=16000000;
    EC_SYSCLK=16000000;
}
```

### PLL for GPIO

1. **Enable either (HSI, or HSE) for PLL and Choose PLL for System Clock**
   - Enable HSE or HSI: **(RCC->CR : HSION=1)**
   - Wait until HSE or HSI is stable: **(RCC->CR : HSIRDY? 1)** 
   - Choose PLL for system clock switch : **(RCC->CFGR : SW = 10)**
   - Check if PLL selection is correct: **(RCC->CFGR : SWS ? 10)**

2. **Select the clock source for PLL**
   - Select the PLL source(HSI or HSE): **(RCC->PLLCFGR : PLLSRC= 0 or 1)**
3. (Optional)Configure PLL parameters
   - Select (M/N/P): **(RCC->PLLCFGR : PLLM, PLLN, …)**

4. **Enable PLL**
   - Enable main PLL: **(RCC->CR : PLLON=1), (RCC->CR : PLLRDY?0)**
5. (Optional)Configure APB/AHB Prescaler 
   - Change Prescaler: (RCC->CFGR : HPRE, PPRE)
6. **Enable GPIOx clock(AHB1ENR )**
   - Enable (RCC_AHB1ENR) for PORTx





#### void RCC_PLL_init()

```c
void RCC_PLL_init() {	
	// To correctly read data from FLASH memory, the number of wait states (LATENCY)
    // must be correctly programmed according to the frequency of the CPU clock
    // (HCLK) and the supply voltage of the device.	
    // Reference Manual 45page [put manual link]
	FLASH->ACR &= ~FLASH_ACR_LATENCY;
	FLASH->ACR |=  FLASH_ACR_LATENCY_2WS;
		
	// Enable the Internal High Speed oscillator (HSI)
    // RCC->CR |= 0x00000001U;
	   RCC->CR |= RCC_CR_HSION;
    
    // Wait 
	//while((RCC->CR & 0x00000002U) == 0 ) {;}
      while((RCC->CR & RCC_CR_HSIRDY) == 0);
    
	
	// Disable PLL for configuration
	RCC->CR    &= ~RCC_CR_PLLON;
	
	// Select clock source to PLL
	RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLSRC; 		// Set source for PLL: clear bits
	RCC->PLLCFGR |= RCC_PLLCFGR_PLLSRC_HSI;     // Set source for PLL: 0 =HSI, 1 = HSE
	
	// Make PLL as 84 MHz
	// PLLM: 2~63, PLLN: 50~432, PLLP= 2.

    // f(VCO clock) = f(PLL clock input) * (PLLN / PLLM) = 16MHz * 84/8 = 168 MHz
	// f(PLL_R) = f(VCO clock) / PLLP = 168MHz/2 = 84MHz
	RCC->PLLCFGR = (RCC->PLLCFGR & ~RCC_PLLCFGR_PLLN) | 84U << 6;
	RCC->PLLCFGR = (RCC->PLLCFGR & ~RCC_PLLCFGR_PLLM) | 8U ; 
	RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLP;  // 00: PLLP = 2, 01: PLLP = 4, 10: PLLP = 6, 11: PLLP = 8	
	
	
	// Enable PLL after configuration
	RCC->CR   |= RCC_CR_PLLON; 
	// Wait till PLL is used as system clock source
    while((RCC->CR & RCC_CR_PLLRDY)>>25 != 0);           // PLLRDY 0: PLL unlocked
                                                         // PLLRDY 1: PLL locked
	
	// Select PLL as system clock
	RCC->CFGR &= ~RCC_CFGR_SW;          // clear
	RCC->CFGR |= RCC_CFGR_SW_PLL;       // set
	
	// Wait until System Clock has been selected
	while ((RCC->CFGR & RCC_CFGR_SWS) != 8UL);           // Why 8UL
	
	// The maximum frequency of the AHB and APB2 is 100MHz,
	// The maximum frequency of the APB1 is 50 MHz.
	RCC->CFGR &= ~RCC_CFGR_HPRE;  		// AHB prescaler = 1; SYSCLK not divided (84MHz)
	RCC->CFGR &= ~RCC_CFGR_PPRE1; 		// APB high-speed prescaler (APB1) = 2, HCLK divided by 2 (42MHz)
	RCC->CFGR |=  RCC_CFGR_PPRE1_2;
	RCC->CFGR &= ~RCC_CFGR_PPRE2; 		// APB high-speed prescaler (APB2) = 1, HCLK not divided	(84MHz)
	
	EC_SYSCLK=84000000;
}

```

#### void RCC_GPIOX_enable (X=A, B, C, D, E, ...)

```c
void RCC_GPIOA_enable()
{
	// HSI is used as system clock         
	// RCC Peripheral Clock Enable Register 
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
}

void RCC_GPIOB_enable()
{
	// HSI is used as system clock         
	// RCC Peripheral Clock Enable Register 
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
}

void RCC_GPIOC_enable()
{
	// HSI is used as system clock         
	// RCC Peripheral Clock Enable Register 
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
}

void RCC_GPIOD_enable()
{
	// HSI is used as system clock         
	// RCC Peripheral Clock Enable Register 
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
}

void RCC_GPIOE_enable()
{
	// HSI is used as system clock         
	// RCC Peripheral Clock Enable Register 
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;
}
```



## **ecGPIO.c (Input&Output)** 

### Process of GPIOx register initiation 

**Ouput setting:**

0. Enable Peripheral Clock (**AHB1ENR**)

1. Configure as Digital Output (**MODER**)

2. Configure pull-up/down resistors (**PUPDR**)

3. For Output: Configure Output Type (**OTYPE**)

4. For Output: Configure Output Speed (**OSPEEDR**)

5. Output Data **(ODR)**

**Input setting:**

0. Enable Peripheral Clock (**AHB1ENR**)

1. Configure as Digital Output (**MODER**)

2. Configure pull-up/down resistors (**PUPDR**)

3. Input Data **(IDR)**

#### void GPIO_init()

```c
// mode: Input(0), Output(1), AlterFunc(2), Analog(3)
void GPIO_init(GPIO_TypeDef *Port, int pin, int mode){        
	if (Port == GPIOA)
		RCC_GPIOA_enable();
	if (Port == GPIOB)
		RCC_GPIOB_enable();
	if (Port == GPIOC)
		RCC_GPIOC_enable();
	if (Port == GPIOD)
		RCC_GPIOD_enable();
	if (Port == GPIOE)
		RCC_GPIOE_enable();
	
	// You can also make a more general function of
	// void RCC_GPIO_enable(GPIO_TypeDef *Port);
    
	GPIO_mode(Port, pin, mode);
	
}
```



#### void GPIO_mode()

```c
// GPIO Mode: Input(00), Output(01), AlterFunc(10), Analog(11)
void GPIO_mode(GPIO_TypeDef *Port, int pin, int mode){
   Port->MODER &= ~(3UL<<(2*pin));    //clear
   Port->MODER |= mode<<(2*pin);      //set
}
```

#### void GPIO_speed()

```c
// GPIO Speed: Low speed (00), Medium speed (01), Fast speed (10), High speed (11)
void GPIO_ospeed(GPIO_TypeDef *Port, int pin, int speed){
	Port->OSPEEDR &= ~(3UL<<(2*pin));    //clear
	Port->OSPEEDR |= speed<<(2*pin);     //set
}
```

#### void GPIO_otype()

```c
// GPIO Push-Pull: No pull-up, pull-down (00), Pull-up (01), Pull-down (10), Reserved (11)
void GPIO_pupd(GPIO_TypeDef *Port, int pin, int pupd){
	Port->PUPDR &= ~(3<<(pin*2));      //clear
	Port->PUPDR |= pupd<<(pin*2);      //set
}
```

#### int GPIO_read()

```c
int GPIO_read(GPIO_TypeDef *Port, int pin){
  int bitVal = ((Port->IDR)>>pin)&1;   // IDR approach move right as pin & 1
	return bitVal;
}
```

#### void toggle()

```c
void toggle(GPIO_TypeDef *Port, int pin){
	Port->ODR ^= (1<<pin);
}
```

#### void LED_init()

```c
void LED_init(void){
	GPIO_init(GPIOA, PA5, OUTPUT);
	GPIO_otype(GPIOA,PA5, 0UL);       // 0:Push-Pull
	GPIO_pupd(GPIOA,PA5, 1UL);        // 01: Pull-up 
	GPIO_ospeed(GPIOA,PA5, 1UL);      // 01:Medium Speed
	
	GPIO_init(GPIOA, PA6, OUTPUT);
	GPIO_otype(GPIOA,PA6, 0UL);       // 0:Push-Pull
	GPIO_pupd(GPIOA,PA6, 1UL);        // 01: Pull-up
	GPIO_ospeed(GPIOA,PA6, 1UL);      // 01:Medium Speed
	
	GPIO_init(GPIOA, PA7, OUTPUT);
	GPIO_otype(GPIOA,PA7, 0UL);       // 0:Push-Pull
	GPIO_pupd(GPIOA,PA7, 1UL);        // 01: Pull-up
	GPIO_ospeed(GPIOA,PA7, 1UL);      // 01:Medium Speed
	
	GPIO_init(GPIOB, PB6, OUTPUT);
	GPIO_otype(GPIOB,PB6, 0UL);       // 0:Push-Pull
	GPIO_pupd(GPIOB,PB6, 1UL);        // 01: Pull-up
	GPIO_ospeed(GPIOB,PB6, 1UL);      // 01:Medium Speed
	
	GPIO_write(GPIOA, PA5,LOW);       //Insurance
	GPIO_write(GPIOA, PA6,LOW);
	GPIO_write(GPIOA, PA7,LOW);
	GPIO_write(GPIOB, PB6,LOW);
}
```

#### void sevensegment_init()

```c
void sevensegment_init(void){
	
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
```

#### void LED_toggle()

```c
void LED_toggle(int _cnt){
			
			int i = 0;
			i=_cnt%4;
	
			switch (i){
				case 1:
					toggle(GPIOA, PA5);
				if(_cnt>4){
				toggle(GPIOB, PB6);
				}
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
```

#### void sevensegment_decode()

```c
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

## **ecEXTI.c (Interrupt)**

### Interrupt

- **ISR (Interrupt Service Routine):** 함수명
- Vector table : RM p.201 참조
- **AHB1**: GPIO, RCC
- **APB1**: TIM2~5
- **APB2**: EXTI, TIM1,9,10,11, SYSCFG



### NVIC Configuration

- NVIC SysTick Interrupt priority 
- NVIC SysTick Enable





### EXTI Configuration

#### Digital Input Setting

1. Enable GPIO peripheral clock **(RCC->AHB1ENR )** 

2. Configure digital input pin(push-button) using GPIO registers.

#### EXTI Setting

3. Connect External Line to the GPIO Port

4. Enable SYSCFG peripheral clock. **(RCC->APB2ENR)** 
5. Connect the corresponding external line to GPIO **(SYSCFG->EXTICR)** 
6. Configure the trigger edge. **(EXTI->FTSR/RTSR)** 
7. Configure Interrupt mask (Enable/Disable EXTI) **(EXTI->IMR)** / Enable = unmask

```c
#include "ecGPIO.h"
#include "ecSysTick.h"
#include "ecEXTI.h"
```

#### void EXTI_init()

```c
void EXTI_init(GPIO_TypeDef *Port, int Pin, int trig_type,int priority){

	// SYSCFG peripheral clock enable	
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;		
	
	// Connect External Line to the GPIO
	int EXTICR_port;
	if		(Port == GPIOA) EXTICR_port = 0;
	else if	(Port == GPIOB) EXTICR_port = 1;
	else if	(Port == GPIOC) EXTICR_port = 2;
	else if	(Port == GPIOD) EXTICR_port = 3;
	else 	EXTICR_port = 4;
	
	SYSCFG->EXTICR[3] &= ~SYSCFG_EXTICR4_EXTI13;		// clear 4 bits
	SYSCFG->EXTICR[3] |= SYSCFG_EXTICR4_EXTI13_PC;		// set 4 bits
	
	// Configure Trigger edge
	if (trig_type == FALL) EXTI->FTSR |= EXTI_FTSR_TR13;        // Falling trigger enable 
	else if	(trig_type == RISE) EXTI->RTSR |= EXTI_RTSR_TR13;   // Rising trigger enable 
	else if	(trig_type == BOTH) {			                    // Both falling/rising trigger enable
		EXTI->RTSR |= EXTI_RTSR_TR13; 
		EXTI->FTSR |= EXTI_FTSR_TR13;
	} 
	
	// Configure Interrupt Mask (Interrupt enabled)
	EXTI->IMR  |= EXTI_IMR_MR13;     // not masked
	
	
	// NVIC(IRQ) Setting
	int EXTI_IRQn = 0;
	if (Pin < 5) 	EXTI_IRQn = Pin + 6;
	else if	(Pin < 10) 	EXTI_IRQn = EXTI9_5_IRQn;
	else 			EXTI_IRQn = EXTI15_10_IRQn;
								
	NVIC_SetPriority(EXTI_IRQn, priority);	// EXTI priority
	NVIC_EnableIRQ(EXTI_IRQn); 	            // EXTI IRQ enable
}
```



#### void EXTI_enable()

```c
void EXTI_enable(uint32_t pin) {
	EXTI->IMR |= 1<<pin;     // not masked (i.e., Interrupt enabled)
}
```

#### void EXTI_disable()

```c
void EXTI_disable(uint32_t pin) {
	EXTI->IMR &= ~1<<pin;     // masked (i.e., Interrupt disabled)
}
```

#### uint32_t is_pending_EXTI()

```c
uint32_t is_pending_EXTI(uint32_t pin){
	uint32_t EXTI_PRx = 1<<pin;     	// check  EXTI pending 	
	return ((EXTI->PR & EXTI_PRx) == EXTI_PRx);
}
```

#### void clear_pending_EXTI()

```c
void clear_pending_EXTI(uint32_t pin){
	EXTI->PR |= 1<<pin;     // clear EXTI pending 
}
```





## **ecSysTick.c (Interrupt)**

### SysTick Configuration

1. Disable SysTick Timer 
   - **SysTick->CTRL ENABLE=0** (SM p.247)
2. Choose clock signal: System clock or ref. clock(STCLK) 
   - **SysTick->CTRL CLKSOURCE = 0 or 1**
     - 0: External clock
     - 1: Processor clock
3. Choose to use Tick Interrupt (timer goes 1->0) 
   - **SysTick->CTRL TICKINT = 0 or 1**
     - 0: Disable SysTick down counter
     - 1: Enable SysTick down counter
4. Write reload Counting value (24-bit) 
   - **SysTick->LOAD RELOAD = (value-1)** 
5. Start SysTick Timer 
   - **SysTick->CTRL ENABLE=1** 
6. (option) Read or Clear current counting value 
   - Read from SysTick->VAL 
   - Write clears value 
7. (option) 10msec calibration value 
   - SysTick->CALIB TENMS = 10ms clock cycle



```c
#include "ecSysTick.h"
#define MCU_CLK_PLL 84000000
#define MCU_CLK_HSI 16000000
volatile uint32_t msTicks;
```

#### void SysTick_init()

```c
void SysTick_init(void){	
	//  SysTick Control and Status Register
	SysTick->CTRL = 0;											// Disable SysTick IRQ and SysTick Counter

	// Select processor clock
	SysTick->CTRL |= SysTick_CTRL_CLKSOURCE_Msk;                // 1 = processor clock;  0 = external clock

	// uint32_t MCU_CLK=EC_SYSTEM_CLK
	// SysTick Reload Value Register
	SysTick->LOAD = 84000000 / 1000 - 1;						// 1ms, for HSI PLL = 84MHz.

	// SysTick Current Value Register
	SysTick->VAL = 0;

	// Enables SysTick exception request
    // 1 = counting down to zero asserts the SysTick exception request
	SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;                  
	
	// Enable SysTick IRQ and SysTick Timer
	SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;
		
	NVIC_SetPriority(SysTick_IRQn, 16);		// Set Priority to 1
	NVIC_EnableIRQ(SysTick_IRQn);			// Enable interrupt in NVIC
}
```

#### void SysTick_Handler()

```c
void SysTick_Handler(void){
	SysTick_counter();	
}
```

#### void SysTick_counter()

```c
void delay_ms (uint32_t mesc){
  uint32_t curTicks;
  curTicks = msTicks;
  while ((msTicks - curTicks) < mesc);
	msTicks = 0;
}
```

#### void SysTick_reset()

```c
void SysTick_reset(void)
{
    // SysTick Current Value Register
	SysTick->VAL = 0;
}
```

#### uint32_t SysTick_val()

```c
uint32_t SysTick_val(void) {
	return SysTick->VAL;
}
```

## **ecTIM.c (Interrupt)**

#### void TIM_init_usec()

```c
void TIM_init_usec(TIM_TypeDef* timerx, uint32_t usec){ 
	
// 1. Enable Timer CLOCK
	if(timerx ==TIM1) RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
	else if(timerx ==TIM2) RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
	else if(timerx ==TIM3) RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
	else if(timerx ==TIM4) RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
	else if(timerx ==TIM5) RCC->APB1ENR |= RCC_APB1ENR_TIM5EN;
	else if(timerx ==TIM9)  RCC->APB2ENR |= RCC_APB2ENR_TIM9EN ;
  	else if(timerx ==TIM10) RCC->APB2ENR |= RCC_APB2ENR_TIM10EN;
  	else if(timerx ==TIM11) RCC->APB2ENR |= RCC_APB2ENR_TIM11EN;
	
// 2. Set CNT period
	TIM_period_us(timerx,usec); 
		
// 3. CNT Direction
	timerx->CR1= TIM_CR1_DIR;		// Upcounter	
	
// 4. Enable Timer Counter
	timerx->CR1 |= TIM_CR1_CEN;		
}
```

#### void TIM_init_msec()

```c
void TIM_init_msec(TIM_TypeDef* timerx, uint32_t msec){ 
	
// 1. Enable Timer CLOCK
	if(timerx ==TIM1) RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
	else if(timerx ==TIM2) RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
	else if(timerx ==TIM3) RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
	else if(timerx ==TIM4) RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
	else if(timerx ==TIM5) RCC->APB1ENR |= RCC_APB1ENR_TIM5EN;
	else if(timerx ==TIM9)  RCC->APB2ENR |= RCC_APB2ENR_TIM9EN ;
  	else if(timerx ==TIM10) RCC->APB2ENR |= RCC_APB2ENR_TIM10EN;
  	else if(timerx ==TIM11) RCC->APB2ENR |= RCC_APB2ENR_TIM11EN;
	
// 2. Set CNT period
	TIM_period_ms(timerx,msec); 
		
// 3. CNT Direction
	timerx->CR1= TIM_CR1_DIR;		// Upcounter	
	
// 4. Enable Timer Counter
	timerx->CR1 |= TIM_CR1_CEN;		
}
```

#### void TIM_period_us()

```c
void TIM_period_us(TIM_TypeDef *TIMx, uint32_t usec){   
	// Period usec = 1 to 1000

	//16bit: 0~65000 
	//32bit: 0~4,294,967,295
	// 1us(1MHz, ARR=1) to 65msec (ARR=0xFFFF)
	uint32_t prescaler = 84;  //1[MHz]=1[us]
	uint32_t ARRval= usec;  // 84MHz/1000000 us
	
	TIMx->PSC = prescaler-1;					
	TIMx->ARR = ARRval-1;					
}
```

#### void TIM_period_ms()

```c
void TIM_period_ms(TIM_TypeDef* TIMx, uint32_t msec){ 
	// Period msec = 1 to 6000
	// 0.1ms(10kHz, ARR=1) to 6.5sec (ARR=0xFFFF)
	uint32_t prescaler = 8400; //10[kHz]=0.1[ms]
	uint16_t ARRval=10*msec;  			// 84MHz/1000ms

	TIMx->PSC = prescaler-1;					
	TIMx->ARR = ARRval-1;							
}
```

#### void TIM_INT_init_usec()

```c
void TIM_INT_init_usec(TIM_TypeDef* timerx, uint32_t usec){
// 1. Initialize Timer	
	TIM_init_usec(timerx,usec);
	
// 2. Enable Update Interrupt
	TIM_INT_enable(timerx);
	
// 3. NVIC Setting
	uint32_t IRQn_reg =0;
	if(timerx ==TIM1)       IRQn_reg = TIM1_UP_TIM10_IRQn;
	else if(timerx ==TIM2)  IRQn_reg = TIM2_IRQn;
	else if(timerx ==TIM3)  IRQn_reg = TIM3_IRQn;
	else if(timerx ==TIM4)  IRQn_reg = TIM4_IRQn;
	else if(timerx ==TIM5)  IRQn_reg = TIM5_IRQn;
	else if(timerx ==TIM9)  IRQn_reg = TIM1_BRK_TIM9_IRQn;
  	else if(timerx ==TIM10) IRQn_reg = TIM1_UP_TIM10_IRQn;
  	else if(timerx ==TIM11) IRQn_reg = TIM1_TRG_COM_TIM11_IRQn;
	
	NVIC_EnableIRQ(IRQn_reg);				
	NVIC_SetPriority(IRQn_reg,2);
}
```

#### void TIM_INT_init_msec()

```c
void TIM_INT_init_msec(TIM_TypeDef* timerx, uint32_t msec){
// 1. Initialize Timer	
	TIM_init_msec(timerx,msec);
	
// 2. Enable Update Interrupt
	TIM_INT_enable(timerx);
	
// 3. NVIC Setting
	uint32_t IRQn_reg =0;
	if(timerx ==TIM1)       IRQn_reg = TIM1_UP_TIM10_IRQn;
	else if(timerx ==TIM2)  IRQn_reg = TIM2_IRQn;
	else if(timerx ==TIM3)  IRQn_reg = TIM3_IRQn;
	else if(timerx ==TIM4)  IRQn_reg = TIM4_IRQn;
	else if(timerx ==TIM5)  IRQn_reg = TIM5_IRQn;
	else if(timerx ==TIM9)  IRQn_reg = TIM1_BRK_TIM9_IRQn;
  	else if(timerx ==TIM10) IRQn_reg = TIM1_UP_TIM10_IRQn;
  	else if(timerx ==TIM11) IRQn_reg = TIM1_TRG_COM_TIM11_IRQn;
	
	
	NVIC_EnableIRQ(IRQn_reg);				
	NVIC_SetPriority(IRQn_reg,2);
}
```

#### void TIM_INT_enable()

```c
void TIM_INT_enable(TIM_TypeDef* timerx){
	timerx->DIER |=1<<0;			// Enable Timer Update Interrupt		
}
```

#### void TIM_INT_disable()

```c
void TIM_INT_disable(TIM_TypeDef* timerx){
	timerx->DIER &= ~(1<<0);				// Disable Timer Update Interrupt
}
```

#### uint32_t is_UIF()

```c
uint32_t is_UIF(TIM_TypeDef *TIMx){
	return ((TIMx->SR &TIM_SR_UIF)==TIM_SR_UIF);
}
```

#### void clear_UIF()

```c
void clear_UIF(TIM_TypeDef *TIMx){
	TIMx->SR &= ~TIM_SR_UIF;
}
```

#### void ICAP_init()

```c
void ICAP_init(IC_t *ICx, GPIO_TypeDef *port, int pin){
// 0. Match Input Capture Port and Pin for TIMx
	ICx->port = port;
	ICx->pin  = pin;
	ICAP_pinmap(ICx);	  										// Port, Pin --(mapping)--> TIMx, Channel
	
	TIM_TypeDef *TIMx = ICx->timer;
	int TIn = ICx->ch; 		
	int ICn = TIn;
	ICx->ICnum = ICn;													// (default) TIx=ICx

// GPIO configuration ---------------------------------------------------------------------	
// 1. Initialize GPIO port and pin as AF
	GPIO_init(port, pin, 2UL);  							// GPIO init as AF=2
	GPIO_ospeed(port, pin, 3UL);  						// speed VHIGH=3	
	GPIO_pupd(port,pin,0UL);

// 2. Configure GPIO AFR by Pin num.
	if(TIMx == TIM1 || TIMx == TIM2)											 port->AFR[pin >> 3] |= 0x01 << (4*(pin % 8)); // TIM1~2
	else if  ((TIMx == TIM3) || (TIMx == TIM4) || (TIMx == TIM5)) port->AFR[pin >> 3] |= 0x02 << (4*(pin % 8));  // TIM3~5
	else if  ((TIMx == TIM9) || (TIMx == TIM10) || (TIMx == TIM11)) port->AFR[pin >> 3] |= 0x03 << (4*(pin % 8));  // TIM9~11

	
// TIMER configuration ---------------------------------------------------------------------			
// 1. Initialize Timer 
	TIM_init_usec(TIMx, 100);
// 2. Initialize Timer Interrpt 
	TIM_INT_init_usec(TIMx, 100);        					// TIMx Interrupt initialize 
// 3. Modify ARR Maxium for 1MHz
	TIMx->PSC = 84-1;						  					// Timer counter clock: 1MHz(1us)  for PLL
	TIMx->ARR = 0xFFFF;											// Set auto reload register to maximum (count up to 65535)
// 4. Disable Counter during configuration
	TIMx->CR1 &= ~TIM_CR1_CEN;  						// Disable Counter during configuration
	
// Input Capture configuration ---------------------------------------------------------------------			
// 1. Select Timer channel(TIx) for Input Capture channel(ICx)
	// Default Setting
	TIMx->CCMR1 &= 	~TIM_CCMR1_CC1S;
	TIMx->CCMR1 &=	~TIM_CCMR1_CC2S;
	TIMx->CCMR2 &=	~TIM_CCMR2_CC3S;
	TIMx->CCMR2 &=	~TIM_CCMR2_CC4S;
	
	TIMx->CCMR1 |= 	TIM_CCMR1_CC1S_0;      					//01<<0   CC1S    TI1=IC1
	TIMx->CCMR1 |= 	TIM_CCMR1_CC2S_0;  				     	//01<<8   CC2s    TI2=IC2
	TIMx->CCMR2 |= 	TIM_CCMR2_CC3S_0;        				//01<<0   CC3s    TI3=IC3
	TIMx->CCMR2 |= 	TIM_CCMR2_CC4S_0;  							//01<<8   CC4s    TI4=IC4


// 2.Filter Duration (use default)

// 3.IC Prescaler (use default)

// 4.Activation Edge: CCyNP/CCyP	
	TIMx->CCER &= ~(15UL<<4*(ICn-1)) 	;					// CCy(Rising) for ICn
	TIMx->CCER &= ~(5UL<<(((ICn-1)*4)+1));
	
// 5.Enable CCy Capture, Capture/Compare interrupt
	TIMx->CCER |= 1UL<<4*(ICn-1);					// CCn(ICn) Capture Enable	

// 6.Enable Interrupt of CC(CCyIE), Update (UIE)
	TIMx->DIER |= 1UL<<ICn;					// Capture/Compare Interrupt Enable	for ICn
	TIMx->DIER |= TIM_DIER_UIE;							// Update Interrupt enable	

// 7.Enable Counter 
	TIMx->CR1	 |= TIM_CR1_CEN;							// Counter enable	
}
```

#### void ICAP_setup()

```c
void ICAP_setup(IC_t *ICx, int ICn, int edge_type){
	TIM_TypeDef *TIMx = ICx->timer;	// TIMx
	int 				CHn 	= ICx->ch;		// Timer Channel CHn
	ICx->ICnum = ICn;

// Disable  CC. Disable CCInterrupt for ICn. 
	TIMx->CCER &= ~(1UL<<4*(ICn-1));															// Capture Disable
	TIMx->DIER &= ~(1UL<<ICn);															// CCn Interrupt Disable	
	
	
// Configure  IC number(user selected) with given IC pin(TIMx_CHn)
	switch(ICn){
			case 1:
					TIMx->CCMR1 &= ~TIM_CCMR1_CC1S;											//reset   CC1S
					if (ICn==CHn) TIMx->CCMR1 |= 	TIM_CCMR1_CC1S_0;     //01<<0   CC1S    Tx_Ch1=IC1
					else TIMx->CCMR1 |= TIM_CCMR1_CC1S_1;      											//10<<0   CC1S    Tx_Ch2=IC1
					break;
			case 2:
					TIMx->CCMR1 &= ~TIM_CCMR1_CC2S;										//reset   CC2S
					if (ICn==CHn) TIMx->CCMR1 |= TIM_CCMR1_CC2S_0;     	//01<<0   CC2S    Tx_Ch2=IC2
					else TIMx->CCMR1 |= TIM_CCMR1_CC2S_1;     											//10<<0   CC2S    Tx_Ch1=IC2
					break;
			case 3:
					TIMx->CCMR2 &= ~TIM_CCMR2_CC3S;											//reset   CC3S
					if (ICn==CHn) TIMx->CCMR2 |= TIM_CCMR2_CC3S_0;	    //01<<0   CC3S    Tx_Ch3=IC3
					else TIMx->CCMR2 |= TIM_CCMR2_CC3S_1;		     				//10<<0   CC3S    Tx_Ch4=IC3
					break;
			case 4:
					TIMx->CCMR2 &= ~TIM_CCMR2_CC4S;										//reset   CC4S
					if (ICn==CHn) TIMx->CCMR2 |= TIM_CCMR2_CC4S_0;	   						  //01<<0   CC4S    Tx_Ch4=IC4
					else TIMx->CCMR2 |= TIM_CCMR2_CC4S_1;	     					//10<<0   CC4S    Tx_Ch3=IC4
					break;
			default: break;
		}


// Configure Activation Edge direction
	TIMx->CCER &= ~(5<<((4*(ICn-1))+1));	  									// Clear CCnNP/CCnP bits for ICn
	switch(edge_type){
		case IC_RISE: TIMx->CCER &= ~(5<<((4*(ICn-1))+1));	 break; //rising:  00
		case IC_FALL: TIMx->CCER |= (1<<((4*(ICn-1))+1));	 break; //falling: 01
		case IC_BOTH: TIMx->CCER |= (1<<((4*(ICn-1))+1));	 break; //both:    11
	}
	
// Enable CC. Enable CC Interrupt. 
	TIMx->CCER |= 1 << (4*(ICn - 1)); 										// Capture Enable
	TIMx->DIER |= 1 << ICn; 															// CCn Interrupt enabled	
}

```

#### void ICAP_counter_us()

```c
void ICAP_counter_us(IC_t *ICx, int usec){	
	TIM_TypeDef *TIMx = ICx->timer;	
	TIMx->PSC = 84*usec-1;						  // Timer counter clock: 1us * usec
	TIMx->ARR = 0xFFFF;									// Set auto reload register to maximum (count up to 65535)
}
```

#### uint32_t is_CCIF()

```c
uint32_t is_CCIF(TIM_TypeDef *TIMx, uint32_t ccNum){
	switch(ccNum){
	case 1 : return ((TIMx->SR & TIM_SR_CC1IF) == TIM_SR_CC1IF); 	break;
    case 2 : return ((TIMx->SR & TIM_SR_CC2IF) == TIM_SR_CC2IF); 	break;
    case 3 : return ((TIMx->SR & TIM_SR_CC3IF) == TIM_SR_CC3IF); 	break;
    case 4 : return ((TIMx->SR & TIM_SR_CC4IF) == TIM_SR_CC4IF); 	break;
  }
}
```

#### void clear_CCIF()

```c
void clear_CCIF(TIM_TypeDef *TIMx, uint32_t ccNum){
	switch(ccNum){
    case 1 : TIMx->SR &= ~TIM_SR_CC1IF; break;
    case 2 : TIMx->SR &= ~TIM_SR_CC2IF;	break;
    case 3 : TIMx->SR &= ~TIM_SR_CC3IF; break;
    case 4 : TIMx->SR &= ~TIM_SR_CC4IF;	break;
  }
}
```

#### void ICAP_pinmap()

```c
//DO NOT MODIFY THIS
void ICAP_pinmap(IC_t *timer_pin){
   GPIO_TypeDef *port = timer_pin->port;
   int pin = timer_pin->pin;
   
   if(port == GPIOA) {
      switch(pin){
         case 0 : timer_pin->timer = TIM2; timer_pin->ch = 1; break;
         case 1 : timer_pin->timer = TIM2; timer_pin->ch = 2; break;
         case 5 : timer_pin->timer = TIM2; timer_pin->ch = 1; break;
         case 6 : timer_pin->timer = TIM3; timer_pin->ch = 1; break;
         //case 7: timer_pin->timer = TIM1; timer_pin->ch = 1N; break;
         case 8 : timer_pin->timer = TIM1; timer_pin->ch = 1; break;
         case 9 : timer_pin->timer = TIM1; timer_pin->ch = 2; break;
         case 10: timer_pin->timer = TIM1; timer_pin->ch = 3; break;
         case 15: timer_pin->timer = TIM2; timer_pin->ch = 1; break;
         default: break;
      }         
   }
   else if(port == GPIOB) {
      switch(pin){
         //case 0: timer_pin->timer = TIM1; timer_pin->ch = 2N; break;
         //case 1: timer_pin->timer = TIM1; timer_pin->ch = 3N; break;
         case 3 : timer_pin->timer = TIM2; timer_pin->ch = 2; break;
         case 4 : timer_pin->timer = TIM3; timer_pin->ch = 1; break;
         case 5 : timer_pin->timer = TIM3; timer_pin->ch = 2; break;
         case 6 : timer_pin->timer = TIM4; timer_pin->ch = 1; break;
         case 7 : timer_pin->timer = TIM4; timer_pin->ch = 2; break;
         case 8 : timer_pin->timer = TIM4; timer_pin->ch = 3; break;
         case 9 : timer_pin->timer = TIM4; timer_pin->ch = 3; break;
         case 10: timer_pin->timer = TIM2; timer_pin->ch = 3; break;
         
         default: break;
      }
   }
   else if(port == GPIOC) {
      switch(pin){
         case 6 : timer_pin->timer = TIM3; timer_pin->ch = 1; break;
         case 7 : timer_pin->timer = TIM3; timer_pin->ch = 2; break;
         case 8 : timer_pin->timer = TIM3; timer_pin->ch = 3; break;
         case 9 : timer_pin->timer = TIM3; timer_pin->ch = 4; break;
         
         default: break;
      }
   }
}
```



## **ecPWM.c (Interrupt)**

#### void PWM_init()

```c
void PWM_init(PWM_t *pwm, GPIO_TypeDef *port, int pin){
// 0. Match Output Port and Pin for TIMx 	
		pwm->port = port;
		pwm->pin  = pin;
	
		PWM_pinmap(pwm);
		TIM_TypeDef *TIMx = pwm->timer;
		int CHn = pwm->ch;	
		
// 1. Initialize GPIO port and pin as AF // initialize in main source code
		
		GPIO_init(port, pin,AF);  // AF=2
		GPIO_ospeed(port, pin,3UL);  // speed VHIGH=3 
		GPIO_otype(port, pin, 0UL);  // push-pull
		GPIO_pupd(port, pin, 0UL);
// 2. Configure GPIO AFR by Pin num.				
	//  AFR[0] for pin: 0~7,     AFR[1] for pin 8~15
	//  AFR=1 for TIM1,TIM2	AFR=2 for TIM3 etc
		 uint16_t AFx = 0;
	
		if ((TIMx == TIM1) || (TIMx == TIM2)) { AFx = 1UL;}
    else if ((TIMx == TIM3) || (TIMx == TIM4) || (TIMx == TIM5)) { AFx = 2UL; }
    else if ((TIMx == TIM9) || (TIMx == TIM10) || (TIMx == TIM11)) { AFx = 3UL; }

    // ? ??? AFR ??? ???? ?? ???
    port->AFR[pin/8] &= ~(0xFUL << (4*(pin%8)));        // 4 bit clear AFRx
    port->AFR[pin/8] |= AFx << (4*(pin%8)); 
			
// 3. Initialize Timer 
		TIM_init_msec(TIMx, 1);	// with default msec=1 value.	
		 
		TIMx->CR1 &= ~TIM_CR1_CEN;	// disable counter
// 3-2. Direction of Counter
		TIMx->CR1 &= ~TIM_CR1_DIR;    // Counting direction: 0 = up-counting, 1 = down-counting
	
			
// 4. Configure Timer Output mode as PWM
	uint32_t ccVal=TIMx->ARR/2;  // default value  CC=ARR/2
	if(CHn == 1){
		TIMx->CCMR1 &= ~TIM_CCMR1_OC1M;                     // Clear ouput compare mode bits for channel 1
		TIMx->CCMR1 |= TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2; 												// OC1M = 110 for PWM Mode 1 output on ch1. #define TIM_CCMR1_OC1M_1          (0x2UL << TIM_CCMR1_OC1M_Pos)
		TIMx->CCMR1	|= TIM_CCMR1_OC1PE;                     // Output 1 preload enable (make CCR1 value changable)
		TIMx->CCR1  = ccVal; 																// Output Compare Register for channel 1 (default duty ratio = 50%)		
		TIMx->CCER &= ~TIM_CCER_CC1P;                       // select output polarity: active high	
		TIMx->CCER  |= TIM_CCER_CC1E;												// Enable output for ch1
	}
	else if(CHn == 2){
		TIMx->CCMR1 &= ~TIM_CCMR1_OC2M;                     // Clear ouput compare mode bits for channel 2
		TIMx->CCMR1 |= TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2; // OC1M = 110 for PWM Mode 1 output on ch2
		TIMx->CCMR1	|= TIM_CCMR1_OC2PE;                     // Output 1 preload enable (make CCR2 value changable)	
		TIMx->CCR2  = ccVal; 															  // Output Compare Register for channel 2 (default duty ratio = 50%)		
		TIMx->CCER &= ~TIM_CCER_CC1P;                       // select output polarity: active high	
		TIMx->CCER  |= TIM_CCER_CC2E;												// Enable output for ch2
	}
	else if(CHn == 3){
		TIMx->CCMR2 &= ~TIM_CCMR2_OC3M;                     // Clear ouput compare mode bits for channel 3
		TIMx->CCMR2 |= TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2; 															// OC1M = 110 for PWM Mode 1 output on ch3
		TIMx->CCMR2	|= TIM_CCMR2_OC3PE;                     					// Output 1 preload enable (make CCR3 value changable)	
		TIMx->CCR3	= ccVal; 															// Output Compare Register for channel 3 (default duty ratio = 50%)		
		TIMx->CCER &= ~TIM_CCER_CC1P;                       				// select output polarity: active high	
		TIMx->CCER |= TIM_CCER_CC3E;															// Enable output for ch3
	}
	else if(CHn == 4){
		TIMx->CCMR2 &= ~TIM_CCMR2_OC4M;
		TIMx->CCMR2 |= TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_2;
		TIMx->CCMR2	|= TIM_CCMR2_OC4PE;
		TIMx->CCR3	= ccVal;
		TIMx->CCER &= ~TIM_CCER_CC1P;
		TIMx->CCER |= TIM_CCER_CC4E;
	}	
	
	
	
// 5. Enable Timer Counter
	if(TIMx == TIM1) TIMx->BDTR |= TIM_BDTR_MOE;					// Main output enable (MOE): 0 = Disable, 1 = Enable	
	TIMx->CR1  |= TIM_CR1_CEN;  													// Enable counter
}

```

#### void PWM_period_ms()

```c
void PWM_period_ms(PWM_t *pwm, uint32_t msec){
	TIM_TypeDef *TIMx = pwm->timer;
	TIM_period_ms(TIMx, msec); 
}
```

#### void PWM_period_us()

```c
void PWM_period_us(PWM_t *pwm, uint32_t usec){
	TIM_TypeDef *TIMx = pwm->timer;
	TIM_period_us(TIMx, usec);
}
```

#### void PWM_pulsewidth_ms()

```c
void PWM_pulsewidth_ms(PWM_t *pwm, float pulse_width_ms){ 
	int CHn = pwm->ch;
	uint32_t fsys = 0;
	uint32_t psc = pwm->timer->PSC;
	
	// Check System CLK: PLL or HSI
	if((RCC->CFGR & (3<<0)) == 2)      { fsys = 84000; }  // for msec 84MHz/1000
	else if((RCC->CFGR & (3<<0)) == 0) { fsys = 16000; }
	
	float fclk = fsys/(psc+1.0);					// fclk=fsys/(psc+1);
	uint32_t ccval = pulse_width_ms *fclk - 1;
	
	switch(CHn){
		case 1: pwm->timer->CCR1 = ccval; break;
		case 2: pwm->timer->CCR2 = ccval; break;
		case 3: pwm->timer->CCR3 = ccval; break;
		case 4: pwm->timer->CCR4 = ccval; break;
		default: break;
	}
}
void PWM_pulsewidth_us(PWM_t *pwm, float pulse_
```

#### void PWM_pulsewidth_us()

```c
void PWM_pulsewidth_us(PWM_t *pwm, float pulse_width_us){ 
	TIM_TypeDef *TIMx = pwm->timer;
	
	int CHn = pwm->ch;
	uint32_t fsys = 0;
	uint32_t psc  = pwm->timer->PSC;
	
	// Check System CLK: PLL or HSI
	if((RCC->CFGR & (3<<0)) == 2)      { fsys = 84; }
	else if((RCC->CFGR & (3<<0)) == 0) { fsys = 16; }
	
	float fclk     = (float) (fsys/(psc+1.0));			    // fclk = fsys/(psc+1);
	uint32_t ccval = pulse_width_us*fclk-1;   // width_ms *fclk;
	
	switch(CHn){
		case 1: TIMx->CCR1 = ccval; break;
		case 2: TIMx->CCR2 = ccval; break;
		case 3: TIMx->CCR3 = ccval; break;
		case 4: TIMx->CCR4 = ccval; break;
		default: break;
	}
}
```

#### void PWM_duty()

```c
void PWM_duty(PWM_t *pwm, float duty) {                         //duty=0 to 1	
	float ccval = (pwm->timer->ARR+1)*duty-1;    				//(ARR+1)*dutyRatio - 1 		         
	int CHn = pwm->ch;
  	
	switch(CHn){
		case 1: pwm->timer->CCR1 = ccval; break;
		case 2: pwm->timer->CCR2 = ccval; break;
		case 3: pwm->timer->CCR3 = ccval; break;
		case 4: pwm->timer->CCR4 = ccval; break;
		default: break;
	}
}
```

#### void PWM_pinmap()

```c
// DO NOT MODIFY HERE
void PWM_pinmap(PWM_t *pwm){
   GPIO_TypeDef *port = pwm->port;
   int pin = pwm->pin;
   
   if(port == GPIOA) {
      switch(pin){
         case 0 : pwm->timer = TIM2; pwm->ch = 1; break;
         case 1 : pwm->timer = TIM2; pwm->ch = 2; break;
         case 5 : pwm->timer = TIM2; pwm->ch = 1; break;
         case 6 : pwm->timer = TIM3; pwm->ch = 1; break;
         //case 7: PWM_pin->timer = TIM1; PWM_pin->ch = 1N; break;
         case 8 : pwm->timer = TIM1; pwm->ch = 1; break;
         case 9 : pwm->timer = TIM1; pwm->ch = 2; break;
         case 10: pwm->timer = TIM1; pwm->ch = 3; break;
         case 15: pwm->timer = TIM2; pwm->ch = 1; break;
         default: break;
      }         
   }
   else if(port == GPIOB) {
      switch(pin){
         //case 0: PWM_pin->timer = TIM1; PWM_pin->ch = 2N; break;
         //case 1: PWM_pin->timer = TIM1; PWM_pin->ch = 3N; break;
         case 3 : pwm->timer = TIM2; pwm->ch = 2; break;
         case 4 : pwm->timer = TIM3; pwm->ch = 1; break;
         case 5 : pwm->timer = TIM3; pwm->ch = 2; break;
         case 6 : pwm->timer = TIM4; pwm->ch = 1; break;
         case 7 : pwm->timer = TIM4; pwm->ch = 2; break;
         case 8 : pwm->timer = TIM4; pwm->ch = 3; break;
         case 9 : pwm->timer = TIM4; pwm->ch = 4; break;
         case 10: pwm->timer = TIM2; pwm->ch = 3; break;
         default: break;
      }
   }
   else if(port == GPIOC) {
      switch(pin){
         case 6 : pwm->timer = TIM3; pwm->ch = 1; break;
         case 7 : pwm->timer = TIM3; pwm->ch = 2; break;
         case 8 : pwm->timer = TIM3; pwm->ch = 3; break;
         case 9 : pwm->timer = TIM3; pwm->ch = 4; break;
         
         default: break;
      }
   }
	 // TIM5 needs to be added, if used.
}
```

#### int update_dir()

```c
int update_dir(int dir, uint8_t idx){
	if(idx%19 == 0) dir *=-1;
	return dir;
}
```

## **ecStepper.c**

#### structure define / state define

```c
//State number 
#define S0 0
#define S1 1
#define S2 2
#define S3 3
#define S4 4
#define S5 5
#define S6 6
#define S7 7

// Stepper Motor function
uint32_t step_per_rev = 64*32;
uint32_t step_delay = 60000/(2048*2);
	 
// Stepper Motor variable
volatile Stepper_t myStepper; 

//FULL stepping sequence  - FSM
typedef struct {
	uint8_t out;
  uint32_t next[2];
} State_full_t;

State_full_t FSM_full[4] = {  
 {0b1100,{S1,S3}},
 {0b0110,{S2,S0}},
 {0b0011,{S3,S1}},
 {0b1001,{S0,S2}}
};

//HALF stepping sequence
typedef struct {
	uint8_t out;
  uint32_t next[2];
} State_half_t;

State_half_t FSM_half[8] = { 
 {0b1000,{S1,S7}},
 {0b1100,{S2,S0}},
 {0b0100,{S3,S1}},
 {0b0110,{S4,S2}},
 {0b0010,{S5,S3}},
 {0b0011,{S6,S4}},
 {0b0001,{S7,S5}},
 {0b1001,{S0,S6}},
};

```

#### void Stepper_init()

```c
void Stepper_init(GPIO_TypeDef* port1, int pin1, GPIO_TypeDef* port2, int pin2, GPIO_TypeDef* port3, int pin3, GPIO_TypeDef* port4, int pin4){
	 
//  GPIO Digital Out Initiation 
	 
	 //port1 = PB10
	 myStepper.port1 = port1;
   myStepper.pin1  = pin1;
	 //port2 = PB4
	 myStepper.port2 = port2;
   myStepper.pin2  = pin2;
	 //port3 = PB5
	 myStepper.port3 = port3;
   myStepper.pin3  = pin3;
	 //port4 = PB3
	 myStepper.port4 = port4;
   myStepper.pin4  = pin4;
	
	
//  GPIO Digital Out Initiation
		// No pull-up Pull-down , Push-Pull, Fast	
		// Port1,Pin1 ~ Port4,Pin4
		GPIO_init(GPIOB, PB10, OUTPUT);
		GPIO_otype(GPIOB,PB10, 0UL);       // 0:Push-Pull
		GPIO_pupd(GPIOB,PB10, 0UL);        // 00: no Pull-up/ pull-down 
		GPIO_ospeed(GPIOB,PB10, 2UL);      // 01:Fast Speed
		
		GPIO_init(GPIOB, PB4, OUTPUT);
		GPIO_otype(GPIOB,PB4, 0UL);       // 0:Push-Pull
		GPIO_pupd(GPIOB,PB4, 0UL);        // 00: no Pull-up/ pull-down 
		GPIO_ospeed(GPIOB,PB4, 2UL);      // 01:Fast Speed
		
		GPIO_init(GPIOB, PB5, OUTPUT);
		GPIO_otype(GPIOB,PB5, 0UL);       // 0:Push-Pull
		GPIO_pupd(GPIOB,PB5, 0UL);        // 00: no Pull-up/ pull-down 
		GPIO_ospeed(GPIOB,PB5, 2UL);      // 01:Fast Speed
		
		GPIO_init(GPIOB, PB3, OUTPUT);
		GPIO_otype(GPIOB,PB3, 0UL);       // 0:Push-Pull
		GPIO_pupd(GPIOB,PB3, 0UL);        // 00: no Pull-up/ pull-down 
		GPIO_ospeed(GPIOB,PB3, 2UL);      // 01:Fast Speed
	
}
```

#### void Stepper_pinOut ()

```c
void Stepper_pinOut (uint32_t state, int mode){
	   if (mode == FULL){         // FULL mode
			 GPIO_write(myStepper.port1, myStepper.pin1, ( (FSM_full[state].out)>>3)&1 );
  		 GPIO_write(myStepper.port2, myStepper.pin2, ( (FSM_full[state].out)>>2)&1 );
			 GPIO_write(myStepper.port3, myStepper.pin3, ( (FSM_full[state].out)>>1)&1 );
			 GPIO_write(myStepper.port4, myStepper.pin4, ( (FSM_full[state].out)>>0)&1 );
			}	 
		 else if (mode == HALF){    // HALF mode
			 GPIO_write(myStepper.port1, myStepper.pin1, ( (FSM_half[state].out)>>3)&1 );
  		 GPIO_write(myStepper.port2, myStepper.pin2, ( (FSM_half[state].out)>>2)&1 );
			 GPIO_write(myStepper.port3, myStepper.pin3, ( (FSM_half[state].out)>>1)&1 );
			 GPIO_write(myStepper.port4, myStepper.pin4, ( (FSM_half[state].out)>>0)&1 );
			}
}
```

#### void Stepper_setSpeed ()

```c
void Stepper_setSpeed (long whatSpeed){      // rppm
		uint32_t step_delay = 	60000/(2048*whatSpeed); 
}
```

#### void Stepper_step()

```c
void Stepper_step(int steps, int direction, int mode){
	 uint32_t state = 0;
	 
	myStepper._step_num = steps;
	
	 for(; myStepper._step_num > 0; myStepper._step_num--){ // run for step size
				                                		// delay (step_delay); 
		    if (mode == FULL){	
					  delay_ms(step_delay); 
						state = FSM_full[state].next[direction];// YOUR CODE       // state = next state
				}
				else if (mode == HALF){
						delay_ms(step_delay/2);
						state = FSM_half[state].next[direction];// YOUR CODE       // state = next state
				}
				Stepper_pinOut(state, mode);
   }
}
```

#### void Stepper_stop ()

```c
void Stepper_stop (void){ 
      // All pins(Port1~4, Pin1~4) set as DigitalOut '0'
    	myStepper._step_num = 0;    
		GPIO_write(myStepper.port1, myStepper.pin1, myStepper._step_num);
		GPIO_write(myStepper.port2, myStepper.pin2, myStepper._step_num);
		GPIO_write(myStepper.port3, myStepper.pin3, myStepper._step_num);
		GPIO_write(myStepper.port4, myStepper.pin4, myStepper._step_num);
}
```

