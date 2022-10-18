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
	SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;                  // 1 = counting down to zero asserts the                                                                            SysTick exception request
	
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