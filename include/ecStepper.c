#include "stm32f4xx.h"
#include "ecStepper.h"

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


void Stepper_setSpeed (long whatSpeed){      // rppm
		uint32_t step_delay = 	60000/(2048*whatSpeed); 
	
}


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


void Stepper_stop (void){ 
     // All pins(Port1~4, Pin1~4) set as DigitalOut '0'
    	myStepper._step_num = 0;    
			GPIO_write(myStepper.port1, myStepper.pin1, myStepper._step_num);
			GPIO_write(myStepper.port2, myStepper.pin2, myStepper._step_num);
			GPIO_write(myStepper.port3, myStepper.pin3, myStepper._step_num);
			GPIO_write(myStepper.port4, myStepper.pin4, myStepper._step_num);
}

