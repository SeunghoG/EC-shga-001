# LAB 1 Smart mini-fan with STM32-duino

###### Date : 09.20.2022

###### Author/Partner: SeunghoGa / hyunwooNam

###### Git hub: [repository link](https://github.com/SeunghoG?tab=repositories)

###### Demo Video: [Youtube link](https://youtu.be/j7jIt_eTXVs)

## Introduction

In this lab, you are required to create a simple program that uses arduino IDE for implementing a simple embedded digital application. 



## Requirement

### Hardware

* MCU
  * NUCLEO-F401RE
* Sensor:
  * Ultrasonic distance sensor(HC-SR04) x1
  * and others

### Software

* Arduino IDE



## Problem



### Procedure

The program needs to run the Fan only when the distance of an object is within a certain value.

Example: An automatic mini-fan that runs only when the face is near the fan. Otherwise turns off.



* As the button **B1** is pressed, change the fan velocity. The MODE(states) are

  MODE(state): **OFF(0%), MID(50%), HIGH(100%)** 

* When the object(face) is detected about 5 cm away, then it automatically pauses the fan temporarily.

  * Even the fan is temporarily paused, the MODE should be changed whenever the button **B1** is pressed

* When the object(face) is detected within 50mm, then it automatically runs the fan.

  It must run at the speed of the current MODE.

* LED(**LED1**): Turned OFF when MODE=OFF. Otherwise, blink the LED with 1 sec period (1s ON, 1s OFF)

  

  

### Configuration



##### Ultrasonic distance sensor

Trigger:

* Generate a trigger pulse as PWM to the sensor
* Pin: **D10** (TIM4 CH1)
* PWM out: 50ms period, 10us pulse-width

Echo:

* Receive echo pulses from the ultrasonic sensor
* Pin: **D7** (Timer 1 CH1)
* Input Capture: Input mode
* Measure the distance by calculating pulse-width of the echo pulse.



##### USART

* Display measured distance in [cm] on serial monitor of Tera-Term.
* Baud rate 9600

##### DC Motor

* PWM: PWM1, set 10ms of period by default
* Pin: **D11** (Timer1 CH1N)



### Circuit/Wiring Diagram

External circuit diagram that connects MCU pins to peripherals(sensor/actuator)

![circuit](https://user-images.githubusercontent.com/113995124/191495162-9ba16f00-2639-4587-9de1-1cf9dcda97f9.png)



## Algorithm



### Overview

Flowchart or FSM table/graph goes here

###### Mealy FSM Table

![KakaoTalk_20220920_215258868 - 복사본](https://user-images.githubusercontent.com/113995124/191263246-1ded6ee4-3050-4aeb-a602-aba6f944fcc1.jpg)



![mealy_state_diagram](https://user-images.githubusercontent.com/113995124/191494917-59a6b9ba-69e9-44ac-8568-52f995506698.png)







### Description with Code

* Lab source code:  [Mini-Fan source code](https://github.com/SeunghoG/EC/blob/main/arduino/LAB:%20Smart%20mini-fan%20with%20STM32-duino.c)

1. Structure definition

   ```c
   typedef struct {
     uint32_t out[2][2][2];     // output = FSM[state].out[input[0]][input[1]].[PWM or LED]
     uint32_t next[2][2];       // nextstate = FSM[state].next[input]
   } State_t;
   ```

2.  Bring the mealy state table to Arduino code

   ```c
   State_t FSM[3] = {
     { { {{0,LOW}, {0,LOW}}, {{0,HIGH}, {80,HIGH}} } , { {s0, s0}, {s1, s1} } },
     { { {{0,HIGH}, {80,HIGH}},{{0,HIGH},{160,HIGH}} }, { {s1, s1}, {s2, s2} } },
     { { {{0,HIGH}, {160,HIGH}},{{0,LOW},{0,LOW}} }, { {s2, s2}, {s0, s0} } }
   };
   
   ```

   We choose state then chosen by input[0] and input[1]. 

3. Make sensor's detected length condition

   ``` c
   if (distance < thresh)
       input[1] = 1;
     else
       input[1] = 0;
   ```

4. State function

   ```c
   void nextState(){
     // [TO-DO] YOUR CODE GOES HERE
     pwmOut = FSM[state].out[input[0]][input[1]][PWM]; // find pwm value by designated mealy table.
     ledOut= FSM[state].out[input[0]][input[1]][LED];  // find led value by designated mealy table.
     
     nextstate = FSM[state].next[input[0]][input[1]];  // updated state.
     state = nextstate;
   
     input[0]=0;                                       // reset button value.
     
   }
   ```

5. Button Pressed function

   ``` c
   void pressed(){
     input[0] = 1;     
   }
   ```

   If button pressed input[0] becomes 1. (interrupt)

6. Blink Method

   ``` c
   if( (millis()/1000)%2 == 0)        
     digitalWrite(ledPin, LOW);
     else
     digitalWrite(ledPin, ledOut);
     
     analogWrite(pwmPin, pwmOut);
   
   ```

   As Doyeon YOO's idea

   > *if system time(s) is even number ,put in LOW signal then odd numbers proceed normally. By  forcing LOW, we can make it blink*...



## Results and Analysis

First, In demo video we can see the fan works when distance < 5cm and 50%pwm mode. 

Second, when button pressed 50%mode to 100%mode and it operate distance < 5cm.

last, we can see the delay operating point to serial print in Tera Term. if we reduce delay() time

,get higher response speed.









