# **LAB: GPIO Digital InOut 7-segment**

**Date:** 2022-09-26

**Author/Partner:** Seungho-Ga / Dabin-Heo

**Github:**  [Github repository](https://github.com/SeunghoG/EC-shga-001)

**Demo Video:** [youtube_link](https://youtu.be/Z5v7XVcmdpY)





## Introduction

In this lab, you are required to create a simple program to control a 7-segment display to show a decimal number (0~9) that increases by pressing a push-button.                                                                                         



## Requirement

### Hardware

- MCU
  - NUCLEO-F401RE
- Actuator/Sensor/Others:
  - 7-segment display(5101ASR)
  - Array resistor (330 ohm)
  - breadboard

### Software

- Keil uVision, CMSIS, EC_HAL library

Problem 1: Connecting 7-Segment

### Procedure

Review 7-segment Decoder and Display from Digital Logic lecture.

- Read here: [7-segment BCD tutorial](https://www.electronics-tutorials.ws/combination/comb_6.html)

The popular BCD 7-segment decoder chips are **74LS47 and CD4511**.

Instead of using the decoder chip, we are going to make the 7-segment decoder with the MCU programming.

![img](https://user-images.githubusercontent.com/38373000/192133325-a4844100-ab1c-445b-8832-837c8f988f35.png)



Connect the common anode 7-segment with the given array resistors.

Apply VCC and GND to the 7-segment display.

Apply 'H' to any 7-segment pin 'a'~'g' and observe if that LED is turned on or off

- Set 'H' on PA8 of MCU and connect to 'a' of the 7-segment.
- Set 'H' on PB10 of MCU and connect to 'b' of the 7-segment.
- Set 'H' on PA7 of MCU and connect to 'c' of the 7-segment.
- Set 'H' on PA6 of MCU and connect to 'd' of the 7-segment.
- Set 'H' on PA5 of MCU and connect to 'e' of the 7-segment.
- Set 'H' on PA9 of MCU and connect to 'f' of the 7-segment.
- Set 'H' on PC7 of MCU and connect to 'g' of the 7-segment.
- Set 'H' on PB6 of MCU and connect to 'dp' of the 7-segment.





### Discussion

1. Draw the truth table for the BCD 7-segment decoder with the 4-bit input.

   ```c
   int number[10][7]={
                      //a b c d e f g//
                       {0,0,0,0,0,0,1},          //zero  0000
                       {1,0,0,1,1,1,1},          //one   0001
                       {0,0,1,0,0,1,0},          //two   0010
                       {0,0,0,0,1,1,0},          //three 0011
                       {1,0,0,1,1,0,0},          //four  0100
                       {0,1,0,0,1,0,0},          //five  0101
                       {0,1,0,0,0,0,0},          //six   0110
                       {0,0,0,1,1,1,1},          //seven 0111
                       {0,0,0,0,0,0,0},          //eight 1000
                       {0,0,0,1,1,0,0},          //nine  1001
                     };
   
   
   ```

2. What are the common cathode and common anode of 7-segment display?

   * common anode type: the common voltage of +VCC is applied to all the diodes.

     0V is given which power-ups the diode.

   * common cathode type: the common voltage of 0V is applied to all the diodes.

     +VCC is given which power-ups the diode.

3. Does the LED of a 7-segment display (common anode) pin turn ON when 'HIGH' is given to the LED pin from the MCU?

   * No, the LED of a 7-segment display is common anode, so we have to give 'LOW' to LED pin from the MCU.



## Problem 2: Display 0~9 with button press

### Procedure

1. Create a new project under the directory `\repos\EC\LAB\LAB_GPIO_7segment`

- The project name is “**LAB_GPIO_7segment”.**
- Create a new source file named as “**LAB_GPIO_7segment.c”**
- Refer to the [sample code](https://github.com/ykkimhgu/EC-student/tree/main/tutorial/tutorial-student)

2. Include your updated library in `\repos\EC\lib\` to your project.

- **ecGPIO.h, ecGPIO.c**
- **ecRCC.h, ecRCC.**



3. Declare and Define the following functions in your library

- You can refer to [an example code of 7-segment control](https://os.mbed.com/users/ShingyoujiPai/code/7SegmentDisplay/file/463ff11d33fa/main.cpp/)

​    **ecGPIO.h**

```c
void sevensegment_init(void);
void sevensegment_decode(int num);
```

​	1) First, check if every number, 0 to 9, can be displayed properly

​	2) Then, create a code to display the number from 0 to 9 with each button press. After the number '9', it should start from '0'     	    again.



### Configuration

![임베핀표](https://user-images.githubusercontent.com/113995124/194503833-06774ce0-b962-47c1-9163-9265b3ce8f4c.PNG)

### Exercise

![KakaoTalk_20221007_172230733](https://user-images.githubusercontent.com/113995124/194508335-7edaf8d1-786d-435c-abcc-aa56358d7a71.jpg)





### Code

 [SourceCode](https://github.com/SeunghoG/EC-shga-001/blob/main/lab/LAB_GPIO_Digital_InOut_7_segment.c)

* We have to initialization each seven segment pin

```c
    sevensegment_init();
	unsigned int cnt = 0;
```

* In Inifinite Loop, the seven segment decode function operate. "For" phrase is artificial time delay

  ```c
  while(1){
  		sevensegment_decode(cnt % 10);
  		if(GPIO_read(GPIOC, BUTTON_PIN) == 0) {
  			cnt++; 
  		}
  		for(int i = 0; i < 500000;i++){}
  	}
  ```

* Setup function

  ```c
  void setup(void)
  {
  	GPIO_init(GPIOC, BUTTON_PIN, INPUT);  // calls RCC_GPIOC_enable()
  	GPIO_pupd(GPIOC ,BUTTON_PIN , 1UL);   // 01: Pull-up
  
  }
  ```

### Results

[demo video link](https://youtu.be/Z5v7XVcmdpY)



## Reference

> https://instrumentationtools.com/difference-between-common-cathode-and-common-anode-7-segment-led-display/#h-common-anode-7-segment-led-display
>
> https://www.geeksforgeeks.org/bcd-to-7-segment-decoder/





## Troubleshooting

* first I made my GPIO write function like this

  ```c
  void GPIO_write(GPIO_TypeDef *Port, int pin,  int output){
  	Port->ODR |=(0UL<<(pin));
  	Port->ODR |=(output << pin);
  }
  ```

  I think masking 0 is as same as clear, but It failed. so I choose clear way.

  ```c
  void GPIO_write(GPIO_TypeDef *Port, int pin,  int output){
  	Port->ODR &= ~(1UL<<(pin));
  	Port->ODR |=(output << pin);
  }
  ```

  



