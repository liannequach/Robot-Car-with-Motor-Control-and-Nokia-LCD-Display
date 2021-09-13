// PWM.h
// Use PB6/M0PWM0 and PB7/M0PWM1 to generate pulse-width modulated outputs.

// CECS 347 Project 1 - Robot Car with Motor Control
// Description: build a wheeled robot that changes its speed and direction according to
// the input switches of the TM4C123G LaunchPad Microcontroller
// Student Name: Len Quach


#include <stdint.h>

// period is 16-bit number of PWM clock cycles in one period (3<=period)
// period for PB6 and PB7 must be the same
// duty is number of PWM clock cycles output is high  (2<=duty<=period-1)
// PWM clock rate = processor clock rate/SYSCTL_RCC_PWMDIV
//                = BusClock/2 
//                = 50 MHz/2 = 25 MHz 

// Output on PB6/M0PWM0 
void PWM0A_Init(uint16_t period, uint16_t duty);

// change duty cycle of PB6 
// duty is number of PWM clock cycles output is high  (2<=duty<=period-1)
void PWM0A_Duty(uint16_t duty);


// Output on PB7/M0PWM1 
void PWM0B_Init(uint16_t period, uint16_t duty);

// change duty cycle of PB7 
// duty is number of PWM clock cycles output is high  (2<=duty<=period-1)
void PWM0B_Duty(uint16_t duty);



