// PWMtest.c
// Initialize port F: inputs PF4 and PF0 for onboard swicthes (sw1,sw2), and outputs PF3-1 for LEDs (red, green, blue) 
// Initialize port E: outputs on PF3-0 for forward/backward direction
// control motor speed and direction

// CECS 347 Project 1 - Robot Car with Motor Control
// Description: build a wheeled robot that changes its speed and direction according to
// the input switches of the TM4C123G LaunchPad Microcontroller
// Student Name: Len Quach


// Preprocessor Directives
#include <stdint.h>
#include "PLL.h"
#include "PWM.h"
#include "tm4c123gh6pm.h"

// Constants 
#define PERIOD 			25000           	// number of machine cycles for 10ms, value is based on 50MHz system clock: 50MHz/2/1000Mhz = 25kHz
#define LIGHT   GPIO_PORTF_DATA_R
#define RED 		0x02
#define BLUE 		0x04
#define GREEN 	0x08


// Function prototypes
// External functions for interrupt control defined in startup.s
extern void DisableInterrupts(void); // Disable interrupts
extern void EnableInterrupts(void);  // Enable interrupts
extern void WaitForInterrupt(void);  // low power mode

// This function initilizes port F and arm PF4, PF0 for falling edge interrupts and also PF3-0 output for LEDs 
void PortF_Init(void);
void GPIOPortF_Handler(void);

// Initialize PE3-0 for 2 DC Motor Direction
void PortE_Init(void);

// Global variables: 
unsigned long speed;
unsigned long direction;

int main(void){
  DisableInterrupts();  // disable interrupts to allow initializations
  PLL_Init();          // bus clock at 80 MHz
  PortF_Init();        // arm PF4, PF0 for falling edge interrupts 
  PortE_Init(); 
	PWM0A_Init(25000,0);     //PWM Left wheel
	PWM0B_Init(25000,0);     //PWM Right wheel
	EnableInterrupts();   // enable after initializations are done
	
	GPIO_PORTE_DATA_R = 0x05;  // start in forward direction mode: IN1=PE0, IN2=PE1 0000_0101
	
	while(1){
    WaitForInterrupt();
  }	
}

// Subroutine to initialize port F pins for input and output
// PF4 and PF0 are input SW1 and SW2 respectively
// PF3-1 are output to the LED
// Initilize port F and arm PF4, PF0 for falling edge interrupts 
void PortF_Init(void){  
	unsigned long volatile delay;
  SYSCTL_RCGC2_R |= 0x00000020; // (a) activate clock for port F
  delay = SYSCTL_RCGC2_R;       // same as: while((SYSCTL_PRGPIO_R&0x20) == 0){};
  GPIO_PORTF_LOCK_R = 0x4C4F434B; // unlock GPIO Port F
  GPIO_PORTF_CR_R = 0x1F;         // allow changes to PF4-0
  GPIO_PORTF_DIR_R &= ~0x11;    // (c) make PF4,0 in (built-in button)
  GPIO_PORTF_DIR_R |= 0x0E;     //     make PF3,PF2,PF1 output 
	GPIO_PORTF_AFSEL_R &= ~0x1F;  //     disable alt funct on PF4-0
  GPIO_PORTF_DEN_R |= 0x1F;     //     enable digital I/O on PF4-0
  GPIO_PORTF_DATA_R = 0x02;     //     make PF1 high (start with red LED)
	GPIO_PORTF_PCTL_R &= ~0x000FFFFF; //  configure PF4-0 as GPIO
  GPIO_PORTF_AMSEL_R &= ~0x1F;  //     disable analog functionality on PF4-0
  GPIO_PORTF_PUR_R |= 0x11;     //     enable weak pull-up on PF4,0
  GPIO_PORTF_IS_R &= ~0x11;     // (d) PF4,PF0 is edge-sensitive
  GPIO_PORTF_IBE_R &= ~0x11;    //     PF4,PF0 is not both edges
  GPIO_PORTF_IEV_R &= ~0x11;    //     PF4,PF0 falling edge event
  GPIO_PORTF_ICR_R = 0x11;      // (e) clear flags 4,0
  GPIO_PORTF_IM_R |= 0x11;      // (f) arm interrupt on PF4,PF0	
  NVIC_PRI7_R = (NVIC_PRI7_R&0xFF1FFFFF)|0x00400000; // (g) bits:23-21 for PORTF, set priority to 2
  NVIC_EN0_R = 0x40000000;      // (h) enable interrupt 30 in NVIC
}

// Initialize outputs on PE3-0 for 2 DC motor direction
void PortE_Init(void){  
	SYSCTL_RCGCGPIO_R |= 0x10;            // activate port E
		while((SYSCTL_PRGPIO_R&0x10) == 0){};
	GPIO_PORTE_DIR_R |= 0x0F;     //  make PE4-0 output
  GPIO_PORTE_AFSEL_R &= ~0x0F;  //     disable alt funct on PE4-0
  GPIO_PORTE_DEN_R |= 0x0F;     //     enable digital I/O on PE4-0
  GPIO_PORTE_PCTL_R &= ~0x0000FFFF; //  configure PE3-0 as GPIO
  GPIO_PORTE_AMSEL_R &= ~0x0F;  //     disable analog functionality on PE4-0
}


// L range: 2500,5000,7500,10000,12500,15000,17500,20000,22500,24500  
// power:    10%  20%  30%  40%   50%   60%   70%   80%   90%   98%
void GPIOPortF_Handler(void){ // called on touch of either SW1 or SW2
  unsigned long duty;// = PWM0B_GetDuty(); 

  if(GPIO_PORTF_RIS_R&0x10){  // SW1 touch - PF4 speed up
		for (int i=0; i<1000000; i++) // for button debounce used to generate the wait for 10ms
    GPIO_PORTF_ICR_R = 0x10;  // acknowledge flag4
		if (direction == 1) {
				GPIO_PORTE_DATA_R = 0x0A;  // backward  00001010	
				LIGHT = BLUE; 
		}
		else {
				GPIO_PORTE_DATA_R = 0x05;  // forward  00000101	
				LIGHT = GREEN;  
		}
		
		speed += 1;
		if (speed == 0){ //robot starts in no motion
			duty = 0; 
			LIGHT = RED;
		}
		else if (speed == 1) 
			duty = 7500; //30% duty cycle
		else if (speed == 2) 
			duty = 15000; //60% duty cycle
		else if (speed == 3) 
			duty = 20000;	//80% duty cycle
		else if (speed == 4) 
			duty = 24500; //98% duty cycle
		else {
			duty = 0;
			LIGHT = RED;
			speed = 0; // reset speed to 0 for cycle repeat
		}					
		PWM0A_Duty(duty);
		PWM0B_Duty(duty); 
	}	
	
	if(GPIO_PORTF_RIS_R&0x01){  // SW2 touch - PF1 change direction
    for (int i=0; i<1000000; i++) 
		GPIO_PORTF_ICR_R = 0x01;  // acknowledge flag0
		direction += 1;
		if (direction == 1) {
				GPIO_PORTE_DATA_R = 0x0A;  // backward  00001010	
				LIGHT = BLUE; 
		}
		else {
				GPIO_PORTE_DATA_R = 0x05;  // go forward  00000101	
				LIGHT = GREEN;  
				direction = 0; // reset to 0 to go backward		
   }
 }
}



