// Main c file 

/********************************
main.c
Default Template file for testing purposes
Aaron Shek, @ 2023 University of Strathclyde
*********************************/

#include <msp430.h>
#include <driverlib.h>
#include <stdio.h>
#include "motor.h"

#define STEP_PIN BIT7
#define LED_PIN BIT0
#define DIR_PIN BIT6

unsigned char SW1_interruptFlag_ = 0;
unsigned char SW2_interruptFlag_ = 0;
unsigned int rate_ = 1;
unsigned int dir = 1;

#pragma vector = PORT1_VECTOR // Interrupt handler
__interrupt void P1_ISR(void)
{
  switch(__even_in_range(P1IV,P1IV_P1IFG7)) 
  {
  case P1IV_P1IFG2: // SW1
    SW1_interruptFlag_ = 1;
    GPIO_clearInterrupt(GPIO_PORT_P1, GPIO_PIN2);
    break;  
  }    
}

#pragma vector = PORT2_VECTOR // Interrupt handler
__interrupt void P2_ISR(void)
{
  switch(__even_in_range(P2IV,P2IV_P2IFG7)) 
  {        
  case P2IV_P2IFG6:  // SW2
    SW2_interruptFlag_ = 1;
    GPIO_clearInterrupt(GPIO_PORT_P2, GPIO_PIN6);
    break;
  }    
}

// ADC interrupt service routine
#pragma vector=ADC_VECTOR           
__interrupt void ADC_ISR(void)
{
  switch(__even_in_range(ADCIV,ADCIV_ADCIFG))
  {
    case ADCIV_ADCIFG:              // we're ready for a reading
    rate_ = (int)(0 + (2*ADCMEM0)); // Max value of 2695, min 600
    //printf("%d\n",rate_);
    __bic_SR_register_on_exit(LPM0_bits); // Clear CPUOFF bit from LMP0 to prevent MCU sleeping
    //ADC_clearInterrupt(ADC_BASE,ADC_COMPLETED_INTERRUPT);
    break;
  }
}

void main(void)
{
  WDTCTL = WDTPW | WDTHOLD;
  // PM5CTL0 &= ~LOCKLPM5;
  // Disable the GPIO power-on default high-impedance mode
  // to activate previously configured port settings
  PMM_unlockLPM5();  
  
  initialiseGPIOs_();
  initialiseADCpot_();
  __enable_interrupt();
  
  // Rotate continuously in full stepping
  while(true) { 
    ADCCTL0 |= 0x03; // Sampling and conversion start
    if ((GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN2) == false) || (GPIO_getInputPinValue(GPIO_PORT_P2, GPIO_PIN6) == false)) {
      CB2buttonAdjust_m_(SW1_interruptFlag_, SW2_interruptFlag_);
    }
    // 200 => 360 degrees, 100 => 180 degrees, 50 => 90 degrees, 25 => 45 degrees, ...
    // True => Clockwise, False => anticlockwise
    // Control examples
    if (rate_ < 2048) {
      dir = 1; // Clockwise
    } else {
      dir = -1; // Anti-clockwise
    }
    //stepMotor_(dir);
    
    __delay_cycles(3000000);
    stepMotor_(100,true); // 180 degrees
    __delay_cycles(3000000);
    stepMotor_(50,true); // 90 degrees
    __delay_cycles(3000000);
    stepMotor_(25,true); // 45 degrees
    
  } 
  // Indicator LED
  P4OUT ^= LED_PIN;
}
