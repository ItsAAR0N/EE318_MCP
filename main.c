// Main c file 

/********************************
main.c
Default Template file for testing purposes
Aaron Shek, @ 2023 University of Strathclyde
*********************************/

#include <msp430.h>
#include <driverlib.h>
#include <stdio.h>
#include <math.h>
#include "motor.h"
#include "transltr.h"

#define STEP_PIN BIT7
#define DIR_PIN BIT6

#define STEP_PIN_2 BIT4
#define DIR_PIN_2 BIT3

#define LED_PIN BIT0

#define M1 1
#define M2 2

#define BUFFER_SIZE 64 // Editable String length

unsigned char SW1_interruptFlag_ = 0;
unsigned char SW2_interruptFlag_ = 0;

char buffer[BUFFER_SIZE];
volatile uint8_t bufferIndex = 0;

unsigned int dir = 1;
float dist_1, ang_1, dist_2, ang_2;

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

// UART interrupt 
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=USCI_A0_VECTOR
__interrupt void USCI_A0_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(USCI_A0_VECTOR))) USCI_A0_ISR (void)
#else
#error Compiler not supported!
#endif
{
  switch(__even_in_range(UCA0IV, USCI_UART_UCTXCPTIFG))
  {
    case USCI_NONE: break;
    case USCI_UART_UCRXIFG:
      while(!(UCA0IFG&UCTXIFG));
      UCA0TXBUF = UCA0RXBUF;
      buffer[bufferIndex] = UCA0RXBUF; // Receive char from RX
      bufferIndex++;
      if(bufferIndex >= BUFFER_SIZE || buffer[bufferIndex - 1] == '\n') {
        buffer[bufferIndex] = '\0'; // Null-terminate the string
        printf("Received: %s\n", buffer);
        bufferIndex = 0; // Reset buffer for next incoming message    
      }
      __no_operation();
      break;
    case USCI_UART_UCTXIFG: break;
    case USCI_UART_UCSTTIFG: break;
    case USCI_UART_UCTXCPTIFG: break;
  }
}


void main(void)
{ 
  WDTCTL = WDTPW | WDTHOLD;
  // PM5CTL0 &= ~LOCKLPM5;
  // Disable the GPIO power-on default high-impedance mode
  // to activate previously configured port settings 
  initialiseGPIOs_();
  initialiseTimer_();
  initialiseADCpot_();
   
  initUART_();
  initClockTo8MHz_();

 
  InvKVals result1 = calc_invK(50.0, 20.0);
  // printf("D1: %f\nAng1: %f\nD2: %f\nAng2: %f\n", result1.dist_1, result1.ang_1, result1.dist_2, result1.ang_2);
  // stepMotor_(result1.ang_1/1.8,true,M1);
  // stepMotor_(result1.ang_1/1.8,true,M1); 
  InvKVals result2 = calc_invK(50.0, 20.0);
  // printf("D1: %f\nAng1: %f\nD2: %f\nAng2: %f\n", result2.dist_1, result2.ang_1, result2.dist_2, result2.ang_2);
  PM5CTL0 &= ~LOCKLPM5; PMM_unlockLPM5();
  
  __enable_interrupt();  
  while(true) { 
    // ADCCTL0 |= 0x03; // Sampling and conversion start
    // GCode_Interpret_("X30 Y20 Z200");
    if ((GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN2) == false) || (GPIO_getInputPinValue(GPIO_PORT_P2, GPIO_PIN6) == false)) {
      CB2buttonAdjust_m_(SW1_interruptFlag_, SW2_interruptFlag_);
    }
    // 200 => 360 degrees, 100 => 180 degrees, 50 => 90 degrees, 25 => 45 degrees, ...
    // True => Clockwise, False => anticlockwise   
    // Control examples 
    stepMotor_(200,true,M1); // 90 degrees clockwise for motor 1
    delay_us(150000);
    stepMotor_(200,false,M2); // 90 degrees anticlockwise
    delay_us(300000);
  } 
  // Indicator LED
  P4OUT ^= LED_PIN;
}
