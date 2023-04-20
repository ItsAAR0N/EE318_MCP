// Main c file 

/********************************
main.c
Main project file 
Aaron Shek, @ 2023 University of Strathclyde
*********************************/

#include <msp430.h>
#include <driverlib.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
#include "motor.h"
#include "transltr.h"

#define LED_PIN BIT0

// ---- Motor Defintions ----
#define M1 1
#define M2 2
#define RADTODEG 57.2958        // 1 Rad = 57.3 degrees
#define STEPSPERDEG 200/360    // 1.8 deg/step
#define STEP_PIN BIT7           // Motor 1
#define DIR_PIN BIT6
#define STEP_PIN_2 BIT4         // Motor 2
#define DIR_PIN_2 BIT3

unsigned int dir = 1;
float dist_1, ang_1, dist_2, ang_2;

// ---- UART Definitions ---- 
#define BAUDRATE 115200
#define BUFFER_SIZE 128                 // Editable String size
char buffer[BUFFER_SIZE];
volatile uint8_t bufferIndex = 0;

// ---- Adjust Interrupts ---- 
unsigned char SW1_interruptFlag_ = 0;
unsigned char SW2_interruptFlag_ = 0;

#pragma vector = PORT1_VECTOR           // Interrupt handler
__interrupt void P1_ISR(void)
{
  switch(__even_in_range(P1IV,P1IV_P1IFG7)) 
  {
  case P1IV_P1IFG2:                     // SW1
    SW1_interruptFlag_ = 1;
    GPIO_clearInterrupt(GPIO_PORT_P1, GPIO_PIN2);
    break;  
  }    
}

#pragma vector = PORT2_VECTOR           // Interrupt handler
__interrupt void P2_ISR(void)
{
  switch(__even_in_range(P2IV,P2IV_P2IFG7)) 
  {        
  case P2IV_P2IFG6:                     // SW2
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
      buffer[bufferIndex] = UCA0RXBUF;          // Receive char from RX
      bufferIndex++;
      if(bufferIndex >= BUFFER_SIZE || buffer[bufferIndex - 1] == '\n') {
        processUARTinstr_(buffer);
        buffer[bufferIndex] = '\0';             // Null-terminate the string
        //printf("Received: %s\n", buffer);
        bufferIndex = 0;                        // Reset buffer for next incoming message  
        memset(buffer, 0, sizeof(buffer));      // Clear the buffer to an empty string
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
  PM5CTL0 &= ~LOCKLPM5;
  PMM_unlockLPM5();
  // Disable the GPIO power-on default high-impedance mode
  // to activate previously configured port settings 
  
  // ---- Initialise GPIOS, Timer Module, ADC module ---- 
  initialiseGPIOs_();
  initialiseTimer_();
  initialiseADCpot_();
  initUART_();
  initClockTo8MHz_();
  
  // ---- Calculate motor steps to reach (0,0) initially
  calc_invK(0, 0); 

  __enable_interrupt(); 
  delay_us(6000000);
  // ---- Test scenarios
  target();

  while(true) { 
    if ((GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN2) == false) || (GPIO_getInputPinValue(GPIO_PORT_P2, GPIO_PIN6) == false)) {
      CB2buttonAdjust_m_(SW1_interruptFlag_, SW2_interruptFlag_);
    }
    ADCCTL0 |= 0x83;                    // ADCENC (ADC enable conversion), ADCSC Start sample and conversion, ADCMSC multiply sample-and-conversion

    // 200 => 360 degrees, 100 => 180 degrees, 50 => 90 degrees, 25 => 45 degrees, ...
    // True => Clockwise, False => anticlockwise
    
    // ---- Control examples ----
    // penManualControl_();
  } 
  // Indicator LED
  P4OUT ^= LED_PIN;
}
