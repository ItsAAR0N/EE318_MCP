// Main c file 

/********************************
main.c
Default Template file for testing purposes
Aaron Shek, @ 2023 University of Strathclyde
*********************************/

#include <msp430.h>
#include <driverlib.h>

#define STEP_PIN BIT7
#define LED_PIN BIT0
#define DIR_PIN BIT6

void main(void)
{
  WDTCTL = WDTPW + WDTHOLD;
  PM5CTL0 &= ~LOCKLPM5;

  // Configure GPIOs
  P1DIR |= STEP_PIN + DIR_PIN;
  P1OUT &= ~STEP_PIN;
  P1OUT |= DIR_PIN;
  P4DIR |= LED_PIN;

  // Configure Timer_A
  TA0CCR0 = 1000;
  TA0CCR1 = 500;
  TA0CCTL1 = OUTMOD_7;
  TA0CTL = TASSEL_2 + MC_1 + TACLR;

  // Rotate continuously in full stepping
  while (1) {
    P1OUT ^= STEP_PIN;
    P4OUT ^= LED_PIN;
    __delay_cycles(1000);
  }
}
