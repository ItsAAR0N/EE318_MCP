// C file for motor driver

/********************************

motor.c

Motor driver 

Aaron Shek, @ 2023 University of Strathclyde

*********************************/

#include "motor.h"
#include <msp430.h>

#include <driverlib.h>

// Define pins
#define STEP_PIN BIT7 // 1000000
#define DIR_PIN BIT6 // 0100000
#define LED_PIN BIT0 // 0000000

void initialiseGPIO()
{
  // Configure GPIOs
  
  // P1DIR |= 0x80;
  // P1OUT |= 0x80;
  
  P1DIR |= 0x40;
  P1OUT &= ~0x40;
  
  // Configure STEP indicator led
  P4DIR |= 0x01;
  
  // Configure MS1 uStepping resolution LED1/5 and MSn pins
  P8DIR |= 0x01; 
  P2DIR |= 0x80; 

  P5DIR |= 0x01; // MS1
  
  GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P1,GPIO_PIN3); // Pull up config SW1
  GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P1,GPIO_PIN4);
  GPIO_enableInterrupt(GPIO_PORT_P1, GPIO_PIN3);
  GPIO_enableInterrupt(GPIO_PORT_P1, GPIO_PIN4);
}

void initialiseTimer()
{
  // Configure Timer_A
  TA0CCR0 = 3000;
  TA0CCR1 = 1000;
  TA0CCTL1 = OUTMOD_7;
  TA0CTL = TASSEL_2 | MC_1 | TACLR; 
  
  P1DIR |= 0x80; // P1.7 output
  P1SEL0 |= 0x80; // P1.7 options select

}

void initialiseADCpot()
{
    // ADC setup
    // Configure the Pin
    SYSCFG2 |= ADCPCTL9; // Turn on analogue pin A9 (so it is not a GPIO).
    // Configure CLock source, operation mode
    ADCCTL0 |= ADCSHT_2 | ADCON;  // 16 ADCCLK cycles, turn on ADC.
    ADCCTL1 |= ADCSHP; // ADDCLK = MODOSC; sampling timer
    ADCCTL2 |= ADCRES; // 10 bit conversion results
    // Configure ADC mux and +ve & -ve references
    ADCMCTL0 |= ADCINCH_9; // A9 ADC input select; Vref = AVCC   
    ADCIFG &= ~0x01;  //Clear interrupt flag  
    ADCIE |= ADCIE0;  //Enable ADC conversion complete interrupt
}


