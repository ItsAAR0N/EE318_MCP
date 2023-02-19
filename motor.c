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

void initialiseGPIOs_()
{
  // Configure Stepper control pins to A4988 interface  
  P1DIR |= STEP_PIN | DIR_PIN;
  P1OUT &= ~STEP_PIN; 
  //P1OUT |= DIR_PIN;
  GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN6);    
  // Indiactor LEDs
  P4DIR |= LED_PIN; 
  // Set buttons using std. lib.
  GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P1,GPIO_PIN2); // Pull up config SW1
  GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P2,GPIO_PIN6);  
  GPIO_enableInterrupt(GPIO_PORT_P1, GPIO_PIN2);
  GPIO_enableInterrupt(GPIO_PORT_P2, GPIO_PIN6); 
  // Configure MS1 uStepping resolution LED1/5 and MSn pins
  P8DIR |= 0x01; 
  P2DIR |= 0x80; 
}

void initialiseTimer_()
{
  // Configure Timer_A
  TA0CCR0 = 3000;
  TA0CCR1 = 1000;
  TA0CCTL1 = OUTMOD_7;
  TA0CTL = TASSEL_2 | MC_1 | TACLR; 
  P1DIR |= 0x80; // P1.7 output
  P1SEL0 |= 0x80; // P1.7 options select

}

void initialiseADCpot_()
{
  // ADC setup
    // Configure Pin for ADC
  GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P8,GPIO_PIN1,GPIO_PRIMARY_MODULE_FUNCTION);
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

void readValue_pot_()
{
  // Calibration procedure

}

void stepMotor_(int n_Steps, char dir)
{ 
  int i = 0;
  while(i < n_Steps) {
  if (dir == true) {
      // P1OUT |= DIR_PIN; // Set direction pin
      GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN6); // Set clockwise direction
  } else {
      //P1OUT &= ~DIR_PIN; // Set direction pin
      GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN6); // Set anti-clockwise direction
  }
  GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN7);
  __delay_cycles(1000);
  GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN7);
  i++;
  }
}

void CB2buttonAdjust_m_(int SW1_interruptFlag, int SW2_interruptFlag)
{ 
  // Calibration procedure
  unsigned char lastPinState_L = 1;
  unsigned char lastPinState_R = 1;
  unsigned char pinState_L, pinState_R;

  // Check button state
  pinState_L = GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN2);
  pinState_R = GPIO_getInputPinValue(GPIO_PORT_P2, GPIO_PIN6);
        
  if (SW1_interruptFlag && lastPinState_L) {
    // Wait for the button to be pressed (WIP. FOR uS' control)
    while (!pinState_L) {
      GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN7);
      __delay_cycles(100000); // More precise movement
      GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN7);   
      GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN6); // Set clockwise direction
      pinState_L = GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN2); // Check button value continuously    
    }
    SW1_interruptFlag = 0; // Clear SW1 flag
    __delay_cycles(500000); // Delay 0.5 second
  }
  lastPinState_L = pinState_L; // Update last pin state

  if (SW2_interruptFlag && lastPinState_R) {
    // Wait for the button to be pressed
    while (!pinState_R) {
      GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN7);
      __delay_cycles(100000);
      GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN7);   
      GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN6); // Set anti-clockwise direction
      pinState_R = GPIO_getInputPinValue(GPIO_PORT_P2, GPIO_PIN6); // Check button value continuously    
    }
    SW2_interruptFlag = 0; // Clear SW2 flag
    __delay_cycles(500000); // Delay 1 second
  }
  lastPinState_R = pinState_R; // Update last pin state 
}
