// C file for motor driver

/********************************
motor.c
Motor driver 
Aaron Shek, @ 2023 University of Strathclyde
*********************************/
#include <msp430.h>
#include <driverlib.h>
#include <stdio.h>
#include <math.h>
#include "motor.h"

// Define pins
#define STEP_PIN BIT7 // 1000000
#define DIR_PIN BIT6 // 0100000
#define LED_PIN BIT0 // 0000000
#define STEP_PIN_2 BIT4 // 0001000
#define DIR_PIN_2 BIT3 // 0000100

const int accel_Val = 4000;
unsigned long difference = 0;

void initialiseGPIOs_()
{
  // Configure Stepper control pins to A4988 interface  
  P1DIR |= STEP_PIN | DIR_PIN | STEP_PIN_2 | DIR_PIN_2;
  P1OUT &= ~STEP_PIN; 
  P1OUT &= ~STEP_PIN_2;
  //P1OUT |= DIR_PIN;
  GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN6); // Clockwise M1
  GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN3);
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

void readValue_pot_(int rate_)
{
  int currentVal = rate_;
}

void delay_us(unsigned long delay) 
{ //  I have no idea if this is us or ms lol
  while (delay--) {
    __delay_cycles(1);
  }
} 

void stepMotor_(int n_Steps, bool dir, int motor_Num)
{ 
  int i = 0;
  int DIRpin, STEPpin;
  
  float time_del = 0.003; // time delay
  int rounded_time_del = 0;
  
  if (motor_Num == 1) {
    DIRpin = GPIO_PIN6; // Stepper 1 DIR pin
    STEPpin = GPIO_PIN7; // Stepper 1 STEP pin
  } else if (motor_Num == 2) {
    DIRpin = GPIO_PIN3; // Stepper 2 DIR pin
    STEPpin = GPIO_PIN4; // Stepper 2 STEP pin
  } else {
    return; 
  }
  
  while(i < n_Steps) {
    if (i < n_Steps/2) { // One half accel, the other half decel
      time_del = pos_Accel_(time_del); // Take in initial time delay 
    } else {
      time_del = neg_Accel_(time_del); 
    }
    rounded_time_del = round(time_del*500000); 
    if (dir == true) {
        GPIO_setOutputHighOnPin(GPIO_PORT_P1, DIRpin); // Set anti-clockwise direction
    } else {
        GPIO_setOutputLowOnPin(GPIO_PORT_P1, DIRpin); // Set false-clockwise direction
    }
    GPIO_setOutputHighOnPin(GPIO_PORT_P1, STEPpin);
    delay_us(rounded_time_del);
    GPIO_setOutputLowOnPin(GPIO_PORT_P1, STEPpin);
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

float pos_Accel_(float time_del) { // Previous time delay input argument
  float dVelocity = time_del * accel_Val;
  time_del = 1/(dVelocity + 1/time_del); // td2 = ... 1/td1
  if (time_del < 0.00025) {
    time_del = 0.00025; // Minimum time delay 
  }
  return time_del; // Return new next time delay
}

float neg_Accel_(float time_del) {
  float dVelocity = time_del * -1 * accel_Val;
  time_del = 1/(dVelocity + 1/time_del);
  if (time_del > 0.003) {
    time_del = 0.003;
  }
  return time_del;
}