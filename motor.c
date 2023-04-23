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
#include "transltr.h"

// ---- Pin configuration  ----
#define STEP_PIN BIT7 // 1000000
#define DIR_PIN BIT6 // 0100000
#define LED_PIN BIT0 // 0000000
#define STEP_PIN_2 BIT4 // 0001000
#define DIR_PIN_2 BIT3 // 0000100

// ---- Motor definition ---- 
#define M1 1
#define M2 2
#define CW 1                            // Motor directions
#define CCW 0

// ---- Acceleraiton values ---- 
const int accel_Val = 4000;
unsigned int rate_ = 1;
long PULSE_WIDTH = 10000;

// ---- ADC interrupt service routine ----     
#pragma vector=ADC_VECTOR               
__interrupt void ADC_ISR(void)    
{    
  switch(__even_in_range(ADCIV,ADCIV_ADCIFG))    
  {    
    case ADCIV_ADCIFG:                  
    rate_ = mapRange_(ADCMEM0,0,1023,1000,32000);    
    //printf("%d\n",rate_);    
    __bic_SR_register_on_exit(LPM0_bits);       // Clear CPUOFF bit from LMP0 to prevent MCU sleeping    
    //ADC_clearInterrupt(ADC_BASE,ADC_COMPLETED_INTERRUPT);    
    break;    
  }    
}

// ---- Timer Configuration ----
uint16_t TimerA1_period = 20000; // PWM Period
unsigned int i;

void initialiseGPIOs_()
{
  // Configure Stepper control pins to A4988 interface  
  P1DIR |= STEP_PIN | DIR_PIN | STEP_PIN_2 | DIR_PIN_2;
  P1OUT &= ~STEP_PIN; 
  P1OUT &= ~STEP_PIN_2;
  //P1OUT |= DIR_PIN;
  GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN6);             // Clockwise M1
  GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN3);
  
  // Indiactor LEDs
  P4DIR |= LED_PIN; 
  
  // Set buttons using std. lib.
  GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P1,GPIO_PIN2); // Pull up config SW1
  GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P2,GPIO_PIN6); 
  GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P2,GPIO_PIN5);
  GPIO_enableInterrupt(GPIO_PORT_P1, GPIO_PIN2);
  GPIO_enableInterrupt(GPIO_PORT_P2, GPIO_PIN6); 
  
  // Configure MS1 uStepping resolution LED1/5 and MSn pins
  P8DIR |= 0x01; 
  P2DIR |= 0x80;
  
  // SG90
  P8DIR |= BIT3;                                                // P8.3 output
  P8SEL0 |= BIT3;                                               // P8.3 options select
  
  // USCI_A0 UART operation
  P1SEL0 |= BIT0 | BIT1;
}

void initialiseTimerPWM_()
{
  // Timer TA1 setup
  TA1CCR0 = TimerA1_period;             // PWM Period
  TA1CCTL2 = OUTMOD_7;                  // CCR1 reset/set                      
  TA1CTL = TASSEL_2 | MC_1 | ID_3 ;             // SMCLK, up mode, Divide by 8 because of 8MHz SMCLK
}

void initialiseADCpot_()	
{	
  // ADC setup	
  // Configure Pin for ADC	
  GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P8,GPIO_PIN1,GPIO_PRIMARY_MODULE_FUNCTION);	
    	
  SYSCFG2 |= ADCPCTL9;          // Turn on analogue pin A9 (so it is not a GPIO).	
  	
  // Configure CLock source, operation mode	
  ADCCTL0 |= ADCSHT_2 | ADCON;  // 16 ADCCLK cycles, turn on ADC.	
  ADCCTL1 |= ADCSHP;            // ADDCLK = MODOSC; sampling timer	
  ADCCTL2 |= ADCRES;            // 10 bit conversion results	
  	
  // Configure ADC mux and +ve & -ve references	
  ADCMCTL0 |= ADCINCH_9;        // A9 ADC input select; Vref = AVCC   	
  ADCIFG &= ~0x01;              //Clear interrupt flag  	
  ADCIE |= ADCIE0;              //Enable ADC conversion complete interrupt	
}

int accel_Val_()
{
  return rate_;
}

void delay_us(unsigned long delay) 
{                                       
  while (delay--) {     
    __delay_cycles(1);
  }
} 

void stepMotor1_(int init_Steps1, int n_Steps1, bool dir1) {
  int i = 0;
  int DIRpin1 = GPIO_PIN6;     // Stepper 1 DIR pin
  int STEPpin1 = GPIO_PIN7;    // Stepper 1 STEP pin
  float time_del = 0.003;      // Time delay
  int rounded_time_del = 0;
  //ADCCTL0 |= 0x03;
        
  if (n_Steps1 < init_Steps1/2) {                           // One half accel, the other half decel
    time_del = pos_Accel_(time_del);                        // Take in initial time delay 
  } else {
    time_del = neg_Accel_(time_del); 
  }
  rounded_time_del = round(time_del*500000); 
      
  if (dir1 == CW) {
    GPIO_setOutputHighOnPin(GPIO_PORT_P1, DIRpin1);         // Set clockwise direction
    // Step motor 1
    GPIO_setOutputHighOnPin(GPIO_PORT_P1, STEPpin1);       
    delay_us(PULSE_WIDTH); // rounded_time_del 25000
    GPIO_setOutputLowOnPin(GPIO_PORT_P1, STEPpin1);
  } else {
    GPIO_setOutputLowOnPin(GPIO_PORT_P1, DIRpin1);          // Set anti-clockwise direction
    GPIO_setOutputHighOnPin(GPIO_PORT_P1, STEPpin1);       
    delay_us(PULSE_WIDTH); // rounded_time_del 25000
    GPIO_setOutputLowOnPin(GPIO_PORT_P1, STEPpin1);
  }
  i++;
}


void stepMotor2_(int init_Steps2, int n_Steps2, bool dir2) {
  int i = 0;
  int DIRpin2 = GPIO_PIN4;     // Stepper 2 DIR pin
  int STEPpin2 = GPIO_PIN3;    // Stepper 2 STEP pin
  float time_del = 0.003;      // Time delay
  int rounded_time_del = 0;
  //ADCCTL0 |= 0x03;
        
  if (i < init_Steps2/2) {                                     // One half accel, the other half decel
    time_del = pos_Accel_(time_del);                        // Take in initial time delay 
  } else {
    time_del = neg_Accel_(time_del); 
  }
  rounded_time_del = round(time_del*500000); 
      
  if (dir2 == CW) {
    GPIO_setOutputHighOnPin(GPIO_PORT_P1, DIRpin2);         // Set clockwise direction
    // Step motor 2
    GPIO_setOutputHighOnPin(GPIO_PORT_P1, STEPpin2);       
    delay_us(PULSE_WIDTH); // rounded_time_del 25000
    GPIO_setOutputLowOnPin(GPIO_PORT_P1, STEPpin2);
  } else {
    GPIO_setOutputLowOnPin(GPIO_PORT_P1, DIRpin2);          // Set anti-clockwise direction
    GPIO_setOutputHighOnPin(GPIO_PORT_P1, STEPpin2);       
    delay_us(PULSE_WIDTH); // rounded_time_del 25000
    GPIO_setOutputLowOnPin(GPIO_PORT_P1, STEPpin2);
  }
  i++;
}


// ---- Without Acceleration
void stepMotor1Basic_(bool DIR_1) {
  
  if (DIR_1 == CW) {
    GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN6); // Set clockwise
    delay_us(PULSE_WIDTH);
    // Step Motor
    GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN7);
    delay_us(PULSE_WIDTH);
    GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN7);
  } else {
    GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN6); // Set anti-clockwise
    delay_us(PULSE_WIDTH);
    // Step motor
    GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN7);
    delay_us(PULSE_WIDTH);
    GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN7);
    delay_us(PULSE_WIDTH);
  }
}

void stepMotor2Basic_(bool DIR_2) {
  if (DIR_2 == CW) {
    GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN4); // Set clockwise
    delay_us(PULSE_WIDTH);
    // Step Motor
    GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN3);
    delay_us(PULSE_WIDTH);
    GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN3);
  } else {
    GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN4); // Set anti-clockwise
    delay_us(PULSE_WIDTH);
    // Step motor
    GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN3);
    delay_us(PULSE_WIDTH);
    GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN3); 
  }
}

void penManualControl_() {
  static unsigned char lastState = 0;
  static unsigned char penState = 0;    // Initialize servoPos to 0 (Pen is down)
  
  unsigned char pinState = GPIO_getInputPinValue(GPIO_PORT_P2, GPIO_PIN5);
  
  if (pinState && !lastState) {
    penState = !penState;               // Toggle pen state
    if (penState) {                     // Set servo position based on pen state
      TA1CCR2 = 2080;
    } else {
      TA1CCR2 = 1040;
    }                                   
    for (int i = 0; i < 10; i++) {      // Wait for servo to reach position
      __delay_cycles(500);
    }
  }
  lastState = pinState;
}

void penUp_() {
  TA1CCR2 = 2080;
  for (int i = 0; i < 10; i++) {        // Wait for servo to reach position
    __delay_cycles(500);
    
  }
}

void penDown_() {
  TA1CCR2 = 1040;
  for (int i = 0; i < 10; i++) {
    __delay_cycles(500);
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
      __delay_cycles(100000);                                           // More precise movement
      GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN7);   
      GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN6);                 // Set clockwise direction
      pinState_L = GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN2);      // Check button value continuously    
    }
    SW1_interruptFlag = 0;              // Clear SW1 flag
    __delay_cycles(500000);             // Delay 0.5 second
  }
  lastPinState_L = pinState_L;          // Update last pin state

  if (SW2_interruptFlag && lastPinState_R) {
    // Wait for the button to be pressed
    while (!pinState_R) {
      GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN7);
      __delay_cycles(100000);
      GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN7);   
      GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN6);                  // Set anti-clockwise direction
      pinState_R = GPIO_getInputPinValue(GPIO_PORT_P2, GPIO_PIN6);      // Check button value continuously    
    }
    SW2_interruptFlag = 0;              // Clear SW2 flag
    __delay_cycles(500000);             // Delay 1 second
  }
  lastPinState_R = pinState_R;          // Update last pin state 
}

float pos_Accel_(float time_del) {              // Previous time delay input argument
  float dVelocity = time_del * accel_Val_();    // !! accel_val
  time_del = 1/(dVelocity + 1/time_del);        // td2 = ... 1/td1
  if (time_del < 0.00025) {
    time_del = 0.00025;                         // Minimum time delay 
  }
  return time_del;                              // Return new next time delay
}

float neg_Accel_(float time_del) {
  float dVelocity = time_del * -1 * accel_Val_();
  time_del = 1/(dVelocity + 1/time_del);
  if (time_del > 0.003) {
    time_del = 0.003;
  }
  return time_del;
}