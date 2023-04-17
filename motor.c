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

// ---- ADC interrupt service routine ---- 
volatile uint16_t ADC_results[3];
#pragma vector=ADC_VECTOR           
__interrupt void ADC_ISR(void)
{
  static uint8_t ADC_index = 0;
  ADC_results[ADC_index] = ADCMEM0; // Store the ADC conversion result
  
  ADC_index++;
  if (ADC_index == 3)
  {
    ADC_index = 0; // Reset the ADC index
    
    // Process the ADC results here
    uint16_t rate_1 = mapRange_(ADC_results[0], 0, 1023, 1000, 32000);
    uint16_t rate_2 = mapRange_(ADC_results[1], 0, 1023, 1000, 32000);
    uint16_t rate_3 = mapRange_(ADC_results[2], 0, 1023, 1000, 32000);
    // Do something with the ADC results, such as send them over UART or update a display
    
    __bic_SR_register_on_exit(LPM0_bits); // Clear CPUOFF bit from LMP0 to prevent MCU sleeping
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

void initialiseTimer_()
{
  // Timer TA1 setup
  TA1CCR0 = TimerA1_period; // PWM Period
  TA1CCTL2 = OUTMOD_7; // CCR1 reset/set                      
  TA1CTL = TASSEL_2 | MC_1; // SMCLK, up mode


}

void initialiseADCpot_()
{
  // ADC setup
  // Configure Pin for ADC
  GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P8,GPIO_PIN1,GPIO_PRIMARY_MODULE_FUNCTION);
  
  SYSCFG2 |= ADCPCTL0 | ADCPCTL1 | ADCPCTL9;          // Turn on analogue pin A9 (so it is not a GPIO).
  
  // Configure CLock source, operation mode
  ADCCTL0 |= ADCSHT_2 | ADCON;  // 16 ADCCLK cycles, turn on ADC.
  ADCCTL1 |= ADCSHP | ADCCONSEQ_3; // ADDCLK = MODOSC; sampling timer; Repeat-sequence-of-channels
  ADCCTL2 |= ADCRES;            // 10 bit conversion results
  
  // Configure ADC mux and +ve & -ve references
  ADCMCTL0 |= ADCINCH_9 | ADCSREF_0;        // A9, A8, A5 ADC input select; Vref = AVCC    ADCINCH_8 | ADCINCH_5
  ADCIFG &= ~0x01;              //Clear interrupt flag  
  ADCIE |= ADCIE0;              //Enable ADC conversion complete interrupt
}

int accel_Val_()
{
  return rate_;
}

void delay_us(unsigned long delay) 
{                                       
  while (delay--) {     //  I have no idea if this is us or ms lol
    __delay_cycles(1);
  }
} 

void stepMotor_(int n_Steps, bool dir, int motor_Num) // float current_position
{ 
  int i = 0;
  int DIRpin, STEPpin, ENpin;
  
  float time_del = 0.003;       // time delay
  int rounded_time_del = 0;
  
  if (motor_Num == 1) {
    DIRpin = GPIO_PIN6;         // Stepper 1 DIR pin
    STEPpin = GPIO_PIN7;        // Stepper 1 STEP pin
  } else if (motor_Num == 2) {
    DIRpin = GPIO_PIN4;         // Stepper 2 DIR pin
    STEPpin = GPIO_PIN3;        // Stepper 2 STEP pin
  } else {
    return; 
  }
  ADCCTL0 |= 0x03;
  while(i < n_Steps) {       
    if (i < n_Steps/2) {        // One half accel, the other half decel
      time_del = pos_Accel_(time_del);          // Take in initial time delay 
    } else {
      time_del = neg_Accel_(time_del); 
    }
    rounded_time_del = round(time_del*500000); 
    
    // Check if the step would exceed the 180-degree limit
    // if (dir == true && current_position + (i+1)*1.8 > 180) {
    //   n_Steps = i + round((180 - current_position)*1.8);
    // } else if (dir == false && current_position - (i+1)*1.8 < 0) {
    //   n_Steps = i + round(current_position*1.8);
    // }
    
    if (dir == true) {
        GPIO_setOutputHighOnPin(GPIO_PORT_P1, DIRpin);  // Set clockwise direction
        // current_position += 1.8; // Update current position for clockwise direction
    } else {
        GPIO_setOutputLowOnPin(GPIO_PORT_P1, DIRpin);   // Set anti-clockwise direction
        // current_position -= 1.8; // Update current position for anti-clockwise direction
    }
    GPIO_setOutputHighOnPin(GPIO_PORT_P1, STEPpin);
    delay_us(25000); // rounded_time_del
    GPIO_setOutputLowOnPin(GPIO_PORT_P1, STEPpin);
    i++;
  }
}

void stepMotor1Basic_(bool DIR_1) {
  if (DIR_1 == CW) {
    GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN6); // Set clockwise
    // Step Motor
    GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN7);
    delay_us(10000);
    GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN7);
  } else {
    GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN6); // Set anti-clockwise
    // Step motor
    GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN7);
    delay_us(10000);
    GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN7);
  }
  //printf("x\n");
}

void stepMotor2Basic_(bool DIR_2) {
  if (DIR_2 == CW) {
    GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN4); // Set clsockwise
    // Step Motor
    GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN7);
    delay_us(10000);
    GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN7);
  } else {
    GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN4); // Set anti-clockwise
    // Step motor
    GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN3);
    delay_us(10000);
    GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN3); 
  }
  //printf("x\n");  
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