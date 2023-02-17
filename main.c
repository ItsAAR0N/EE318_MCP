// Main c file 

/********************************
main.c
Default Template file for testing purposes
Aaron Shek, @ 2023 University of Strathclyde
*********************************/

#include <msp430.h>
#include <driverlib.h>
#include "motor.h"

#define STEP_PIN BIT7
#define LED_PIN BIT0
#define DIR_PIN BIT6

unsigned char SW1_interruptFlag_ = 0;
unsigned char SW2_interruptFlag_ = 0;
unsigned int rate_ = 0;

#pragma vector = PORT1_VECTOR // Interrupt handler
__interrupt void P1_ISR(void)
{
  switch(__even_in_range(P1IV,P1IV_P1IFG7)) 
  {
  case P1IV_P1IFG3: //It is SW1
    SW1_interruptFlag_ = 1;
    GPIO_clearInterrupt(GPIO_PORT_P1, GPIO_PIN3);
    break;
        
  case P1IV_P1IFG4:  //It is SW2
    SW2_interruptFlag_ = 1;
    GPIO_clearInterrupt(GPIO_PORT_P1, GPIO_PIN4);
    break;
  }    
}

// ADC interrupt service routine
#pragma vector=ADC_VECTOR           
__interrupt void ADC_ISR(void)
{
  switch(__even_in_range(ADCIV,ADCIV_ADCIFG))
  {
    case ADCIV_ADCIFG:              
    rate_ = (int)(1000 + (2.00*ADCMEM0)); // Max value of 2695, min 600
    //printf("%d\n",rate_);
    __bic_SR_register_on_exit(LPM0_bits); // Clear CPUOFF bit from LMP0 to prevent MCU sleeping
    //ADC_clearInterrupt(ADC_BASE,ADC_COMPLETED_INTERRUPT);
    break;
  }
}


void main(void)
{
  WDTCTL = WDTPW | WDTHOLD;
  PM5CTL0 &= ~LOCKLPM5;
    // Configure Pin for ADC
    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P8,GPIO_PIN1,GPIO_PRIMARY_MODULE_FUNCTION);  
  unsigned int MS1_isLow = 1;
   
  initialiseGPIO();
  // Configure Timer_A
  TA0CCR0 = 3000;
  TA0CCR1 = 1000;
  TA0CCTL1 = OUTMOD_7;
  TA0CTL = TASSEL_2 | MC_1 | TACLR; 
  
  P1DIR |= 0x80; // P1.7 output
  P1SEL0 |= 0x80; // P1.7 options select
  
  P8OUT = 0x01; // P8.0 stays high
  P5OUT = !(0x01); // Initially off
  
  // Disable the GPIO power-on default high-impedance mode
  // to activate previously configured port settings
  PMM_unlockLPM5();  

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
  
  __enable_interrupt();
  
  // Rotate continuously in full stepping
  while (1) {
    
    if (SW1_interruptFlag_ == 1) // If sw1 flag activated
    {
        MS1_isLow = !MS1_isLow; // 
        SW1_interruptFlag_ = 0;
    } 

    // if (SW1_interruptFlag_ == 1) // If sw1 flag activated
    // {
    //     MS1_isLow = !MS1_isLow; // 
    //     SW1_interruptFlag_ = 0;
    // } 
    
    if (1 == MS1_isLow) {
      P2OUT = !(0x80); 
      P5OUT = 0x01; // MS1 off
    }
    else {
      P2OUT = 0x80; // Until turned on
      P5OUT = !(0x01);
    }  
    
    // Now connected to TimerA0 process instead
    //P1OUT ^= STEP_PIN;
     ADCCTL0 |= 0x03;          // Sampling and conversion start
    while(ADCCTL0 & ADCBUSY) {
      int memval = rate_;
      TA0CCR0 = memval;     
      __delay_cycles(1);
    }
    
    P4OUT ^= LED_PIN;
    __delay_cycles(1000);
  }
  
}