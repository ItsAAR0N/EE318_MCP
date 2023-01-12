// Main c file 

/********************************

main.c
Default Template file for testing purposes

Aaron Shek, @ 2023 University of Strathclyde

*********************************/

#include <msp430.h>
#include <driverlib.h>
#include <stdio.h>

unsigned int rate_ = 0;
uint32_t TimerA0_period = 20000; // PWM Period
unsigned int i;

// ADC interrupt service routine
#pragma vector=ADC_VECTOR           
__interrupt void ADC_ISR(void)
{
  switch(__even_in_range(ADCIV,ADCIV_ADCIFG))
  {
    case ADCIV_ADCIFG:              
    rate_ = (int)(600 + (2.05*ADCMEM0)); // Max value of 2695, min 600
    //printf("%d\n",rate_);
    __bic_SR_register_on_exit(LPM0_bits); // Clear CPUOFF bit from LMP0 to prevent MCU sleeping
    //ADC_clearInterrupt(ADC_BASE,ADC_COMPLETED_INTERRUPT);
    break;
  }
}

void main(void)
{
    WDTCTL = WDTPW | WDTHOLD; // Stop WDT
    
    P1DIR |= 0x80; // P1.7 output
    P1SEL0 |= 0x80; // P1.7 options select
    
    P4DIR |= 0x01; // Indicator LED
    P4OUT |= 0x01; 
    
    // Configure Pin for ADC
    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P8,GPIO_PIN1,GPIO_PRIMARY_MODULE_FUNCTION);
    
    // Disable the GPIO power-on default high-impedance mode to activate
    // previously configured port settings
    PM5CTL0 &= ~LOCKLPM5; PMM_unlockLPM5();
    
    // Timer TA0 setup
    TA0CCR0 = TimerA0_period; // PWM Period
    TA0CCTL1 = OUTMOD_7; // CCR1 reset/set                      
    TA0CTL = TASSEL_2 | MC_1; // SMCLK, up mode
    
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
      
    while(1) {  
      ADCCTL0 |= 0x03;          // Sampling and conversion start
      while(ADCCTL0 & ADCBUSY);
      int memval = rate_;
      TA0CCR1 = memval;     
      __delay_cycles(10000);
    }  
    // __bis_SR_register(LPM0_bits); // Enter LPM0
}
