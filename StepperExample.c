// Main c file 

/********************************

main.c
Default Template file for testing purposes

Aaron Shek, @ 2023 University of Strathclyde

*********************************/

#include <msp430.h>
#include <driverlib.h>
#include <stdio.h>

#include "28BYJ48.h"

unsigned int stepsPerRevolution = 200;
unsigned int x = 5000;

// A = GPIO_PIN3
// A_BAR = GPIO_PIN4
// B = GPIO_PIN5
// B_BAR = GPIO_PIN6

void main(void)
{
    WDTCTL = WDTPW | WDTHOLD; // Stop WDT
    
    GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN3 | GPIO_PIN4 | GPIO_PIN5 | GPIO_PIN6);
    PM5CTL0 &= ~LOCKLPM5;
    while(1) 
    {
      for (int i = 0; i < (stepsPerRevolution/4); i++) {
        GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN3);
        GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN4);
        GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN5);
        GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN6);
        __delay_cycles(5000);   
        
        GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN3);
        GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN4);
        GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN5);
        GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN6);
        __delay_cycles(5000);
        
        GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN3);
        GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN4);
        GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN5);
        GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN6);
        __delay_cycles(5000);   
        
        GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN3);
        GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN4);
        GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN5);
        GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN6);
        __delay_cycles(5000);  
        
      }
      __delay_cycles(1000); 
           
    };   
    // __bis_SR_register(LPM0_bits); // Enter LPM0
}

