// C file for translator

/********************************
transltr.c
Translator
Aaron Shek, @ 2023 University of Strathclyde
*********************************/
#include <msp430.h>
#include <driverlib.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include "transltr.h"

#define PI 3.14159265358979323846

#define WIDTH 320
#define SPACING 110
#define ARM 190

#define LED_OUT P1OUT
#define LED_DIR P1DIR
#define LED_PIN BIT2

#define SMCLK_115200 0
#define SMCLK_9600 1
#define UART_MODE       SMCLK_115200 //SMCLK_9600//

const float offset_1 = (WIDTH / 2) - (SPACING / 2); // M1 offset
const float offset_2 = (WIDTH / 2) - (SPACING / 2) + SPACING; // M2 
const float space_dim_Y = 350; // Length of sheet
const float length_arm_L = ARM;

float x_coord;
float y_coord;

// UART initialisation
void initUART_()
{
    // Configure USCI_A0 for UART mode
    UCA0CTLW0 |= UCSWRST; // Put eUSCI in reset
#if UART_MODE == SMCLK_115200

    UCA0CTLW0 |= UCSSEL__SMCLK; // CLK = SMCLK  (DCO @ 8MHz)
    // Baud Rate Setting
    // Use Table 15-4
    // N = f_BRCLK/Baud Rate = 8 MHz/115200 = 69.444.. (N >16) 
    UCA0BRW = 4; // INT(N/16) = 4
    // UCBRFx = INT[(N/16) - INT(N/16) * 16] = approx. 5. 
    UCA0MCTLW |= UCOS16 | UCBRF_5 | 0x55;   //0x55 is UCBRSx = 0x55

#elif UART_MODE == SMCLK_9600

    UCA0CTLW0 |= UCSSEL__SMCLK; // CLK = SMCLK
    // Baud Rate Setting
    // Use Table 21-5
    UCA0BRW = 52; // INT(N/16) = 52
    UCA0MCTLW |= UCOS16 | UCBRF_5 | 0x49;   //0x49 is UCBRSx = 0x49
#else
    # error "Please specify baud rate to 115200 or 9600"
#endif

    UCA0CTLW0 &= ~UCSWRST; // Initialize eUSCI
    UCA0IE |= UCRXIE; // Enable USCI_A0 RX interrupt
}

void initGPIO_()
{
    LED_DIR |= LED_PIN;
    LED_OUT &= ~LED_PIN;

    // USCI_A0 UART operation
    P1SEL0 |= BIT0 | BIT1;

    // Disable the GPIO power-on default high-impedance mode to activate
    // previously configured port settings
    PM5CTL0 &= ~LOCKLPM5;
}

void initClockTo8MHz_()
{
    // Configure one FRAM waitstate as required by the device datasheet for MCLK
    // operation beyond 8MHz _before_ configuring the clock system.
    FRCTL0 = FRCTLPW | NWAITS_1;

    __bis_SR_register(SCG0);    // disable FLL
    CSCTL3 |= SELREF__REFOCLK;  // Set REFO as FLL reference source
    CSCTL0 = 0;                 // clear DCO and MOD registers
    CSCTL1 &= ~(DCORSEL_7);     // Clear DCO frequency select bits first
    CSCTL1 |= DCORSEL_3;        // Set DCO = 16MHz
    CSCTL2 = FLLD_0 + 244;      // set to fDCOCLKDIV = (FLLN + 1)*(fFLLREFCLK/n)
                                //                   = (244 + 1)*(32.768 kHz/1)
                                //                   = 8 MHz
    __delay_cycles(3);
    __bic_SR_register(SCG0); // enable FLL
    while(CSCTL7 & (FLLUNLOCK0 | FLLUNLOCK1)); // FLL locked

    CSCTL4 = SELMS__DCOCLKDIV | SELA__REFOCLK;
}

void GCode_Interpret_(char* buffer)
{
  // Typical G-Code command as follows: G## X## Y## Z## F##
  char* ptr; // pointer used for string parsing
  int x_coord, y_coord; // variables to store X, Y, and Z values
  
  // Extract G value
  ptr = strstr(buffer, "G"); // Pen up-down
  x_coord = atoi(ptr+1); 

  // Extract X value
  ptr = strstr(buffer, "X"); // find the "X" character in the string
  x_coord = atoi(ptr+1); // convert the substring after "X" to an integer

  // Extract Y value
  ptr = strstr(buffer, "Y"); 
  y_coord = atoi(ptr+1); 

  //printf("X: %d\nY: %d\n", x_coord, y_coord);
  
  // To call the function for example
  //char input[] = "X30 Y20 Z200";
  //GCode_Interpret_(input);
}

InvKVals calc_invK(float x_coord, float y_coord) { 
  InvKVals result;
  if (x_coord < offset_1) {
    // Left arm if on LHS of plane
    result.dist_1 = sqrt((offset_1 - x_coord)*(offset_1 - x_coord) + (space_dim_Y - y_coord)*(space_dim_Y - y_coord));
    result.ang_1 = PI + atan((offset_1 - x_coord)/(space_dim_Y - y_coord)) + acos(result.dist_1/(2 * length_arm_L));
    
    // Right arm if on LHS of plane
    result.dist_2 = sqrt((offset_2 - x_coord)*(offset_2 - x_coord) + (space_dim_Y - y_coord)*(space_dim_Y - y_coord));
    result.ang_2 = PI + atan((offset_2 - x_coord)/(space_dim_Y - y_coord)) + acos(result.dist_2/(2 * length_arm_L)); 
  } else {
    // Left arm if on RHS of plane  
    result.dist_1 = sqrt((x_coord - offset_1)*(x_coord - offset_2) + (space_dim_Y - y_coord)*(space_dim_Y - y_coord));
    result.ang_1 = PI + atan((x_coord - offset_1)/(space_dim_Y - y_coord)) + acos(result.dist_1/(2 * length_arm_L));
    
    // Right arm if on RHS of plane
    result.dist_2 = sqrt((x_coord - offset_2)*(x_coord - offset_2) + (space_dim_Y - y_coord)*(space_dim_Y - y_coord));
    result.ang_2 = PI - atan((x_coord - offset_2)/(space_dim_Y - y_coord)) + acos(result.dist_2/(2 * length_arm_L));  
  }
  
  return result;
}

long mapRange_(long x, long in_min, long in_max, long out_min, long out_max) { 
  return (x - in_min)*(out_max - out_min)/(in_max - in_min) + out_min;
}
