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
#include "motor.h"

// ---- Constants ----
#define PI 3.14159265358979323846
#define RADTODEG 57.2958 // 1 Rad = 57.3 degrees
#define STEPSPERDEG 1.8 // 1.8 deg/step

// ---- Physical dimensions ----
#define WIDTH 320
#define SPACING 110
#define ARM 190

// ---- LED setup ----
#define LED_OUT P1OUT
#define LED_DIR P1DIR
#define LED_PIN BIT2

// ---- Motor definition ---- 
#define M1 1
#define M2 2

// ---- UART interface ----
#define SMCLK_115200 0
#define SMCLK_9600 1
#define UART_MODE       SMCLK_115200    // SMCLK_9600

// ---- Dimension calculations ----
const float offset_1 = (WIDTH / 2) - (SPACING / 2);             // M1 offset
const float offset_2 = (WIDTH / 2) - (SPACING / 2) + SPACING;   // M2 
const float space_dim_Y = 350;                                  // Length of sheet
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

void processUARTinstr_(char* buffer)
{
  // Typical G-Code command as follows: G## X## Y## Z## F##
  char* ptr;                            // pointer used for string parsing
  float g_cmd, x_coord, y_coord;        // variables to store X, Y, and Z values
  
  // Extract G value
  ptr = strstr(buffer, "G");            // Pen up-down
  g_cmd = atoi(ptr+1); 
  
  // Extract X value
  ptr = strstr(buffer, "X");            // find the "X" character in the string
  x_coord = atoi(ptr+1);                // convert the substring after "X" to an integer
  
  // Extract Y value
  ptr = strstr(buffer, "Y"); 
  y_coord = atoi(ptr+1); 
  
  MoveTo_(x_coord,y_coord);
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

void MoveTo_(float x_coord, float y_coord)
{ 
  float currentSteps_1;
  float currentSteps_2;
  InvKVals result = calc_invK(x_coord, y_coord); 
  currentSteps_1 = round((result.ang_1*RADTODEG)/STEPSPERDEG);
  currentSteps_2 = round((result.ang_2*RADTODEG)/STEPSPERDEG);     
  stepMotor_(currentSteps_1,true,M1);
  stepMotor_(currentSteps_2,true,M2);
  // calc_invK(float x_coord, float y_coord);
}
                       
long mapRange_(long x, long in_min, long in_max, long out_min, long out_max) { 
  return (x - in_min)*(out_max - out_min)/(in_max - in_min) + out_min;
}


// ---- TEST CASES ---- 

void abc() {
  // ---- Letter C
  penUp_();
  MoveTo_(71.607584, 21.035879);
  penDown_();
  MoveTo_(70.163261, 20.392706);
  MoveTo_(68.665833, 19.931058);
  MoveTo_(67.123530, 19.654409);
  MoveTo_(65.471170, 19.558347);
  MoveTo_(60.824111, 20.352237);
  MoveTo_(57.604313, 22.327055);
  MoveTo_(55.521083, 25.458294);
  MoveTo_(54.702496, 29.861135);
  MoveTo_(55.523176, 34.275230);
  MoveTo_(57.604313, 37.395213);
  MoveTo_(60.826497, 39.380152);
  MoveTo_(65.471170, 40.177231);
  MoveTo_(67.123530, 40.081169);
  MoveTo_(68.665833, 39.804521);
  MoveTo_(70.163261, 39.342872);
  MoveTo_(71.607584, 38.699700);
  MoveTo_(71.607584, 34.586572);
  MoveTo_(70.133934, 35.457974);
  MoveTo_(68.798946, 36.010859);
  MoveTo_(67.396000, 36.346751);
  MoveTo_(65.883816, 36.463436);
  MoveTo_(63.362672, 35.969576);
  MoveTo_(61.571020, 34.706372);
  MoveTo_(60.460591, 32.773568);
  MoveTo_(60.000312, 29.861135);
  MoveTo_(60.459740, 26.961684);
  MoveTo_(61.571020, 25.029205);
  MoveTo_(63.362672, 23.766000);
  MoveTo_(65.883816, 23.272141);
  MoveTo_(67.396000, 23.388826);
  MoveTo_(68.798946, 23.724718);
  MoveTo_(70.133934, 24.277603);
  MoveTo_(71.607584, 25.149006);
  MoveTo_(71.607584, 21.035879);
  penUp_();

  // ---- Top inside-loop in letter 'B'
  penUp_();
  MoveTo_(43.041974, 32.124019);
  penDown_();
  MoveTo_(44.193140, 32.287491);
  MoveTo_(44.878907, 32.656463);
  MoveTo_(45.321578, 33.273441);
  MoveTo_(45.504526, 34.227172);
  MoveTo_(45.322608, 35.168069);
  MoveTo_(44.878907, 35.784570);
  MoveTo_(44.190670, 36.163294);
  MoveTo_(43.041974, 36.330325);
  MoveTo_(40.206713, 36.330325);
  MoveTo_(40.206713, 32.124019);
  MoveTo_(43.041974, 32.124019);
  penUp_();

  // ----- bottom inside-loop in letter 'B'
  penUp_();
  MoveTo_(43.215018, 23.431875);
  penDown_();
  MoveTo_(44.684832, 23.634884);
  MoveTo_(45.531148, 24.084119);
  MoveTo_(46.084429, 24.845298);
  MoveTo_(46.316505, 26.054160);
  MoveTo_(46.088504, 27.238072);
  MoveTo_(45.544461, 27.984270);
  MoveTo_(44.697894, 28.432828);
  MoveTo_(43.215018, 28.636513);
  MoveTo_(40.206713, 28.636513);
  MoveTo_(40.206713, 23.431875);
  MoveTo_(43.215018, 23.431875);
  penUp_();

  // ---- Outside of letter 'B'
  penUp_();
  MoveTo_(47.980391, 30.579932);
  penDown_();
  MoveTo_(49.467494, 29.872216);
  MoveTo_(50.536123, 28.809558);
  MoveTo_(51.189538, 27.438932);
  MoveTo_(51.441274, 25.641517);
  MoveTo_(50.881551, 23.051631);
  MoveTo_(49.497855, 21.355344);
  MoveTo_(47.408388, 20.394118);
  MoveTo_(43.587730, 19.944368);
  MoveTo_(35.081941, 19.944368);
  MoveTo_(35.081941, 39.817832);
  MoveTo_(42.775754, 39.817832);
  MoveTo_(46.788467, 39.403201);
  MoveTo_(48.765745, 38.566589);
  MoveTo_(50.084134, 37.024736);
  MoveTo_(50.629298, 34.559950);
  MoveTo_(50.441596, 33.165564);
  MoveTo_(49.950432, 32.084086);
  MoveTo_(49.146555, 31.229561);
  MoveTo_(47.980391, 30.579932);
  penUp_();

  // ---- Outside of letter 'A'
  penUp_();
  MoveTo_(26.057020, 23.564986);
  penDown_();
  MoveTo_(18.043741, 23.564986);
  MoveTo_(16.779187, 19.944368);
  MoveTo_(11.627794, 19.944368);
  MoveTo_(18.988829, 39.817832);
  MoveTo_(25.098621, 39.817832);
  MoveTo_(32.459656, 19.944368);
  MoveTo_(27.308262, 19.944368);
  MoveTo_(26.057020, 23.564986);
  penUp_();

  // ---- Inside of letter 'A'
  penUp_();
  MoveTo_(19.321606, 27.252160);
  penDown_();
  MoveTo_(24.765843, 27.252160);
  MoveTo_(22.050380, 35.158949);
  MoveTo_(19.321606, 27.252160);
  penUp_();

  // ---- Home
  MoveTo_(0.0000, 0.0000);
}

void radials() {

  // ---- Move to the centre of the square
  penUp_();
  MoveTo_(100, 100);

  // ---- Draw octant 0 radials
  penDown_();
  MoveTo_(150, 100);
  penUp_();
  MoveTo_(100, 100);

  penDown_();
  MoveTo_(150, 125);
  penUp_();
  MoveTo_(100, 100);

  penDown_();
  MoveTo_(150, 150);
  penUp_();
  MoveTo_(100, 100);

  // ---- Draw octant 1 radials
  penDown_();
  MoveTo_(125, 150);
  penUp_();
  MoveTo_(100, 100);

  penDown_();
  MoveTo_(100, 150);
  penUp_();
  MoveTo_(100, 100);

  // ---- Draw octant 2 radials
  penDown_();
  MoveTo_(75, 150);
  penUp_();
  MoveTo_(100, 100);

  penDown_();
  MoveTo_(50, 150);
  penUp_();
  MoveTo_(100, 100);

  // ---- Draw octant 3 radials
  penDown_();
  MoveTo_(50, 125);
  penUp_();
  MoveTo_(100, 100);

  penDown_();
  MoveTo_(50, 100);
  penUp_();
  MoveTo_(100, 100);

  // ---- Draw octant 4 radials
  penDown_();
  MoveTo_(50, 75);
  penUp_();
  MoveTo_(100, 100);

  penDown_();
  MoveTo_(50, 50);
  penUp_();
  MoveTo_(100, 100);

  // ---- Draw octant 5 radials
  penDown_();
  MoveTo_(75, 50);
  penUp_();
  MoveTo_(100, 100);

  penDown_();
  MoveTo_(100, 50);
  penUp_();
  MoveTo_(100, 100);

  // ---- Draw octant 6 radials
  penDown_();
  MoveTo_(125, 50);
  penUp_();
  MoveTo_(100, 100);

  penDown_();
  MoveTo_(150, 50);
  penUp_();
  MoveTo_(100, 100);

  // ---- Draw octant 7 radials
  penDown_();
  MoveTo_(150, 75);
  penUp_();
  MoveTo_(100, 100);
  penUp_();

  // ---- Draw box
  MoveTo_(50, 50);
  penDown_();
  MoveTo_(50, 150);
  MoveTo_(150, 150);
  MoveTo_(150, 50);
  MoveTo_(50, 50);
  penUp_();

  // ---- Home
  MoveTo_(0.0000, 0.0000);
}