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
#define STEPSPERDEG 3200/360 // 0.556 degrees per step

// ---- GCODE buffer ---
//#define MAX_COMMANDS 12
float gcodeBuffer[MAX_COMMANDS][2];
volatile uint8_t gcodeIndex = 0;

// ---- Physical dimensions ----
//#define WIDTH 300
//#define SPACING 110
#define ARM 190
#define SCALE_FACTOR 1
#define X_OFFSET 0
#define Y_OFFSET 0

// ---- LED setup ----
#define LED_OUT P1OUT
#define LED_DIR P1DIR
#define LED_PIN BIT2

// ---- Motor definition ---- 
#define M1 1
#define M2 2
#define CW 1                            // Motor directions
#define CCW 0
// ---- UART interface ----
#define SMCLK_115200 0
#define SMCLK_9600 1
#define UART_MODE       SMCLK_115200    // SMCLK_9600

// ---- Dimension calculations ----
// const float offset_1 = (WIDTH / 2) - (SPACING / 2);             // M1 offset
// const float offset_2 = (WIDTH / 2) - (SPACING / 2) + SPACING;   // M2 
const float offset_1 = 105;
const float offset_2 = 215;
const float space_dim_Y = 300;                                  // Length of sheet
const float length_arm = ARM;
// const float YAXIS = 345;                // Motor distance to (0,0)

// ---- Inverse Kinematic calculations ----
float x_coord;
float y_coord;
long previousSteps_1 = 0;
long previousSteps_2 = 0;
int32_t  STEPS1;
int32_t  STEPS2;

// ---- Calculate Delays ----
long DELAY1;
long DELAY2;
long DELAY_MIN = 10000; // Minimum inter-step delay (uS) between motor steps

// ---- Timer delay ---- 
volatile uint64_t timer_count = 0;

// ---- Timer count ---- 
unsigned long count = 0;
static uint64_t start_time = 0;
static long elapsed_time = 0.0;
static bool is_running = false;

volatile unsigned long micros_counter = 0;

#pragma vector = TIMER0_A0_VECTOR
__interrupt void Timer_A0_ISR(void)
{
  micros_counter += 1000;
}

void initialiseTimerMicros_()
{
  // Timer TA0 setup
  TA0CTL = TASSEL_1 + ID_3 + MC_1; // select ACLK as clock source, divide by 8, and set to up mode
  TA0CCR0 = 1000; // set the compare register value to 1000
  TA0CCTL0 = CCIE; // enable the compare interrupt
}

long Micros_()
{
  return micros_counter;
}

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
    UCA0BRW = 4;                // INT(N/16) = 4
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

    __bis_SR_register(SCG0);    // Disable FLL
    CSCTL3 |= SELREF__REFOCLK;  // Set REFO as FLL reference source
    CSCTL0 = 0;                 // Clear DCO and MOD registers
    CSCTL1 &= ~(DCORSEL_7);     // Clear DCO frequency select bits first
    CSCTL1 |= DCORSEL_3;        // Set DCO = 16MHz
    CSCTL2 = FLLD_0 + 244;      // set to fDCOCLKDIV = (FLLN + 1)*(fFLLREFCLK/n)
                                //                   = (244 + 1)*(32.768 kHz/1)
                                //                   = 8 MHz
    __delay_cycles(3);
    __bic_SR_register(SCG0);    // Enable FLL
    while(CSCTL7 & (FLLUNLOCK0 | FLLUNLOCK1)); // FLL locked

    CSCTL4 = SELMS__DCOCLKDIV | SELA__REFOCLK;
}

void processUARTinstr_(char* buffer)
{
  // Typical G-Code command as follows: G## X## Y## Z## F##
  char* ptr;                                            // pointer used for string parsing
  
  int g_cmd = 0; x_coord = 0 , y_coord = 0;           
  float x_coord = 0 , y_coord = 0;
  
  // Extract G value
  ptr = strstr(buffer, "G");                              // Pen up-down
  if (ptr != NULL) {
    g_cmd = atoi(ptr+1);
  }

  // Extract X value
  ptr = strstr(buffer, "X");                              // find the "X" character in the string
  if (ptr != NULL) {
    x_coord = atoi(ptr+1);                                // convert the substring after "X" to an integer
  }
  // Extract Y value
  ptr = strstr(buffer, "Y"); 
  if (ptr != NULL) {   
    y_coord = atoi(ptr+1); 
  }
  
  switch (g_cmd) {
  case 0: 
    penDown_(); 
    break;
  case 1: 
    penUp_(); 
    break;
  case 2: 
    Star_();
    break;
  case 3: 
    Spiral_();
    break;
  case 4: 
    Pentagon_();
    break;
  case 5: 
    Circle_();
    break;
  case 6: 
    HilbertCurve_();
    break;
  case 7: 
    UoS_();
    break;
  }
  if(gcodeIndex < MAX_COMMANDS) {
    gcodeBuffer[gcodeIndex][0] = x_coord;
    gcodeBuffer[gcodeIndex][1] = y_coord;
    gcodeIndex++;
  }
}

void clearExecutedCommands() {
  if(gcodeIndex > 0) {
    for(int i = 0; i < gcodeIndex - 1; i++) {
      gcodeBuffer[i][0] = gcodeBuffer[i+1][0];
      gcodeBuffer[i][1] = gcodeBuffer[i+1][1];
    }
    gcodeBuffer[gcodeIndex - 1][0] = 0.0;
    gcodeBuffer[gcodeIndex - 1][1] = 0.0;
    gcodeIndex--;
  }
}

InvKVals calc_invK(float x_coord, float y_coord) { 
  InvKVals result;
  
  // Calculate distances (L)
  result.dist_1 = sqrt((offset_1 - x_coord)*(offset_1 - x_coord) + (space_dim_Y - y_coord)*(space_dim_Y - y_coord));

  result.dist_2 = sqrt((offset_2 - x_coord)*(offset_2 - x_coord) + (space_dim_Y - y_coord)*(space_dim_Y - y_coord));

  // Calculate M1 angle when at (x,y)
  if (x_coord > offset_1) {
    result.ang_1 = (PI + acos(result.dist_1/(2*length_arm)) - atan((x_coord - offset_1)/(space_dim_Y - y_coord)));      // LHS
  } else {
    result.ang_1 = (PI + acos(result.dist_1/(2*length_arm)) + atan((offset_1 - x_coord)/(space_dim_Y - y_coord)));      // RHS
  }

  // Calculate M2 angle when at start pos. (0,0)
  if (x_coord > offset_2) {
    result.ang_2 = (PI - acos(result.dist_2/(2*length_arm)) - atan((x_coord - offset_2)/(space_dim_Y - y_coord)));
  } else {
    result.ang_2 = (PI - acos(result.dist_2/(2*length_arm)) + atan((offset_2 - x_coord)/(space_dim_Y - y_coord)));
  }

  // Calculate steps required to reach (x,y) from 12
  STEPS1 = result.ang_1*RADTODEG*STEPSPERDEG; 

  STEPS2 = result.ang_2*RADTODEG*STEPSPERDEG;   
  
  
  
  return result;
}

long calculateDelay_(long Steps1, long Steps2) {
  long M1delay = DELAY_MIN; // 10000
  long Mdelay; 
  if (Steps1 > Steps2) {
    return Mdelay = (Steps1*M1delay)/Steps2;
  }
}

void calculateDelays_(long Steps1, long Steps2) {
  
  float rotateTime;
  
  long minSteps;
  long maxSteps;
  long maxDelay;
  // Find maxima and minima 
  maxSteps = max(Steps1,Steps2);
  minSteps = min(Steps1,Steps2);
  // Error prevention
  if (maxSteps < 1) {
    maxSteps = 1;
  }
  
  if (minSteps < 1) {
    minSteps = 1;
  }
  
   // Calculate the total time for completion of one move
  rotateTime = (float)(maxSteps * DELAY_MIN);
  
  // Calculate delay for motor with Minsteps
  maxDelay = (long)(rotateTime / ((float)minSteps)); 
  
  // Assign delays to each motor
  DELAY1 = (Steps1 > Steps2) ? DELAY_MIN : maxDelay; 
  DELAY2 = (Steps1 > Steps2) ? maxDelay: DELAY_MIN;
}

void MoveTo_(float x_coord, float y_coord)
{ 
  if (x_coord == 0 && y_coord == 0) {
    x_coord = x_coord*SCALE_FACTOR;
    y_coord = y_coord*SCALE_FACTOR;
  } else {  
    x_coord = (x_coord-X_OFFSET)*SCALE_FACTOR;
    y_coord = (y_coord-Y_OFFSET)*SCALE_FACTOR;
  }
  // ---- Motor steps
  long previousSteps_1 = STEPS1;
  long previousSteps_2 = STEPS2;

  long currentSteps_1;
  long currentSteps_2;
  long Steps1;
  long Steps2;
  
  // ---- Directions
  bool DIR_1; 
  bool DIR_2;
  
  // ---- Motor timers
  long currentTime; 
  long prevTime1;
  long prevTime2;
  long deltaT1; 
  long deltaT2;
  
  // Calculate steps required
  InvKVals result = calc_invK(x_coord, y_coord); 
  currentSteps_1 = STEPS1;
  currentSteps_2 = STEPS2;
  Steps1 = abs(previousSteps_1-currentSteps_1);
  Steps2 = abs(previousSteps_2-currentSteps_2);
  
  if (currentSteps_1 > previousSteps_1) {
    DIR_1 = CW;
  } else {
      DIR_1 = CCW;
  }

  if (currentSteps_2 > previousSteps_2) {
    DIR_2 = CW;
  } else {
      DIR_2 = CCW;
  }
  
  // Calculate motor delays for additional steps
  calculateDelays_(Steps1,Steps2);
  
  // ---- Reload the timers and counters
  //prevTime1 = Micros_();
  //prevTime2 = Micros_(); 
  //printf("Prev time: %f s\n", Micros_());
  int init_Steps1 = Steps1;
  int init_Steps2 = Steps2;
  long M1_delay;
  long M2_delay;
  
  // if (Steps1 > Steps2) {
  //   M2_delay = calculateDelay_(Steps1,Steps2);
  // }
  // else M1_delay = calculateDelay_(Steps1,Steps2);
    
  
  // M2_delay = calculateDelay_(Steps1,Steps2);
  while ((Steps1 != 0) || (Steps2 != 0)) {
    // ---- Step M1
    if (Steps1 > 0) {
        Steps1--;
        stepMotor1_(init_Steps1,Steps1,DIR_1);
    }
    // ---- Step M2
    if (Steps2 > 0) {
        Steps2--;
        stepMotor2_(init_Steps2,Steps2,DIR_2);
    }
  }
  
  
  
}

int max(int a, int b) {
    return (a > b) ? a : b;
}

int min(int a, int b) {
    return (a < b) ? a : b;
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

void target()
{
// ----- circle
  penUp_();
  MoveTo_(130,100);
  MoveTo_(128,88);
  MoveTo_(122,79);
  MoveTo_(112,73);
  MoveTo_(100,70);
  MoveTo_(87,73);
  MoveTo_(78,79);
  MoveTo_(71,88);
  MoveTo_(69,100);
  MoveTo_(71,111);
  MoveTo_(78,123);
  MoveTo_(87,130);
  MoveTo_(100,132);
  MoveTo_(112,130);
  MoveTo_(122,123);
  MoveTo_(129,110);
  MoveTo_(130,100);
  penUp_();

  // ----- back-slash
  penUp_();
  MoveTo_(50,150);
  penDown_();
  MoveTo_(78,123);
  MoveTo_(100,100);
  MoveTo_(123,79);
  MoveTo_(150,50);
  penUp_();

  // ----- slash
  penUp_();
  MoveTo_(50,50);
  penDown_();
  MoveTo_(78,79);
  MoveTo_(100,100);
  MoveTo_(122,123);
  MoveTo_(150,150);
  penUp_();

  // ----- square
  penUp_();
  MoveTo_(50,150);
  penDown_();
  MoveTo_(100,150);
  MoveTo_(150,150);
  MoveTo_(150,100);
  MoveTo_(150,50);
  MoveTo_(100,50);
  MoveTo_(50,50);
  MoveTo_(50,100);
  MoveTo_(50,150);
  penUp_();

  // ------ home
  MoveTo_(0.0000, 0.0000);
}
void Star_()
{
  MoveTo_(128.782807, 203.172129);
  MoveTo_(82.000352, 179.620859);
  MoveTo_(36.077709, 204.807569);
  MoveTo_(44.019721, 153.037068);
  MoveTo_(5.874853, 117.145152);
  MoveTo_(57.565742, 108.700512);
  MoveTo_(79.913563, 61.331369);
  MoveTo_(103.918275, 107.882792);
  MoveTo_(155.874853, 114.498962);
  MoveTo_(119.019695, 151.713960);
  MoveTo_(128.782807, 203.172129);
}

void Spiral_()
{
  MoveTo_(107.571205, 69.519593);
  MoveTo_(106.753109, 68.129927);
  MoveTo_(107.882905, 66.492306);
  MoveTo_(109.958053, 65.598965);
  MoveTo_(112.233779, 66.056739);
  MoveTo_(114.934669, 68.921538);
  MoveTo_(115.680249, 73.372026);
  MoveTo_(114.743987, 76.480198);
  MoveTo_(112.753843, 79.127200);
  MoveTo_(109.936148, 81.018577);
  MoveTo_(106.383781, 82.060666);
  MoveTo_(101.490891, 81.845738);
  MoveTo_(97.046695, 79.885699);
  MoveTo_(93.465633, 76.451328);
  MoveTo_(91.127426, 71.841789);
  MoveTo_(90.318516, 65.387454);
  MoveTo_(91.854096, 59.110896);
  MoveTo_(95.635484, 53.623630);
  MoveTo_(101.200840, 49.700022);
  MoveTo_(109.150654, 47.644236);
  MoveTo_(117.302703, 48.574341);
  MoveTo_(124.629866, 52.405808);
  MoveTo_(130.237893, 58.654240);
  MoveTo_(133.754282, 67.955578);
  MoveTo_(133.637593, 78.016047);
  MoveTo_(129.942411, 87.228332);
  MoveTo_(123.140387, 94.619823);
  MoveTo_(118.174827, 97.657920);
  MoveTo_(112.724784, 99.685950);
  MoveTo_(106.844532, 100.689508);
  MoveTo_(100.968947, 100.601110);
  MoveTo_(95.281281, 99.466282);
  MoveTo_(89.824180, 97.271668);
  MoveTo_(84.787196, 94.050987);
  MoveTo_(80.442869, 89.992359);
  MoveTo_(76.550653, 84.648511);
  MoveTo_(73.723942, 78.571186);
  MoveTo_(72.077848, 72.062796);
  MoveTo_(71.645559, 65.202449);
  MoveTo_(72.473140, 58.454452);
  MoveTo_(74.519149, 52.020760);
  MoveTo_(77.736761, 46.071309);
  MoveTo_(81.996589, 40.787280);
  MoveTo_(87.648252, 36.002596);
  MoveTo_(94.279034, 32.315829);
  MoveTo_(101.602045, 29.919545);
  MoveTo_(109.269784, 28.922581);
  MoveTo_(116.912242, 29.342394);
  MoveTo_(124.298830, 31.143336);
  MoveTo_(131.225454, 34.289686);
  MoveTo_(137.504694, 38.695519);
  MoveTo_(143.310212, 44.712237);
  MoveTo_(147.837024, 51.755496);
  MoveTo_(150.983411, 59.616458);
  MoveTo_(152.643152, 68.064055);
  MoveTo_(152.726979, 76.589268);
  MoveTo_(151.230640, 85.056200);
  MoveTo_(148.231051, 92.971195);
  MoveTo_(143.796036, 100.213547);
  MoveTo_(137.593051, 107.004812);
  MoveTo_(130.218912, 112.435827);
  MoveTo_(121.774642, 116.446535);
  MoveTo_(112.607746, 118.840568);
  MoveTo_(103.120582, 119.505829);
  MoveTo_(93.732534, 118.422114);
  MoveTo_(84.721058, 115.615360);
  MoveTo_(76.442800, 111.186455);
  MoveTo_(68.631901, 104.830926);
  MoveTo_(62.196392, 97.047361);
  MoveTo_(57.355820, 88.151905);
  MoveTo_(54.195529, 78.285230);
  MoveTo_(52.910600, 67.977663);
  MoveTo_(53.553285, 57.548987);
  MoveTo_(56.087907, 47.563629);
  MoveTo_(60.419878, 38.293501);
}

void Pentagon_()
{
  MoveTo_(131.492929, 102.893652);
  MoveTo_(69.689531, 113.163469);
  MoveTo_(40.374771, 60.143365);
  MoveTo_(84.060650, 17.105322);
  MoveTo_(140.374771, 43.526453);
  MoveTo_(131.492929, 102.893652);
}

void Circle_()
{
  MoveTo_(132.225429, 65.939871);
  MoveTo_(131.271309, 75.100230);
  MoveTo_(128.393363, 84.029281);
  MoveTo_(123.780027, 92.142389);
  MoveTo_(117.580768, 99.255483);
  MoveTo_(110.032179, 105.097093);
  MoveTo_(101.422352, 109.444276);
  MoveTo_(92.079878, 112.131121);
  MoveTo_(82.225429, 113.055261);
  MoveTo_(72.370980, 112.131121);
  MoveTo_(63.028505, 109.444276);
  MoveTo_(54.418678, 105.097093);
  MoveTo_(46.870090, 99.255483);
  MoveTo_(40.670831, 92.142389);
  MoveTo_(36.057495, 84.029281);
  MoveTo_(33.179549, 75.100230);
  MoveTo_(32.225429, 65.939871);
  MoveTo_(33.179549, 56.779512);
  MoveTo_(36.005495, 47.968756);
  MoveTo_(40.595415, 39.843864);
  MoveTo_(46.870090, 32.624259);
  MoveTo_(54.418678, 26.782649);
  MoveTo_(63.154044, 22.386466);
  MoveTo_(72.504233, 19.723555);
  MoveTo_(82.225429, 18.824480);
  MoveTo_(91.946625, 19.723555);
  MoveTo_(101.296813, 22.386466);
  MoveTo_(109.919146, 26.711584);
  MoveTo_(117.580768, 32.624259);
  MoveTo_(123.780027, 39.737352);
  MoveTo_(128.393363, 47.850460);
  MoveTo_(131.271309, 56.779512);
  MoveTo_(132.225429, 65.939871);
}

void UoS_()
{
  MoveTo_(106.704873, 31.097765);
  MoveTo_(105.074803, 40.322949);
  MoveTo_(103.042418, 43.359205);
  MoveTo_(100.032609, 45.684657);
  MoveTo_(95.788599, 47.124807);
  MoveTo_(90.323256, 47.408727);
  MoveTo_(85.756269, 46.582681);
  MoveTo_(82.016033, 44.615498);
  MoveTo_(78.792579, 40.704602);
  MoveTo_(77.260268, 35.509375);
  MoveTo_(76.985687, 7.930838);
  MoveTo_(82.133893, 7.930838);
  MoveTo_(82.271184, 34.534858);
  MoveTo_(83.267862, 38.348277);
  MoveTo_(85.516603, 41.239972);
  MoveTo_(89.374421, 42.835982);
  MoveTo_(94.508864, 42.805754);
  MoveTo_(98.033247, 41.351766);
  MoveTo_(100.355381, 38.467755);
  MoveTo_(101.530263, 32.818970);
  MoveTo_(101.556675, 7.930838);
  MoveTo_(106.704873, 7.930838);
  MoveTo_(106.704873, 31.097765);
  MoveTo_(140.896248, 32.137807);
  MoveTo_(139.953999, 38.586633);
  MoveTo_(137.115179, 43.500743);
  MoveTo_(132.715109, 46.540466);
  MoveTo_(126.919011, 47.442925);
  MoveTo_(121.416856, 46.174696);
  MoveTo_(117.273095, 42.828470);
  MoveTo_(114.784408, 37.789172);
  MoveTo_(114.102794, 31.277362);
  MoveTo_(115.228251, 25.110211);
  MoveTo_(118.131722, 20.468100);
  MoveTo_(122.451082, 17.641532);
  MoveTo_(128.087520, 16.806748);
  MoveTo_(133.369786, 17.995856);
  MoveTo_(137.507066, 21.223332);
  MoveTo_(140.048952, 25.984577);
  MoveTo_(140.896248, 32.137807);
  MoveTo_(135.852045, 32.137807);
  MoveTo_(134.569091, 25.230610);
  MoveTo_(130.720382, 21.578806);
  MoveTo_(127.505706, 21.009362);
  MoveTo_(124.202930, 21.599387);
  MoveTo_(121.727478, 23.293944);
  MoveTo_(120.104078, 25.909114);
  MoveTo_(119.158442, 30.962189);
  MoveTo_(119.646673, 36.790744);
  MoveTo_(121.343460, 40.432139);
  MoveTo_(124.315729, 42.677253);
  MoveTo_(128.976560, 43.132313);
  MoveTo_(132.756410, 41.377846);
  MoveTo_(135.072706, 37.703759);
  MoveTo_(135.852045, 32.137807);
  MoveTo_(176.985687, 35.595945);
  MoveTo_(176.311152, 39.161821);
  MoveTo_(174.362702, 42.476861);
  MoveTo_(171.367740, 44.931126);
  MoveTo_(167.502746, 46.605939);
  MoveTo_(157.068569, 47.107666);
  MoveTo_(147.136493, 44.254292);
  MoveTo_(147.136493, 37.806034);
  MoveTo_(154.615762, 41.687411);
  MoveTo_(162.524567, 42.881270);
  MoveTo_(168.951363, 41.108166);
  MoveTo_(170.857086, 39.190634);
  MoveTo_(171.614783, 36.760392);
  MoveTo_(171.063233, 33.492750);
  MoveTo_(169.003988, 31.560046);
  MoveTo_(154.913360, 28.217280);
  MoveTo_(149.703403, 25.044856);
  MoveTo_(147.970753, 22.098305);
  MoveTo_(147.370990, 18.424323);
  MoveTo_(147.913728, 15.002906);
  MoveTo_(149.611847, 12.087096);
  MoveTo_(154.097080, 8.778543);
  MoveTo_(159.917104, 7.310599);
  MoveTo_(167.823892, 7.696891);
  MoveTo_(175.425625, 9.932918);
  MoveTo_(175.425625, 16.017162);
  MoveTo_(167.815256, 12.479375);
  MoveTo_(160.083185, 11.763628);
  MoveTo_(154.487967, 13.965492);
  MoveTo_(153.087156, 15.898784);
  MoveTo_(152.739847, 18.296313);
  MoveTo_(153.566954, 21.079194);
  MoveTo_(155.744296, 22.845266);
  MoveTo_(168.780906, 25.796168);
  MoveTo_(173.401949, 27.910815);
  MoveTo_(176.101223, 31.048771);
  MoveTo_(176.985687, 35.595945);
}



void HilbertCurve_() 
{
  MoveTo_(12.209328, 5.391146);
  MoveTo_(12.292898, 17.903877);
  MoveTo_(13.025446, 19.790744);
  MoveTo_(14.512741, 20.922863);
  MoveTo_(17.632273, 21.253228);
  MoveTo_(19.963108, 20.348316);
  MoveTo_(21.044302, 18.656012);
  MoveTo_(21.876090, 14.094888);
  MoveTo_(24.564711, 12.315097);
  MoveTo_(36.597565, 12.397361);
  MoveTo_(38.665936, 13.718821);
  MoveTo_(39.470303, 16.304285);
  MoveTo_(38.306844, 20.202067);
  MoveTo_(32.083450, 21.991000);
  MoveTo_(30.412038, 25.250251);
  MoveTo_(30.647081, 27.746921);
  MoveTo_(31.728275, 29.439225);
  MoveTo_(37.178642, 30.768519);
  MoveTo_(38.906202, 32.276706);
  MoveTo_(39.376286, 36.217580);
  MoveTo_(37.694429, 38.850053);
  MoveTo_(35.239544, 39.476832);
  MoveTo_(24.184727, 39.294021);
  MoveTo_(21.876090, 37.596494);
  MoveTo_(20.965955, 32.805551);
  MoveTo_(19.414676, 30.966999);
  MoveTo_(15.093817, 30.579180);
  MoveTo_(12.586701, 32.694559);
  MoveTo_(12.632404, 46.377931);
  MoveTo_(14.512741, 48.195591);
  MoveTo_(19.790744, 49.389083);
  MoveTo_(21.279344, 52.522978);
  MoveTo_(20.045373, 56.640134);
  MoveTo_(14.195434, 58.186189);
  MoveTo_(12.367328, 60.672413);
  MoveTo_(12.367328, 72.837151);
  MoveTo_(13.546456, 74.893770);
  MoveTo_(16.017011, 75.813047);
  MoveTo_(20.125026, 74.747522);
  MoveTo_(21.991000, 68.447086);
  MoveTo_(25.250251, 66.775675);
  MoveTo_(27.746921, 67.010717);
  MoveTo_(29.439225, 68.091911);
  MoveTo_(30.768519, 73.542278);
  MoveTo_(32.276706, 75.269838);
  MoveTo_(36.085695, 75.762121);
  MoveTo_(38.729920, 74.246099);
  MoveTo_(39.476832, 71.603180);
  MoveTo_(39.104682, 59.967286);
  MoveTo_(37.287022, 58.086949);
  MoveTo_(32.083450, 56.973110);
  MoveTo_(30.474716, 54.267514);
  MoveTo_(31.207265, 50.082457);
  MoveTo_(34.633657, 48.578187);
  MoveTo_(53.713859, 48.593857);
  MoveTo_(57.031871, 50.360590);
  MoveTo_(57.652121, 53.568916);
  MoveTo_(56.711952, 56.326744);
  MoveTo_(55.019649, 57.407938);
  MoveTo_(50.559070, 58.186189);
  MoveTo_(48.793643, 60.426924);
  MoveTo_(48.605609, 72.038008);
  MoveTo_(50.265268, 75.154928);
  MoveTo_(53.271196, 75.844385);
  MoveTo_(56.154380, 75.029573);
  MoveTo_(57.370070, 73.317682);
  MoveTo_(58.416008, 68.354375);
  MoveTo_(61.758830, 66.766534);
  MoveTo_(64.226773, 67.048585);
  MoveTo_(65.802861, 68.091911);
  MoveTo_(67.132155, 73.542278);
  MoveTo_(68.740889, 75.323375);
  MoveTo_(72.710490, 75.715112);
  MoveTo_(75.269838, 73.960131);
  MoveTo_(75.844385, 71.453014);
  MoveTo_(75.468318, 59.967286);
  MoveTo_(73.650659, 58.086949);
  MoveTo_(68.176787, 56.781159);
  MoveTo_(66.754782, 53.118419);
  MoveTo_(68.176787, 49.455678);
  MoveTo_(73.650659, 48.149888);
  MoveTo_(75.422615, 46.377931);
  MoveTo_(75.468318, 32.694559);
  MoveTo_(72.961201, 30.579180);
  MoveTo_(68.640343, 30.966999);
  MoveTo_(67.089064, 32.805551);
  MoveTo_(66.122780, 37.694429);
  MoveTo_(63.746242, 39.324054);
  MoveTo_(52.667921, 39.470303);
  MoveTo_(49.994969, 38.599341);
  MoveTo_(48.619972, 35.814091);
  MoveTo_(49.204966, 32.178772);
  MoveTo_(50.987369, 30.725428);
  MoveTo_(56.154380, 29.575027);
  MoveTo_(57.658650, 26.148635);
  MoveTo_(57.329591, 23.714642);
  MoveTo_(56.241868, 22.182951);
  MoveTo_(50.876377, 20.922863);
  MoveTo_(49.095280, 19.314130);
  MoveTo_(48.760998, 15.093817);
  MoveTo_(50.767997, 12.632404);
  MoveTo_(64.559749, 12.632404);
  MoveTo_(66.331706, 14.404360);
  MoveTo_(67.637496, 19.878232);
  MoveTo_(71.300237, 21.300237);
  MoveTo_(74.962977, 19.878232);
  MoveTo_(76.268767, 14.404360);
  MoveTo_(78.149105, 12.586701);
  MoveTo_(89.784998, 12.214551);
  MoveTo_(92.427917, 12.961463);
  MoveTo_(93.963526, 15.740183);
  MoveTo_(93.336747, 19.607933);
  MoveTo_(91.499500, 21.006434);
  MoveTo_(86.628904, 21.991000);
  MoveTo_(84.983609, 24.968200);
  MoveTo_(85.157279, 36.719004);
  MoveTo_(86.536193, 38.729920);
  MoveTo_(89.179112, 39.476832);
  MoveTo_(92.929340, 38.306844);
  MoveTo_(94.718272, 32.083450);
  MoveTo_(97.977524, 30.412038);
  MoveTo_(100.474194, 30.647081);
  MoveTo_(102.166498, 31.728275);
  MoveTo_(103.495792, 37.178642);
  MoveTo_(105.003979, 38.906202);
  MoveTo_(108.944853, 39.376286);
  MoveTo_(111.577325, 37.694429);
  MoveTo_(112.204104, 35.239544);
  MoveTo_(112.051327, 24.308777);
  MoveTo_(110.421701, 21.932239);
  MoveTo_(105.532824, 20.965955);
  MoveTo_(103.694272, 19.414676);
  MoveTo_(103.306452, 15.093817);
  MoveTo_(105.532824, 12.543610);
  MoveTo_(119.105204, 12.632404);
  MoveTo_(120.922863, 14.512741);
  MoveTo_(122.052372, 19.700644);
  MoveTo_(125.107920, 21.267592);
  MoveTo_(129.292977, 20.125026);
  MoveTo_(130.966999, 14.094888);
  MoveTo_(133.526347, 12.339907);
  MoveTo_(145.809913, 12.430006);
  MoveTo_(147.882201, 13.901631);
  MoveTo_(148.572964, 16.754782);
  MoveTo_(147.397753, 20.202067);
  MoveTo_(141.174359, 21.991000);
  MoveTo_(139.502948, 25.250251);
  MoveTo_(139.737990, 27.746921);
  MoveTo_(140.819184, 29.439225);
  MoveTo_(146.269551, 30.768519);
  MoveTo_(147.997111, 32.276706);
  MoveTo_(148.467195, 36.217580);
  MoveTo_(146.880660, 38.791292);
  MoveTo_(144.180287, 39.480749);
  MoveTo_(132.694559, 39.104682);
  MoveTo_(130.814222, 37.287022);
  MoveTo_(129.700383, 32.083450);
  MoveTo_(126.994786, 30.474716);
  MoveTo_(122.809730, 31.207265);
  MoveTo_(121.305460, 34.633657);
  MoveTo_(121.321129, 53.713859);
  MoveTo_(123.087863, 57.031871);
  MoveTo_(126.296189, 57.652121);
  MoveTo_(129.054017, 56.711952);
  MoveTo_(130.135211, 55.019649);
  MoveTo_(130.913462, 50.559070);
  MoveTo_(133.154197, 48.793643);
  MoveTo_(144.765281, 48.605609);
  MoveTo_(147.882201, 50.265268);
  MoveTo_(148.571658, 53.271196);
  MoveTo_(147.756845, 56.154380);
  MoveTo_(146.044955, 57.370070);
  MoveTo_(141.081648, 58.416008);
  MoveTo_(139.493807, 61.758830);
  MoveTo_(139.775858, 64.226773);
 

 
}