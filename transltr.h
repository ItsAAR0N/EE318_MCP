// Translator header file 

/********************************
transltr.h
Default Template file for testing purposes
Aaron Shek, @ 2023 University of Strathclyde
*********************************/

#ifndef TRANSLTR_H
#define TRANSLTR_H

typedef struct InvKVals {
  float dist_1;
  float ang_1;
  float dist_2;
  float ang_2;
} InvKVals;

#define MAX_COMMANDS 200
extern float gcodeBuffer[MAX_COMMANDS][2];
extern volatile uint8_t gcodeIndex;

extern void initUART_();
extern void initGPIO_();
extern void initClockTo8MHz_();
extern void initialiseTimerMicros_();
extern long Micros_();
extern void processUARTinstr_(char* buffer);
extern void clearExecutedCommands();
extern InvKVals calc_invK(float x_coord, float y_coord);
extern void MoveTo_(float x_coord, float y_coord);
extern long calculateDelay_(long Steps1, long Steps2);
extern void calculateDelays_(long Steps1, long Steps2);
extern int min(int a, int b); 
extern int max(int a, int b);
extern long mapRange_(long x, long in_min, long in_max, long out_min, long out_max);
extern void abc();
extern void radials();
extern void target();
extern void Star_();
extern void Spiral_();
extern void Pentagon_();
extern void Circle_();
extern void UoS_();
extern void HilbertCurve_();
#endif