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

extern void initUART_();
extern void initGPIO_();
extern void initClockTo16MHz_();
extern void GCode_Interpret_(char* buffer);
extern InvKVals calc_invK(float x_coord, float y_coord);
extern long mapRange_(long x, long in_min, long in_max, long out_min, long out_max);
#endif