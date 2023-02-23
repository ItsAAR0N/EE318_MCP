// Motor header file 

/********************************
motor.h
Default Template file for testing purposes
Aaron Shek, @ 2023 University of Strathclyde
*********************************/

#ifndef HEADER_H
#define HEADER_H

extern void initialiseGPIOs_(); 
extern void initialiseTimer_();
extern void initialiseADCpot_();
extern void readValue_pot_();
extern void stepMotor_(int n_Steps, char dir);
extern void CB2buttonAdjust_m_(int SW1_interruptFlag, int SW2_interruptFlag);
#endif