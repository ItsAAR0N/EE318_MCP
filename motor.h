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
extern int accel_Val_();
extern void delay_us(unsigned long delay);
extern void stepMotor_(int n_Steps, bool dir, int motor_Num);
extern void stepMotor1Basic_(bool DIR_1);
extern void stepMotor2Basic_(bool DIR_2);
extern void penManualControl_();
extern void penUp_();
extern void penDown_();
extern void CB2buttonAdjust_m_(int SW1_interruptFlag, int SW2_interruptFlag);
extern float pos_Accel_(float waitTime);
extern float neg_Accel_(float waitTime);
#endif