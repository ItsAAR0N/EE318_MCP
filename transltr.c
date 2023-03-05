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
#include "transltr.h"

#define PI 3.14159265358979323846

#define WIDTH 320
#define SPACING 110
#define ARM 190

const float offset_1 = (WIDTH / 2) - (SPACING / 2); // M1 offset
const float offset_2 = (WIDTH / 2) - (SPACING / 2) + SPACING; // M2 
const float space_dim_Y = 350; // Length of sheet
const float length_arm_L = ARM;

float x_coord;
float y_coord;

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
// https://techexplorations.com/guides/arduino/programming/map-function/
long map(long x, long in_min, long in_max, long out_min, long out_max) { 
  return (x - in_min)*(out_max - out_min)/(in_max - in_min) + out_min;
}
