// C file for translator

/********************************
transltr.c
Translator
Aaron Shek, @ 2023 University of Strathclyde
*********************************/

#include "transltr.h"
#include <msp430.h>
#include <math.h>
#include <driverlib.h>

#define PI 3.14159265358979323846

const float offset_1; 
const float offset_2;
const float space_dim_Y = 0;
const float length_arm_L = 0;

float x_coord;
float y_coord;

struct invK_vals {
  float dist_1; 
  float ang_1;  
  float ang_2; 
  float dist_2; 
};

struct invK_vals calc_invK_(float x_coord, float y_coord)
{ 
  struct invK_vals invK_out;
  if (x_coord < offset_1) {
  // Left arm if on LHS of plane
  invK_out.dist_1 = sqrt((offset_1 - x_coord)*(offset_1 - x_coord) + (space_dim_Y - y_coord));
  invK_out.ang_1 = PI + atan((offset_1 - x_coord)/(space_dim_Y - y_coord)) + acos(invK_out.dist_1/(2 * length_arm_L));
  
  // Right arm if on LHS of plane
  invK_out.dist_2 = sqrt((offset_2 - x_coord)*(offset_2 - x_coord) + (space_dim_Y - y_coord));
  invK_out.ang_2 = PI + atan((offset_2 - x_coord)/(space_dim_Y - y_coord)) + acos(invK_out.dist_2/(2 * length_arm_L)); 
  return invK_out;
  } else {
    
  // Left arm if on RHS of plane  
  invK_out.dist_1 = sqrt((x_coord - offset_1)*(x_coord - offset_2) + (space_dim_Y - y_coord));
  invK_out.ang_1 = PI + atan((offset_1 - x_coord)/(space_dim_Y - y_coord)) + acos(invK_out.dist_1/(2 * length_arm_L));
  
  // Right arm if on RHS of plane
  invK_out.dist_2 = sqrt((x_coord - offset_2)*(x_coord - offset_2) + (space_dim_Y - y_coord));
  invK_out.ang_2 = PI + atan((x_coord - offset_2)/(space_dim_Y - y_coord)) + acos(invK_out.dist_2/(2 * length_arm_L));  
  return invK_out;
  }
}
// If we want to access it
// struct invK_vals result;
// result = calc_invK_(x_coord, y_coord);
// float dist_1 = result.dist_1;
// float ang_1 = result.ang_1;
// float ang_2 = result.ang_2;
// float dist_2 = result.dist_2;

