// Testing.cpp : This file contains the 'main' function. Program execution begins and ends there.
//
#include <math.h>
#include <iostream>
#include <vector>
#include <map>

using namespace std;

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
        invK_out.dist_1 = sqrt((offset_1 - x_coord) * (offset_1 - x_coord) + pow((space_dim_Y - y_coord),2));
        invK_out.ang_1 = PI + atan((offset_1 - x_coord) / (space_dim_Y - y_coord)) + acos(invK_out.dist_1 / (2 * length_arm_L));

        // Right arm if on LHS of plane
        invK_out.dist_2 = sqrt((offset_2 - x_coord) * (offset_2 - x_coord) + pow((space_dim_Y - y_coord),2));
        invK_out.ang_2 = PI + atan((offset_2 - x_coord) / (space_dim_Y - y_coord)) + acos(invK_out.dist_2 / (2 * length_arm_L));
        return invK_out;
    }
    else {

        // Left arm if on RHS of plane  
        invK_out.dist_1 = sqrt((x_coord - offset_2) * (x_coord - offset_2) + pow((space_dim_Y - y_coord),2));
        invK_out.ang_1 = PI + atan((offset_1 - x_coord) / (space_dim_Y - y_coord)) + acos(invK_out.dist_1 / (2 * length_arm_L));

        // Right arm if on RHS of plane
        invK_out.dist_2 = sqrt((x_coord - offset_2) * (x_coord - offset_2) + pow((space_dim_Y - y_coord),2));
        invK_out.ang_2 = PI + atan((x_coord - offset_2) / (space_dim_Y - y_coord)) + acos(invK_out.dist_2 / (2 * length_arm_L));
        return invK_out;
    }
}

int main()
{
    for (float x = 0; x <= 100; x += 10) {
        for (float y = 0; y <= 100; y += 10) {
            invK_vals invK = calc_invK_(x, y);
            cout << "x_coord = " << x << ", y_coord = " << y << endl;
            cout << "dist_1 = " << invK.dist_1 << ", ang_1 = " << invK.ang_1 * 180 / PI << ", ang_2 = " << invK.ang_2 * 180 / PI << ", dist_2 = " << invK.dist_2 << endl << endl;
        }
    }


}
