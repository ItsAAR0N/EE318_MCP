// TEST.cpp : This file contains the 'main' function. Program execution begins and ends there.
//


#include <iostream>
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
void GCode_Interpret_(char* buffer)
{
    // Typical G-Code command as follows: G## X## Y## Z## F##
    char* ptr; // pointer used for string parsing
    int x, y, z; // variables to store X, Y, and Z values

    // extract X value
    ptr = strstr(buffer, "X"); // find the "X" character in the string
    x = atoi(ptr + 1); // convert the substring after "X" to an integer

    // extract Y value
    ptr = strstr(buffer, "Y");
    y = atoi(ptr + 1);

    // extract Z value
    ptr = strstr(buffer, "Z");
    z = atoi(ptr + 1);

    printf("X: %d\nY: %d\nZ: %d\n", x, y, z);
}
int main()
{
    std::cout << "Hello World!\n";
    char input[] = "X30 Y20 Z200";
    GCode_Interpret_(const_cast<char*>(input));
}

// Run program: Ctrl + F5 or Debug > Start Without Debugging menu
// Debug program: F5 or Debug > Start Debugging menu

// Tips for Getting Started: 
//   1. Use the Solution Explorer window to add/manage files
//   2. Use the Team Explorer window to connect to source control
//   3. Use the Output window to see build output and other messages
//   4. Use the Error List window to view errors
//   5. Go to Project > Add New Item to create new code files, or Project > Add Existing Item to add existing code files to the project
//   6. In the future, to open this project again, go to File > Open > Project and select the .sln file
