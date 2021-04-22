#ifndef _AUX_GUARD
#define _AUX_GUARD

#include <iostream>
#include <fstream>
#include <cstring>
#include <chrono>
#include "templates.hpp"

void WeightUpdateR1(float , float , float , float [], float * , bool *);
void WeightUpdateR2(float , float , float , float [], float * , bool *);
void WeightUpdateR3(float , float , float , float [], float * , bool *);
void WeightUpdateB1(float , float , float , float , float[] , float[], float *, bool *);
void WeightUpdateB2(float , float , float , float , float[] , float[], float *, bool *);
void WeightUpdateB3(float , float , float , float , float[] , float[], float *, bool *);
void WeightUpdateB4(float , float , float , float , float[] , float[], float *, bool *);

void SpeedSaturation1(float * , float , int *, int *);

void PrintWeight();


float * ExtraCalculation(float, float, int [], float [], float [], float , int []);
int CheckQuadrant(float, float);
void WeightUpdateRobot(float , float, float [], float [], float, int, float *);
void InitialFilter(float *, float *, matrix_hal::IMUData, matrix_hal::GPIOControl, matrix_hal::IMUSensor, float, float);
void PrintWeight(float [], float []);

#endif