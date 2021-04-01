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
void WeightUpdateB (float , float , float , float , float[] , float[], float *, bool *)

void SpeedSaturation1(float * , float , int *, int *);

#endif