#ifndef _AUX_GUARD
#define _AUX_GUARD

#include <iostream>
#include <fstream>
#include <cstring>
#include <chrono>
#include "templates.hpp"

void WeightUpdate1(float , float , float , float , float * , bool *);
void WeightUpdate2(float , float , float , float , float * , bool *);
void WeightUpdate3(float , float , float , float , float * , bool *);

void SpeedSaturation1(float * , float , const int , int );
void SpeedSaturation2(float * , float , const int , int );

#endif