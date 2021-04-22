#ifndef _AUX_GUARD
#define _AUX_GUARD

#include <iostream>
#include <fstream>
#include <cstring>
#include <chrono>
#include "templates.hpp"

// Interfaces with IMU sensor
#include "matrix_hal/imu_sensor.h"
// Holds data from IMU sensor
#include "matrix_hal/imu_data.h"
// Communicates with MATRIX device
#include "matrix_hal/matrixio_bus.h"

void WeightUpdateR1(float , float , float , float [], float * , bool *);
void WeightUpdateR2(float , float , float , float [], float * , bool *);
void WeightUpdateR3(float , float , float , float [], float * , bool *);
void WeightUpdateB1(float , float , float , float , float[] , float[], float *, bool *);
void WeightUpdateB2(float , float , float , float , float[] , float[], float *, bool *);
void WeightUpdateB3(float , float , float , float , float[] , float[], float *, bool *);
void WeightUpdateB4(float , float , float , float , float[] , float[], float *, bool *);

void SpeedSaturation1(float * , float , int *, int *);

void PrintWeight();
void InitialFilter(float * , float *, matrix_hal::IMUData, matrix_hal::GPIOControl , matrix_hal::IMUSensor, float, float);

#endif