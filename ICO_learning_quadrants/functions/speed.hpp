#include <iostream>
#include "../../motor_control/motor_class.hpp"

// Interfaces with IMU sensor
#include "matrix_hal/imu_sensor.h"
// Holds data from IMU sensor
#include "matrix_hal/imu_data.h"
// Communicates with MATRIX device
#include "matrix_hal/matrixio_bus.h"

float * ExtraCalculation(float, float, int [], float [], float [], float , int []);
int CheckQuadrant(float, float);
void WeightUpdateRobot(float , float, float [], float [], float, int, float *);
void InitialFilter(float *, float *, matrix_hal::IMUData, matrix_hal::GPIOControl, matrix_hal::IMUSensor, float, float);
void PrintWeight(float [], float []);
float LowPassFilter(float, float, float, float);