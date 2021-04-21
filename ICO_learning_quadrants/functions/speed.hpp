#include <iostream>

// Interfaces with IMU sensor
#include "matrix_hal/imu_sensor.h"
// Holds data from IMU sensor
#include "matrix_hal/imu_data.h"
// Communicates with MATRIX device
#include "matrix_hal/matrixio_bus.h"

float * ExtraCalculation(float, float, int [], float [], float [], float , int []);
int CheckQuadrant(float, float);
void WeightUpdateRobot(float , float, float [], float [], float, int, float *);