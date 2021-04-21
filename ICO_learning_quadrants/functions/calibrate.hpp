#ifndef _calibrate_GUARD
#define _calibrate_GUARD

#include <iostream>
#include <fstream>
#include <cstring>
#include <chrono>
#include "templates.hpp"
#include "variety.hpp"

float BiasRoll(matrix_hal::IMUData & imu_data, matrix_hal::GPIOControl, matrix_hal::IMUSensor, int, float, float);

float BiasPitch(matrix_hal::IMUData & imu_data, matrix_hal::GPIOControl, matrix_hal::IMUSensor, int, float, float);

float LowPassFilter(float, float, float, float);
#endif