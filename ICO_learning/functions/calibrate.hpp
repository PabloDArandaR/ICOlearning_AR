#include <iostream>
#include <fstream>
#include <cstring>
#include <chrono>
#include "templates.hpp"
#include "variety.hpp"

// Interfaces with IMU sensor
#include "matrix_hal/imu_sensor.h"
// Holds data from IMU sensor
#include "matrix_hal/imu_data.h"
// Communicates with MATRIX device
#include "matrix_hal/matrixio_bus.h"

float BiasRoll(matrix_hal::IMUData & imu_data, matrix_hal::GPIOControl, matrix_hal::IMUSensor, int);

float BiasPitch(matrix_hal::IMUData & imu_data, matrix_hal::GPIOControl, matrix_hal::IMUSensor, int);