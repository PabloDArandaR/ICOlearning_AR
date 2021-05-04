#include <iostream>
#include <fstream>
#include <cstring>
#include <chrono>
#include <thread>
#include "speed.hpp"

// Interfaces with IMU sensor
#include "matrix_hal/imu_sensor.h"
// Holds data from IMU sensor
#include "matrix_hal/imu_data.h"
// Communicates with MATRIX device
#include "matrix_hal/matrixio_bus.h"

void RunRobot(float [], float [] ,Motor , Motor , matrix_hal::IMUData &, matrix_hal::GPIOControl &, matrix_hal::IMUSensor, float, float, int [], float, float);

void TrainBothRobot(Motor , Motor, matrix_hal::IMUData & , float [], float [], float, int [], matrix_hal::GPIOControl, matrix_hal::IMUSensor, float, float, float, int *, float);