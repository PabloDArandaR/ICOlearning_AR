#include <iostream>
#include <fstream>
#include <cstring>
#include <chrono>
#include <thread>
#include "aux.cpp"
#include "../../motor_control/motor_class.hpp"

// Interfaces with GPIO
#include "matrix_hal/gpio_control.h"
// Interfaces with IMU sensor
#include "matrix_hal/imu_sensor.h"
// Holds data from IMU sensor
#include "matrix_hal/imu_data.h"
// Communicates with MATRIX device
#include "matrix_hal/matrixio_bus.h"
// Interfaces with Everloop
#include "matrix_hal/everloop.h"
// Holds data for Everloop
#include "matrix_hal/everloop_image.h"

using namespace std::chrono_literals;

// GPIO via Matrix Creator
#define  TB6612_LEFT_MOTOR_PWMB        15 // (Orange)
#define  TB6612_RIGHT_MOTOR_PWMA         14 // (Green)
#define  TB6612_LEFT_MOTOR_BIN1        13 // (Blue)
#define  TB6612_LEFT_MOTOR_BIN2        12 // (Brown)
#define  TB6612_RIGHT_MOTOR_AIN1         11 // (Grey)
#define  TB6612_RIGHT_MOTOR_AIN2         10 // (Pink)

int main()
{
    // Create MatrixIOBus object for hardware communication
	matrix_hal::MatrixIOBus bus;
    
    // Initialize bus and exit program if error occurs
    if (!bus.Init())
    {
	    return false;
    }
    
    // Create GPIOControl object
	matrix_hal::GPIOControl gpio;
    // Create IMUData object
    matrix_hal::IMUData imu_data;
    // Create IMUSensor object
    matrix_hal::IMUSensor imu_sensor;
    // Create Everloop object for LEDs activation
    matrix_hal::Everloop everloop;
    
	// Set gpio to use MatrixIOBus bus
	gpio.Setup(&bus);
    // Set imu_sensor to use MatrixIOBus bus
    imu_sensor.Setup(&bus);
    // Set everloop to use MatrixIOBus bus
    everloop.Setup(&bus);
    // Overwrites imu_data with new data from IMU sensor
    imu_sensor.Read(&imu_data);
    // Holds the number of LEDs on MATRIX device
    int ledCount = bus.MatrixLeds();
    // Create EverloopImage object, with size of ledCount
    matrix_hal::EverloopImage everloop_image(ledCount);


    // The error for applying speed could be in the way the speed has been calculated
    Motor left(TB6612_LEFT_MOTOR_PWMB, TB6612_LEFT_MOTOR_BIN1, TB6612_LEFT_MOTOR_BIN2, &gpio);
    Motor right(TB6612_RIGHT_MOTOR_PWMA, TB6612_RIGHT_MOTOR_AIN1, TB6612_RIGHT_MOTOR_AIN2, &gpio);


    left.setMotorSpeedDirection(&gpio, 0, 0);
    right.setMotorSpeedDirection(&gpio, 0, 0);

    return 0;
}