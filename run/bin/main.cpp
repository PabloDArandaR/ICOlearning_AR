#include <iostream>
#include <fstream>
#include <cstring>
#include <chrono>
#include <thread>
#include "aux.cpp"

// Interfaces with IMU sensor
#include "matrix_hal/imu_sensor.h"
// Holds data from IMU sensor
#include "matrix_hal/imu_data.h"
// Communicates with MATRIX device
#include "matrix_hal/matrixio_bus.h"


#include "../cpp/driver/everloop.h"
#include "../cpp/driver/everloop_image.h"

using namespace std::chrono_literals;

// GPIO via Matrix Creator
#define  TB6612_LEFT_MOTOR_PWMB        15 // (Orange)
#define  TB6612_RIGHT_MOTOR_PWMA         14 // (Green)
#define  TB6612_LEFT_MOTOR_BIN1        13 // (Blue)
#define  TB6612_LEFT_MOTOR_BIN2        12 // (Brown)
#define  TB6612_RIGHT_MOTOR_AIN1         11 // (Grey)
#define  TB6612_RIGHT_MOTOR_AIN2         10 // (Pink)



int main(int argc, char* argv[]) {

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///// Declaring global variables
    float roll, pitch, threshold, reflex;                         // Variables related to some learning parameters
    float speed[2], extra[2];                                                   // Stores the value of base speed
    float weight_roll[2], weight_pitch[4];                                  // Stores the weights related to each one of the signals taken into consideration
    int   dir[2];                                       // Selection of the update function and number of training sessions done
    float sampling_time, cutoff, limit {100.0f};                                    // Sampling time and cutoff frequency. Necessary for the Low Pass Filter

    auto begin = std::chrono::high_resolution_clock::now();         // Beginning of the program
    auto end = std::chrono::high_resolution_clock::now();           // Final of the iteration

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Initialization of Matrix Creator related variables and initialization

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

    everloop_image.leds[34 % everloop_image.leds.size()].red     = 100;

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///// Initialize the variables

    // Pitch, Roll Output
    pitch = imu_data.pitch;
    roll = imu_data.roll;

    ////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Weights:

    //This file will have the format:
    // Weight roll
    // Weight pitch front
    // Weight pitch back
    readWeights(argv[1], weight_roll, weight_pitch);


    ////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Introduce cutoff frequency
    cutoff = std::stof(argv[2]);
    ////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Introduce Sampling time
    sampling_time = std::stof(argv[3]);
    ////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Introduce Threshold
    threshold = std::stof(argv[4]);
    ////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Speeds and learning rate
    speed[0] = std::stof(argv[5]);
    speed[1] = std::stof(argv[5]);
    
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///// Declaring the motor variables (more info on the motor_control.cpp file)
    /// Ports in order: PWM signal, input 1, input 2, gpio address of the Matrix Creator

    // The error for applying speed could be in the way the speed has been calculated
    Motor left(TB6612_LEFT_MOTOR_PWMB, TB6612_LEFT_MOTOR_BIN1, TB6612_LEFT_MOTOR_BIN2, &gpio);
    Motor right(TB6612_RIGHT_MOTOR_PWMA, TB6612_RIGHT_MOTOR_AIN1, TB6612_RIGHT_MOTOR_AIN2, &gpio);

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///// Initialize roll and pitch values

    InitialFilter(roll, pitch, imu_data, &gpio, imu_sensor, sampling_time, cutoff);

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///// Beginning of the training and running

    end = std::chrono::high_resolution_clock::now();

    while (true){

        begin = std::chrono::high_resolution_clock::now();

        ///////////////////////////////////////////////////////////////////////////////////////////////////
        // Sample new roll and pitch


        // Overwrites imu_data with new data from IMU sensor
        imu_sensor.Read(&imu_data);

        roll = LowPassFilter(sampling_time/1000.0f, cutoff, roll, imu_data.roll);
        pitch = LowPassFilter(sampling_time/1000.0f, cutoff, pitch, imu_data.pitch);
        reflex = pitch + roll;

        ///////////////////////////////////////////////////////////////////////////////////////////////////
        // Calculate extra for each side

        if ((abs(pitch) > threshold) && (abs(roll) > threshold))
        {
            extra[0] = ExtraL(pitch, roll, speed, weight_roll, weight_pitch, limit, dir);
            extra[1] = ExtraR(pitch, roll, speed, weight_roll, weight_pitch, limit, dir);
            SpeedSaturation1(extra, limit, speed, dir);
        }
        else
        {
            extra[0] = 0;
            extra[1] = 0;
        }

        ///////////////////////////////////////////////////////////////////////////////////////////////////
        // Apply action

        std::cout << "Speeds are:     " << speed[0] + extra[0] << "  " << speed[1] + extra[1] <<std::endl;

        left.setMotorSpeedDirection(&gpio, speed[0] + extra[0], dir[0]);
        right.setMotorSpeedDirection(&gpio, speed[1] + extra[1], dir[1]);

        ///////////////////////////////////////////////////////////////////////////////////////////////////
        // Wait for sampling

        std::this_thread::sleep_for( std::chrono::milliseconds((int)sampling_time) - std::chrono::duration_cast<std::chrono::milliseconds>(end - std::chrono::high_resolution_clock::now()));

    }
}


