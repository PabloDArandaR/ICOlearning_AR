#include <iostream>
#include <fstream>
#include <cstring>
#include <chrono>
#include <thread>
#include "speed.hpp"
#include "training_functions.hpp"

// Interfaces with IMU sensor
#include "matrix_hal/imu_sensor.h"
// Holds data from IMU sensor
#include "matrix_hal/imu_data.h"
// Communicates with MATRIX device
#include "matrix_hal/matrixio_bus.h"
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Running function
void RunRobot(float weight_roll[], float weight_pitch[] ,Motor left, Motor right, matrix_hal::IMUData & imu_data, matrix_hal::GPIOControl & gpio, matrix_hal::IMUSensor imu_sensor, float sampling_time, float cutoff, int speed[], float limit, float threshold)
{
    ////////////////////////////////////////////////////////////////////////
    // Initialization of variables
    int duration = 10000;
    auto begin = std::chrono::high_resolution_clock::now();
    auto end = std::chrono::high_resolution_clock::now();
    float extra [2];
    float roll {0}, pitch {0}, reflex {0};
    int dir[2];
    float roll_original{0};
    float pitch_original{0};
    std::fstream file;
    file.open("evolution_run.csv", std::ios_base::app);

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// Initialize the filter

    InitialFilter(&roll, &pitch, imu_data, gpio, imu_sensor, sampling_time, cutoff);

    // This 2 values will be used to determine the bias, supposing with this that the robot is starting with both real angles = 0
    roll_original = roll;
    pitch_original = pitch;
    dir[0] = 0;
    dir[1] = 0;

    // Initialize the timing variables
    begin = std::chrono::high_resolution_clock::now();
    end = std::chrono::high_resolution_clock::now();

    while(true)
    {

        if (std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() > 10000)
        {
            left.setMotorSpeedDirection(&gpio, 0 , dir[0]);
            right.setMotorSpeedDirection(&gpio, 0 , dir[1]);
            break;
        }

        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Initialize variables:
        dir[0] = 0;
        dir[1] = 0;

        /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /// Remeasure the value of the relevant variables

        // Overwrites imu_data with new data from IMU sensor
        imu_sensor.Read(&imu_data);

        roll = LowPassFilter(sampling_time/1000.0f, cutoff, roll, imu_data.roll);
        pitch = LowPassFilter(sampling_time/1000.0f, cutoff, pitch, imu_data.pitch);
        reflex = pitch + roll;


        /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /// Recalculate the actions

        if ((abs(roll - roll_original) < 2) && (abs(pitch) < 2))
        {
            extra[0] = 0;
            extra[1] = 0;
        }
        else
        {
            extra[0] = ExtraL(pitch, roll-roll_original, speed, weight_roll, weight_pitch, limit, dir);
            if (extra[0] < 0)
            {
                extra[0] = - extra[0];
                dir[0] = 1;
            }
            extra[1] = ExtraR(pitch, roll-roll_original, speed, weight_roll, weight_pitch, limit, dir);
            if (extra[1] < 0)
            {
                extra[1] = - extra[1];
                dir[1] = 1;
            }
        }

        /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /// Apply the actions

        left.setMotorSpeedDirection(&gpio, speed[0] + extra[0], dir[0]);
        right.setMotorSpeedDirection(&gpio, speed[1] + extra[1], dir[1]);

        /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /// Write in file and set timer 

        std::cout << "Value of roll:  " << roll << std::endl;
        std::cout << "Value of pitch: " << pitch << std::endl;
        std::cout << "------------------------------------------------------------" << std::endl;
       
        file << weight_roll[0] << "," << weight_roll[1] << "," << weight_pitch[0] << "," << weight_pitch[1] << "," << weight_pitch[2] << "," << weight_pitch[3] << "," << imu_data.roll << "," << roll << "," << imu_data.pitch << "," << pitch << "," << speed[0]+extra[0] << "," << speed[1]+extra[1] << "," << reflex << std::endl;        

        end = std::chrono::high_resolution_clock::now();
    }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Training function
void RunRobot2(Motor left, Motor right, matrix_hal::IMUData & imu_data, float weight_roll[], float weight_pitch[], float learning_rate, int speed[], matrix_hal::GPIOControl gpio, matrix_hal::IMUSensor imu_sensor, float limit, float sampling_time, float cutoff, int * iteration, float threshold)
{
    //Variables required for the different calculations:
    float roll, pitch, reflex {0}, extra [2], bias_roll;
    int dir[2], quadrant;
    bool reflex_ON {false};
    std::ofstream file;
    auto finish = std::chrono::high_resolution_clock::now();
    auto start = std::chrono::high_resolution_clock::now();
    auto beginning = std::chrono::high_resolution_clock::now();
    file.open("evolution_both.csv", std::ios_base::app);

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Increase the value of the iteration variable to acknowledge how many iterations have been accomplished
    *iteration++;

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Initialize filter

    InitialFilter(&roll, &pitch, imu_data, gpio, imu_sensor, sampling_time, cutoff);
    bias_roll = roll;

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Training loop

    while(true)
    {

        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Working condition:

        if (std::chrono::duration_cast<std::chrono::milliseconds>(finish - start).count() > 10000)
        {
            left.setMotorSpeedDirection(&gpio, 0 , dir[0]);
            right.setMotorSpeedDirection(&gpio, 0 , dir[1]);
            break;
        }

        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Initialize variables:
        dir[0] = 0;
        dir[1] = 0;
        
        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Measure the new values of the variable

        // Overwrites imu_data with new data from IMU sensor
        imu_sensor.Read(&imu_data);

        roll = LowPassFilter(sampling_time/1000.0f, cutoff , roll,imu_data.roll);
        pitch = LowPassFilter(sampling_time/1000.0f, cutoff, pitch, imu_data.pitch);
        
        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Calculate the new speed

        extra[0] = ExtraL(pitch, roll - bias_roll, speed, weight_roll, weight_pitch, limit, dir);
        extra[1] = ExtraR(pitch, roll - bias_roll, speed, weight_roll, weight_pitch, limit, dir);
        SpeedSaturation1(extra, limit, speed, dir);

        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Apply the action

        left.setMotorSpeedDirection(&gpio, speed[0] + extra[0], dir[0]);
        right.setMotorSpeedDirection(&gpio, speed[1] + extra[1], dir[1]);

        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Write in file

        std::cout << "Value of roll:  " << roll << std::endl;
        std::cout << "Value of pitch: " << pitch << std::endl;
        std::cout << "------------------------------------------------------------" << std::endl;
        file << weight_roll[0] << "," << weight_roll[1] << "," << weight_pitch[0] << "," << weight_pitch[1] << "," << weight_pitch[2] << "," << weight_pitch[3] << "," << imu_data.roll << "," << roll - bias_roll << "," << imu_data.pitch << "," << pitch << "," << speed[0]+extra[0] << "," << speed[1]+extra[1] << "," << reflex << std::endl;   
        
        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Assuring sampling time

        finish = std::chrono::high_resolution_clock::now();

        std::this_thread::sleep_for( std::chrono::milliseconds((int)sampling_time) - std::chrono::duration_cast<std::chrono::milliseconds>(finish - start));
    }
    file.close();
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Training function
void TrainBothRobot(Motor left, Motor right, matrix_hal::IMUData & imu_data, float weight_roll[], float weight_pitch[], float learning_rate, int speed[], matrix_hal::GPIOControl gpio, matrix_hal::IMUSensor imu_sensor, float limit, float sampling_time, float cutoff, int * iteration, float threshold)
{
    //Variables required for the different calculations:
    float roll, pitch, reflex {0}, extra [2], bias_roll;
    int dir[2], quadrant;
    bool reflex_ON {false};
    std::ofstream file;
    auto finish = std::chrono::high_resolution_clock::now();
    auto start = std::chrono::high_resolution_clock::now();
    auto beginning = std::chrono::high_resolution_clock::now();
    file.open("evolution_both.csv", std::ios_base::app);

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Increase the value of the iteration variable to acknowledge how many iterations have been accomplished
    *iteration++;

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Initialize filter

    InitialFilter(&roll, &pitch, imu_data, gpio, imu_sensor, sampling_time, cutoff);
    bias_roll = roll;

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Training loop

    while(true)
    {

        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Working condition:

        if (std::chrono::duration_cast<std::chrono::milliseconds>(finish - start).count() > 10000)
        {
            left.setMotorSpeedDirection(&gpio, 0 , dir[0]);
            right.setMotorSpeedDirection(&gpio, 0 , dir[1]);
            break;
        }

        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Initialize variables:
        dir[0] = 0;
        dir[1] = 0;
        
        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Measure the new values of the variable

        // Overwrites imu_data with new data from IMU sensor
        imu_sensor.Read(&imu_data);

        roll = LowPassFilter(sampling_time/1000.0f, cutoff , roll,imu_data.roll);
        pitch = LowPassFilter(sampling_time/1000.0f, cutoff, pitch, imu_data.pitch);

        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Check the quadrant

        quadrant = CheckQuadrant(pitch, roll - bias_roll);

        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Update the weight

        WeightUpdateRobot(roll - bias_roll,  pitch, weight_roll, weight_pitch, learning_rate, quadrant, &reflex);
        
        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Calculate the new speed

        extra[0] = ExtraL(pitch, roll - bias_roll, speed, weight_roll, weight_pitch, limit, dir);
        extra[1] = ExtraR(pitch, roll - bias_roll, speed, weight_roll, weight_pitch, limit, dir);
        SpeedSaturation1(extra, limit, speed, dir);

        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Apply the action

        left.setMotorSpeedDirection(&gpio, speed[0] + extra[0], dir[0]);
        right.setMotorSpeedDirection(&gpio, speed[1] + extra[1], dir[1]);

        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Write in file

        std::cout << "Value of roll:  " << roll << std::endl;
        std::cout << "Value of pitch: " << pitch << std::endl;
        std::cout << "------------------------------------------------------------" << std::endl;
        file << weight_roll[0] << "," << weight_roll[1] << "," << weight_pitch[0] << "," << weight_pitch[1] << "," << weight_pitch[2] << "," << weight_pitch[3] << "," << imu_data.roll << "," << roll - bias_roll << "," << imu_data.pitch << "," << pitch << "," << speed[0]+extra[0] << "," << speed[1]+extra[1] << "," << reflex << std::endl;   
        
        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Assuring sampling time

        finish = std::chrono::high_resolution_clock::now();

        std::this_thread::sleep_for( std::chrono::milliseconds((int)sampling_time) - std::chrono::duration_cast<std::chrono::milliseconds>(finish - start));
    }
    file.close();
}

