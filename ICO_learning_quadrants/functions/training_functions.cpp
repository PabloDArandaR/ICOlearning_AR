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

void RunRobot(float weight_roll[], float weight_pitch[] ,Motor left, Motor right, matrix_hal::IMUData & imu_data, matrix_hal::GPIOControl & gpio, matrix_hal::IMUSensor imu_sensor, float sampling_time, float cutoff, int speed[], float limit)
{
    ////////////////////////////////////////////////////////////////////////
    // Initialization of variables
    int duration = 10000;
    auto begin = std::chrono::high_resolution_clock::now();
    auto end = std::chrono::high_resolution_clock::now();
    float * extra;
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

    // Initialize the timing variables
    begin = std::chrono::high_resolution_clock::now();
    end = std::chrono::high_resolution_clock::now();

    while(std::chrono::duration_cast<std::chrono::milliseconds>(end-begin).count() > duration)
    {
        /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /// Remeasure the value of the relevant variables

        // Overwrites imu_data with new data from IMU sensor
        imu_sensor.Read(&imu_data);

        roll = LowPassFilter(sampling_time/1000.0f, cutoff, roll, imu_data.roll);
        pitch = LowPassFilter(sampling_time/1000.0f, cutoff, pitch, imu_data.pitch);
        reflex = pitch + roll;

        /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /// Recalculate the actions

        float * ExtraCalculation(float, float, int [], float [], float [], float , int []);
        extra = ExtraCalculation(pitch, roll, speed, weight_roll, weight_pitch, limit, dir);

        /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /// Apply the actions

        if ((abs(pitch) > 1) | (abs(roll) > 1))
        {
            left.setMotorSpeedDirection(&gpio, speed[0] + extra[0], dir[0]);
            right.setMotorSpeedDirection(&gpio, speed[1] + extra[1], dir[1]);
        }
        else
        {
            left.setMotorSpeedDirection(&gpio, speed[0] , dir[0]);
            right.setMotorSpeedDirection(&gpio, speed[1] , dir[1]);
        }

        /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /// Write in file and set timer 

        file << weight_roll[0] << "," << weight_roll[1] << "," << weight_pitch[0] << "," << weight_pitch[1] << "," << weight_pitch[2] << "," << weight_pitch[3] << "," << imu_data.roll << "," << roll << "," << imu_data.pitch << "," << pitch << "," << speed[0]+extra[0] << "," << speed[1]+extra[1] << "," << reflex << std::endl;        

        end = std::chrono::high_resolution_clock::now();
    }
}

void TrainBothRobot(Motor left, Motor right, matrix_hal::IMUData & imu_data, float weight_roll[], float weight_pitch[], float learning_rate, int speed[], matrix_hal::GPIOControl gpio, matrix_hal::IMUSensor imu_sensor, float limit, float sampling_time, float cutoff, int * iteration)
{
    //Variables required for the different calculations:
    float bias_roll, bias_pitch, mean_roll, mean_pitch, reflex {0}, extra [2];
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

    InitialFilter(&mean_roll, &mean_pitch, imu_data, gpio, imu_sensor, sampling_time, cutoff);

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
        // Measure the new values of the variable

        mean_roll = LowPassFilter(sampling_time/1000.0f, cutoff , mean_roll,imu_data.roll);
        mean_pitch = LowPassFilter(sampling_time/1000.0f, cutoff, mean_pitch, imu_data.pitch);

        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Check the quadrant

        quadrant = CheckQuadrant(mean_roll, mean_pitch);

        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Update the weight

        WeightUpdateRobot(mean_roll,  mean_pitch, weight_roll, weight_pitch, learning_rate, quadrant, &reflex);
        
        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Calculate the new speed

        //extra = ExtraCalculation(mean_pitch, mean_roll, speed, weight_roll, weight_pitch, limit, dir);
        extra[0] = ExtraL(mean_pitch, mean_roll, speed, weight_roll, weight_pitch, limit, dir);
        extra[1] = ExtraR(mean_pitch, mean_roll, speed, weight_roll, weight_pitch, limit, dir);
        SpeedSaturation1(extra, limit, speed, dir);

        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Apply the action

        if ((abs(mean_pitch) > 1) | (abs(mean_roll) > 1))
        {
            left.setMotorSpeedDirection(&gpio, speed[0] + extra[0], dir[0]);
            right.setMotorSpeedDirection(&gpio, speed[1] + extra[1], dir[1]);
        }
        else
        {
            left.setMotorSpeedDirection(&gpio, speed[0] , dir[0]);
            right.setMotorSpeedDirection(&gpio, speed[1] , dir[1]);
        }

        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Write in file

        file << weight_roll[0] << "," << weight_roll[1] << "," << weight_pitch[0] << "," << weight_pitch[1] << "," << weight_pitch[2] << "," << weight_pitch[3] << "," << imu_data.roll << "," << mean_roll << "," << imu_data.pitch << "," << mean_pitch << "," << speed[0]+extra[0] << "," << speed[1]+extra[1] << "," << reflex << std::endl;   
        
        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Assuring sampling time

        finish = std::chrono::high_resolution_clock::now();

        std::this_thread::sleep_for( std::chrono::milliseconds((int)sampling_time) - std::chrono::duration_cast<std::chrono::milliseconds>(finish - start));
    }

    file.close();

}