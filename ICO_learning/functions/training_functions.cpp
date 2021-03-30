#include <iostream>
#include <fstream>
#include <cstring>
#include <chrono>
#include "templates.hpp"
#include "aux.hpp"
#include "calibrate.hpp"

// Interfaces with IMU sensor
#include "matrix_hal/imu_sensor.h"
// Holds data from IMU sensor
#include "matrix_hal/imu_data.h"
// Communicates with MATRIX device
#include "matrix_hal/matrixio_bus.h"

// GPIO via Matrix Creator
#define  TB6612_RIGHT_MOTOR_PWMA        15 // (Orange)
#define  TB6612_LEFT_MOTOR_PWMB         14 // (Green)
#define  TB6612_RIGHT_MOTOR_AIN1        13 // (Blue)
#define  TB6612_RIGHT_MOTOR_AIN2        12 // (Brown)
#define  TB6612_LEFT_MOTOR_BIN1         11 // (Grey)
#define  TB6612_LEFT_MOTOR_BIN2         10 // (Pink)

void Run(float weight_roll[], float weight_pitch[] ,Motor left, Motor right, matrix_hal::IMUData imu_data, matrix_hal::GPIOControl gpio, matrix_hal::IMUSensor imu_sensor, float sampling_time, float cutoff, int speed[])
{
    int duration = 10000;
    bool run {true};
    auto begin = std::chrono::high_resolution_clock::now();
    auto end = std::chrono::high_resolution_clock::now();
    float extra[2];
    float roll {0}, pitch {0};
    int dir[2];

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// Initialize the filter



    for (int i = 0; i < 10; i++)
    {
        begin = std::chrono::high_resolution_clock::now();
        roll = LowPassFilter(sampling_time/1000.0f, cutoff, roll, imu_data.roll);
        pitch = LowPassFilter(sampling_time/1000.0f, cutoff, pitch, imu_data.pitch);
        end = std::chrono::high_resolution_clock::now();
        std::this_thread::sleep_for(std::chrono::milliseconds((int)sampling_time) - std::chrono::duration_cast<std::chrono::milliseconds>(end - begin));
    }

    begin = std::chrono::high_resolution_clock::now();
    end = std::chrono::high_resolution_clock::now();

    while(run)
    {
        if ( std::chrono::duration_cast<std::chrono::milliseconds>(end-begin).count() > duration)
        {
            left.setMotorSpeedDirection(&gpio, 0, dir[0]);
            right.setMotorSpeedDirection(&gpio, 0, dir[1]);
            run = false;
        }
        else
        {
            // Overwrites imu_data with new data from IMU sensor
            imu_sensor.Read(&imu_data);
            extra[0] = weight_pitch[0]*imu_data.pitch + weight_roll[0]*imu_data.roll;
            extra[1] = weight_pitch[1]*imu_data.pitch + weight_roll[1]*imu_data.roll;
            SpeedSaturation1(extra, 100, speed, dir);

            left.setMotorSpeedDirection(&gpio, speed[0] + extra[0], dir[0]);
            right.setMotorSpeedDirection(&gpio, speed[1] + extra[1], dir[1]);
        }
        
        end = std::chrono::high_resolution_clock::now();
    }
}

void train_roll(Motor left, Motor right, matrix_hal::IMUData imu_data, float weight_roll[], float learning_rate, int speed[], matrix_hal::GPIOControl gpio, matrix_hal::IMUSensor imu_sensor, float limit, int update_method, float sampling_time, float cutoff)
{
    //Variables required for the different calculations:
    float bias_roll;
    float roll_data[50];
    float mean_roll {0};
    int dir[2];
    float reflex {0};
    float extra[2];
    bool reflex_ON {false};
    std::ofstream file;
    auto finish = std::chrono::high_resolution_clock::now();
    auto start = std::chrono::high_resolution_clock::now();
    auto learning_start = std::chrono::high_resolution_clock::now();
    file.open("evolution.txt", std::ios_base::app);

    //Stabilize measurements part:

    for (int i = 0; i < sizeof(roll_data)/sizeof(roll_data[0]); i++){
        roll_data[i] = 0;
    }

    for (int i = 1; i < sizeof(roll_data)/sizeof(roll_data[0]); i++){

        start = std::chrono::high_resolution_clock::now();
        // Overwrites imu_data with new data from IMU sensor
        imu_sensor.Read(&imu_data);

        
        roll_data[i] = LowPassFilter(sampling_time/1000.0f, cutoff , roll_data[i-1],imu_data.roll);

        // std::cout << "Roll data in iteration " << i << " is: " << roll_data[i];


        finish = std::chrono::high_resolution_clock::now();

        std::this_thread::sleep_for(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::milliseconds((int)sampling_time) - (finish - start)));
    }

    mean_roll = mean(roll_data);

    // std::cout << "Finished calculating the mean original" << std::endl;

    bias_roll = BiasRoll(imu_data, gpio, imu_sensor, 100, sampling_time/1000.0f, cutoff);

    learning_start = std::chrono::high_resolution_clock::now();
    //Learning part
    while (true){

        // Record start time
        start = std::chrono::high_resolution_clock::now();

        //std::cout << "Value of the signal at the beginning of the iteration: " << mean_roll << std::endl;

        dir[0] = 1;
        dir[1] = 1;
        extra[0] = 0;
        extra[1] = 0;

        // Overwrites imu_data with new data from IMU sensor
        imu_sensor.Read(&imu_data);

        // roll_and_add(imu_data.roll, roll_data);

        // mean_roll = mean(roll_data) - bias_roll;

        // mean_roll = mean(roll_data);
        
        
        //std::cout << "Value of the signal just before the Filter: " << mean_roll << std::endl;


        // mean_roll = LowPassFilter(sampling_time/1000.0f, cutoff , mean_roll,imu_data.roll) - bias_roll;


        //mean_roll = LowPassFilter(sampling_time/1000.0f, cutoff , mean_roll,imu_data.roll - bias_roll);


        mean_roll = LowPassFilter(sampling_time/1000.0f, cutoff , mean_roll,imu_data.roll);

        if (abs(mean_roll) > 50.0f){
            break;
        }
        
        
        //std::cout << "Value of the signal after obtaining the new value from the low pass filter: " << mean_roll << std::endl;


        //Reflex signal calculation -> Depends on the roll angle sign to see which weight is updated

        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Weight update and speed saturation


        //print("Weights before weight update function:");
        //std::cout << weight_roll[0] << "  " << weight_roll[1] << std::endl;

        if (update_method == 1)
        {
            WeightUpdate1(mean_roll, limit, learning_rate, weight_roll, &reflex, &reflex_ON);
        }
        else if (update_method == 2)
        {
            WeightUpdate2(mean_roll, limit, learning_rate, weight_roll, &reflex, &reflex_ON);
        }
        else if (update_method == 3)
        {
            WeightUpdate3(mean_roll, limit, learning_rate, weight_roll, &reflex, &reflex_ON);
        }

        //print("Weights after weight update function:");
        //std::cout << weight_roll[0] << "  " << weight_roll[1] << std::endl;

        extra[0] = weight_roll[0]*mean_roll + reflex;
        extra[1] = weight_roll[1]*mean_roll + reflex;

        SpeedSaturation1(extra, 100, speed, dir);


        //std::cout << "Value of the signal after updating weights and saturating speed: " << mean_roll << std::endl;


        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Update speed in the motors
        if (abs(mean_roll) > 1)
        {
            left.setMotorSpeedDirection(&gpio, speed[0] + extra[0], dir[0]);
            right.setMotorSpeedDirection(&gpio, speed[1] + extra[1], dir[1]);
        }
        else
        {
            left.setMotorSpeedDirection(&gpio, speed[0], 0);
            right.setMotorSpeedDirection(&gpio, speed[1], 0);
        }



        //std::cout << "Value of the signal after updating speed in the motors: " << mean_roll << std::endl;

        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Writing in screen

        //std::cout << "speed[0] = " << speed[0]+extra[0] << "    speed[1] = " << speed[1]+extra[1] << std::endl;
        //std::cout << "dir[0]   =  " << dir[0] << "    dir[1] =  " << dir[1] << std::endl;
        std::cout << "Roll angle: " << mean_roll << std::endl;
        //std::cout << "Weight[0] = " << weight_roll[0] << "    Weight[1] = " << weight_roll[1] << std::endl;
        
        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Writing in file

        file << weight_roll[0] << "," << weight_roll[1] << "," << imu_data.roll << "," << mean_roll << "," << speed[0]+extra[0] << "," << speed[1]+extra[1] << "," << reflex << "," << std::chrono::duration_cast<std::chrono::milliseconds>(start - learning_start).count() << reflex_ON << std::endl;


        //std::cout << "Value of signal after the iteration: " << mean_roll << std::endl;

        //std::cout << "-------------------------------------------------------------------------------------------" << std::endl;

        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Timing sample

        finish = std::chrono::high_resolution_clock::now();

        std::this_thread::sleep_for( std::chrono::milliseconds((int)sampling_time) - std::chrono::duration_cast<std::chrono::milliseconds>(finish - start));

    }
    
    std::cout << "Finished learning round!" << std::endl;
        
    left.setMotorSpeedDirection(&gpio, 0, dir[0]);
    right.setMotorSpeedDirection(&gpio, 0, dir[1]);

    file.close();
}
