#include <iostream>
#include <fstream>
#include "../motor_control/motor_class.hpp"
#include <cstring>
#include <chrono>

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


void train_roll(Motor left, Motor right, matrix_hal::IMUData imu_data, float weight_roll[], float learning_rate, int speed[], matrix_hal::GPIOControl gpio, matrix_hal::IMUSensor imu_sensor)
{
    //Variables required for the different calculations:
    float sampling_time;
    float roll_data[10];
    int mean_roll {0};
    int dir[2];
    float reflex {0};
    float diff {0};
    float extra[2];
    std::ofstream file;
    auto finish = std::chrono::high_resolution_clock::now();
    auto start = std::chrono::high_resolution_clock::now();
    file.open("evolution.txt", std::ios_base::app);

    //Stabilize measurements part:
    
    
    for (int i = 0; i < sizeof(roll_data)/sizeof(roll_data[0]); i++){
        roll_data[i] = 0;
    }

    for (int i = 0; i < sizeof(roll_data)/sizeof(roll_data[0]); i++){
        // Overwrites imu_data with new data from IMU sensor
        imu_sensor.Read(&imu_data);
        roll_data[i] = imu_data.roll;
    }

    mean_roll = mean(roll_data);

    sampling_time = 10;

    //Learning part
    while (true){


        // Record start time
        start = std::chrono::high_resolution_clock::now();

        diff = 0;
        dir[0] = 1;
        dir[1] = 1;
        extra[0] = 0;
        extra[1] = 0;

        // Overwrites imu_data with new data from IMU sensor
        imu_sensor.Read(&imu_data);

        roll_and_add(imu_data.roll, roll_data);
        mean_roll = mean(roll_data);

        if (abs(mean_roll) > 50.0f){
            break;
        }

        //Reflex signal calculation -> Depends on the roll angle sign to see which weight is updated

        if (mean_roll > 3.0f){
            diff = mean_roll - reflex;
            weight_roll[0] += learning_rate*mean_roll*diff;
        
            reflex = mean_roll;
        }
        else if (mean_roll < -3.0f){
            diff = mean_roll - reflex;
            weight_roll[1] += learning_rate*mean_roll*diff;

            reflex = mean_roll;
        }
        else {
            reflex = 0;
        }

        extra[0] = weight_roll[0]*mean_roll + reflex;
        extra[1] = weight_roll[1]*mean_roll + reflex;


        if (extra[0] < 0){
            extra[0] = -extra[0];
            dir[0] = 0;
        }
        else{
            dir[0] = 1;
        }
        if ((extra[0] + speed[0]) > 100){
            extra[0] = 100-speed[0];
        }


        if (extra[1] < 0){
            extra[1] = -extra[1];
            dir[1] = 0;
        }
        else{
            dir[1] = 1;
        }
        if ((extra[1] + speed[1]) > 100){
            extra[1] = 100-speed[1];
        }

        std::cout << "speed[0] = " << speed[0]+extra[0] << "    speed[1] = " << speed[1]+extra[1] << std::endl;
        std::cout << "dir[0]   =  " << dir[0] << "    dir[1] =  " << dir[1] << std::endl;

        left.setMotorSpeedDirection(&gpio, speed[0] + extra[0], dir[0]);
        right.setMotorSpeedDirection(&gpio, speed[1] + extra[1], dir[1]);

        std::cout << "Roll angle: " << mean_roll << std::endl;
        std::cout << "Weight[0] = " << weight_roll[0] << "    Weight[1] = " << weight_roll[1] << std::endl;
        
        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Record end time
        finish = std::chrono::high_resolution_clock::now();

        std::cout << "Time spent: " << std::chrono::duration_cast<std::chrono::milliseconds>(finish - start).count() << " miliseconds" << std::endl;

        while (std::chrono::duration_cast<std::chrono::milliseconds>(finish - start).count() < sampling_time)
        {
            finish = std::chrono::high_resolution_clock::now();
        }

        file << weight_roll[0] << "," << weight_roll[1] << "," << imu_data.roll << "," << mean_roll << "," << speed[0]+extra[0] << "," << speed[1]+extra[1] << "," << reflex << "," << std::chrono::duration_cast<std::chrono::milliseconds>(finish - start).count() << std::endl;

        std::cout << "-------------------------------------------------------------------------------------------" << std::endl;

    }
    
    std::cout << "Finished learning round!" << std::endl;
    
    
    left.setMotorSpeedDirection(&gpio, 0, dir[0]);
    right.setMotorSpeedDirection(&gpio, 0, dir[1]);

    file.close();
}

