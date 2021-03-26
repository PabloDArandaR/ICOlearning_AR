#include <iostream>
#include <fstream>
#include <cstring>
#include <chrono>
#include "templates.hpp"
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

void WeightUpdate1(float mean, float limit, float learning_rate, float * weight, float * reflex)
{
    float diff {.0f};

    if (mean > limit){
        diff = mean - *reflex;
        weight[0] += learning_rate*mean*diff;
        
        *reflex = mean;
    }
    else if (mean < -limit){
        diff = mean - *reflex;
        weight[1] += learning_rate*mean*diff;

        *reflex = mean;
        }
    else 
    {
        *reflex = 0.0f;
    }
}

void WeightUpdate2(float mean, float limit, float learning_rate, float * weight, float * reflex)
{
    float diff {.0f};

    diff = mean - *reflex;
    weight[0] += learning_rate*mean*diff;
    weight[1] -= learning_rate*mean*diff;

    if (abs(mean) < limit)
    {
        *reflex = 0;
    }
    else
    {
        *reflex = mean;
    }
}

void WeightUpdate3(float mean, float limit, float learning_rate, float * weight, float * reflex)
{
    float diff {.0f};

    if (mean > limit){
        diff = mean - *reflex;
        weight[0] += learning_rate*mean*diff;
        weight[1] -= learning_rate*mean*diff;
        
        *reflex = mean;
    }
    else if (mean < -limit){
        diff = mean - *reflex;
        weight[1] += learning_rate*mean*diff;
        weight[0] -= learning_rate*mean*diff;

        *reflex = mean;
        }
    else 
    {
        *reflex = 0.0f;
    }
}

void SpeedSaturation(float * extra, float limit, const int speed[], int dir[])
{
    for (int i = 0; i < sizeof(extra)/sizeof(extra[0]); i++)
    {
        if (extra[i] < 0){
            extra[i] = -extra[i];
            dir[i] = 1;
        }
        else{
            dir[i] = 0;
        }
        if ((extra[i] + speed[i]) > 100){
            extra[i] = limit-speed[0];
        }

    }
}

void train_roll(Motor left, Motor right, matrix_hal::IMUData imu_data, float weight_roll[], float learning_rate, int speed[], matrix_hal::GPIOControl gpio, matrix_hal::IMUSensor imu_sensor, float limit, int update_method)
{
    //Variables required for the different calculations:
    float bias_roll;
    int sampling_time;
    float roll_data[500];
    float mean_roll {0};
    int dir[2];
    float reflex {0};
    float diff {0};
    float extra[2];
    std::ofstream file;
    auto finish = std::chrono::high_resolution_clock::now();
    auto start = std::chrono::high_resolution_clock::now();
    file.open("evolution.txt", std::ios_base::app);

    //Stabilize measurements part:


    sampling_time = 10;
    
    
    for (int i = 0; i < sizeof(roll_data)/sizeof(roll_data[0]); i++){
        roll_data[i] = 0;
    }

    for (int i = 1; i < sizeof(roll_data)/sizeof(roll_data[0]); i++){

        start = std::chrono::high_resolution_clock::now();
        // Overwrites imu_data with new data from IMU sensor
        imu_sensor.Read(&imu_data);

        
        roll_data[i] = LowPassFilter(0.01f, 0.1f , roll_data[i-1],imu_data.roll);

        std::cout << "Roll data in iteration " << i << " is: " << roll_data[i];


        finish = std::chrono::high_resolution_clock::now();

        std::this_thread::sleep_for(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::milliseconds(sampling_time) - (finish - start)));
    }

    mean_roll = mean(roll_data);

    bias_roll = BiasRoll(imu_data, gpio, imu_sensor, 10000);

    print("Bias roll:");
    print(bias_roll);

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

        // roll_and_add(imu_data.roll, roll_data);

        // mean_roll = mean(roll_data) - bias_roll;

        // mean_roll = mean(roll_data);

        mean_roll = LowPassFilter(0.01f, 0.1f , mean_roll,imu_data.roll) - bias_roll;

        // mean_roll = LowPassFilter(0.01f, 0.1f , mean_roll,imu_data.roll);

        if (abs(mean_roll) > 50.0f){
            break;
        }

        //Reflex signal calculation -> Depends on the roll angle sign to see which weight is updated

        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Weight update and speed staration

        if (update_method == 1)
        {
            WeightUpdate1(mean_roll, limit, learning_rate, weight_roll, &reflex);
        }
        else if (update_method == 2)
        {
            WeightUpdate2(mean_roll, limit, learning_rate, weight_roll, &reflex);
        }
        else if (update_method == 3)
        {
            WeightUpdate3(mean_roll, limit, learning_rate, weight_roll, &reflex);
        }
        
        print("Weights after weight update function:");
        std::cout << weight_roll[0] << "  " << weight_roll[1] << std::endl;

        extra[0] = weight_roll[0]*mean_roll + reflex;
        extra[1] = weight_roll[1]*mean_roll + reflex;

        SpeedSaturation(extra, 100, speed, dir);

        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Update speed in the motors
        if (abs(mean_roll) > 2)
        {
            left.setMotorSpeedDirection(&gpio, speed[0] + extra[0], dir[0]);
            right.setMotorSpeedDirection(&gpio, speed[1] + extra[1], dir[1]);
        }
        else
        {
            left.setMotorSpeedDirection(&gpio, speed[0], 0);
            right.setMotorSpeedDirection(&gpio, speed[1], 0);
        }



        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Writing in screen

        //std::cout << "speed[0] = " << speed[0]+extra[0] << "    speed[1] = " << speed[1]+extra[1] << std::endl;
        //std::cout << "dir[0]   =  " << dir[0] << "    dir[1] =  " << dir[1] << std::endl;
        std::cout << "Roll angle: " << mean_roll << std::endl;
        //std::cout << "Weight[0] = " << weight_roll[0] << "    Weight[1] = " << weight_roll[1] << std::endl;
        
        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Timing sample

        finish = std::chrono::high_resolution_clock::now();

        std::this_thread::sleep_for(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::milliseconds(sampling_time) - (finish - start)));

        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Writing in file

        file << weight_roll[0] << "," << weight_roll[1] << "," << imu_data.roll << "," << mean_roll << "," << speed[0]+extra[0] << "," << speed[1]+extra[1] << "," << reflex << "," << std::chrono::duration_cast<std::chrono::milliseconds>(finish - start).count() << std::endl;

        std::cout << "-------------------------------------------------------------------------------------------" << std::endl;

    }
    
    std::cout << "Finished learning round!" << std::endl;
    
    
    left.setMotorSpeedDirection(&gpio, 0, dir[0]);
    right.setMotorSpeedDirection(&gpio, 0, dir[1]);

    file.close();
}

/*
void train_pitch(Motor left, Motor right, matrix_hal::IMUData imu_data, float weight_pitch[], float learning_rate, int speed[], matrix_hal::GPIOControl gpio, matrix_hal::IMUSensor imu_sensor)
{
    //Variables required for the different calculations:
    float sampling_time, bias_pitch;
    float pitch_data[10];
    int mean_pitch {0};
    int dir[2];
    float reflex {0};
    float diff {0};
    float extra[2];
    std::ofstream file;
    auto finish = std::chrono::high_resolution_clock::now();
    auto start = std::chrono::high_resolution_clock::now();
    file.open("evolution.txt", std::ios_base::app);

    //Stabilize measurements part:
    
    
    for (int i = 0; i < sizeof(pitch_data)/sizeof(pitch_data[0]); i++){
        pitch_data[i] = 0;
    }

    for (int i = 0; i < sizeof(pitch_data)/sizeof(pitch_data[0]); i++){
        // Overwrites imu_data with new data from IMU sensor
        imu_sensor.Read(&imu_data);
        pitch_data[i] = imu_data.pitch;
    }

    bias_pitch = BiasPitch(imu_data, gpio, imu_sensor, 10000);
    mean_pitch = mean(pitch_data)-bias_pitch;

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

        roll_and_add(imu_data.pitch, pitch_data);
        mean_pitch = mean(pitch_data) - bias_pitch;

        if (abs(mean_pitch) > 50.0f){
            break;
        }

        //Reflex signal calculation -> Depends on the pitch angle sign to see which weight is updated

        if (mean_pitch > 3.0f){
            diff = mean_pitch - reflex;
            weight_pitch[0] += learning_rate*mean_pitch*diff;
        
            reflex = mean_pitch;
        }
        else if (mean_pitch < -3.0f){
            diff = mean_pitch - reflex;
            weight_pitch[1] += learning_rate*mean_pitch*diff;

            reflex = mean_pitch;
        }
        else {
            reflex = 0;
        }

        extra[0] = weight_pitch[0]*mean_pitch + reflex;
        extra[1] = weight_pitch[1]*mean_pitch + reflex;


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

        std::cout << "pitch angle: " << mean_pitch << std::endl;
        std::cout << "Weight[0] = " << weight_pitch[0] << "    Weight[1] = " << weight_pitch[1] << std::endl;
        
        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Record end time
        finish = std::chrono::high_resolution_clock::now();

        std::cout << "Time spent: " << std::chrono::duration_cast<std::chrono::milliseconds>(finish - start).count() << " miliseconds" << std::endl;

        while (std::chrono::duration_cast<std::chrono::milliseconds>(finish - start).count() < sampling_time)
        {
            finish = std::chrono::high_resolution_clock::now();
        }

        file << weight_pitch[0] << "," << weight_pitch[1] << "," << imu_data.pitch << "," << mean_pitch << "," << speed[0]+extra[0] << "," << speed[1]+extra[1] << "," << reflex << "," << std::chrono::duration_cast<std::chrono::milliseconds>(finish - start).count() << std::endl;

        std::cout << "-------------------------------------------------------------------------------------------" << std::endl;

    }
    
    std::cout << "Finished learning round!" << std::endl;
    
    
    left.setMotorSpeedDirection(&gpio, 0, dir[0]);
    right.setMotorSpeedDirection(&gpio, 0, dir[1]);

    file.close();
}
*/

/*
void train_pitch_roll(Motor left, Motor right, matrix_hal::IMUData imu_data, float weight_pitch[], float weight_roll[], float learning_rate, int speed[], matrix_hal::GPIOControl gpio, matrix_hal::IMUSensor imu_sensor)
{
    //Variables required for the different calculations:
    float sampling_time;
    float pitch_data[10];
    int mean_pitch {0};
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
    
    
    for (int i = 0; i < sizeof(pitch_data)/sizeof(pitch_data[0]); i++){
        pitch_data[i] = 0;
        roll_data[i] = 0;
    }

    for (int i = 0; i < sizeof(pitch_data)/sizeof(pitch_data[0]); i++){
        // Overwrites imu_data with new data from IMU sensor
        imu_sensor.Read(&imu_data);
        pitch_data[i] = imu_data.pitch;
        roll_data[i] = imu_data.roll;
    }

    mean_pitch = mean(pitch_data);
    mean_roll  = mean(roll_data);

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

        pitch_and_add(imu_data.pitch, pitch_data);
        pitch_and_add(imu_data.roll, roll_data);
        mean_pitch = mean(pitch_data);
        mean_pitch = mean(roll_data);

        if ((abs(mean_pitch) > 50.0f) | (abs(mean_roll) > 50.0f){
            break;
        }

        //Reflex signal calculation -> Depends on the pitch angle sign to see which weight is updated

        if (mean_pitch > 3.0f){
            diff = mean_pitch - reflex;
            weight_pitch[0] += learning_rate*mean_pitch*diff;
        
            reflex = mean_pitch;
        }
        else if (mean_pitch < -3.0f){
            diff = mean_pitch - reflex;
            weight_pitch[1] += learning_rate*mean_pitch*diff;

            reflex = mean_pitch;
        }
        else {
            reflex = 0;
        }

        extra[0] = weight_pitch[0]*mean_pitch + reflex;
        extra[1] = weight_pitch[1]*mean_pitch + reflex;


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
template <typename T>
void print_array(T array_of_values []){
    int number_of_values {sizeof(array_of_values) / (sizeof(array_of_values[0]))};

    for (int i = 0; i <= number_of_values; i++){
        std::cout <<  array_of_values[i] << "  ";
    }

    std::cout << std::endl;
}
d[1]) > 100){
            extra[1] = 100-speed[1];
        }

        std::cout << "speed[0] = " << speed[0]+extra[0] << "    speed[1] = " << speed[1]+extra[1] << std::endl;
        std::cout << "dir[0]   =  " << dir[0] << "    dir[1] =  " << dir[1] << std::endl;

        left.setMotorSpeedDirection(&gpio, speed[0] + extra[0], dir[0]);
        right.setMotorSpeedDirection(&gpio, speed[1] + extra[1], dir[1]);

        std::cout << "pitch angle: " << mean_pitch << std::endl;
        std::cout << "Weight[0] = " << weight_pitch[0] << "    Weight[1] = " << weight_pitch[1] << std::endl;
        
        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Record end time
        finish = std::chrono::high_resolution_clock::now();

        std::cout << "Time spent: " << std::chrono::duration_cast<std::chrono::milliseconds>(finish - start).count() << " miliseconds" << std::endl;

        while (std::chrono::duration_cast<std::chrono::milliseconds>(finish - start).count() < sampling_time)
        {
            finish = std::chrono::high_resolution_clock::now();
        }


        std::cout << "-------------------------------------------------------------------------------------------" << std::endl;

    }
    
    std::cout << "Finished learning round!" << std::endl;
    
    
    left.setMotorSpeedDirection(&gpio, 0, dir[0]);
    right.setMotorSpeedDirection(&gpio, 0, dir[1]);

    file.close();
}
*/

