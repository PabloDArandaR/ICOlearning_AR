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
    ////////////////////////////////////////////////////////////////////////
    // Initialization of variables
    int duration = 10000;
    auto begin = std::chrono::high_resolution_clock::now();
    auto end = std::chrono::high_resolution_clock::now();
    float extra[2];
    float roll {0}, pitch {0};
    int dir[2];
    float roll_original{0};
    float pitch_original{0};

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// Initialize the filter

    for (int i = 0; i < 50; i++)
    {
        // Capture the initial moment of the iteration
        begin = std::chrono::high_resolution_clock::now();

        // Calculate both roll and pitch based on the new reading
        roll = LowPassFilter(sampling_time/1000.0f, cutoff, roll, imu_data.roll);
        pitch = LowPassFilter(sampling_time/1000.0f, cutoff, pitch, imu_data.pitch);

        //Capture the final moment of the iteration
        end = std::chrono::high_resolution_clock::now();

        // Check how much time you have to wait to accomplish the sampling time.
        std::this_thread::sleep_for(std::chrono::milliseconds((int)sampling_time) - std::chrono::duration_cast<std::chrono::milliseconds>(end - begin));
    }

    // This 2 values will be used to determine the bias, supposing with this that the robot is starting with both real angles = 0
    roll_original = roll;
    pitch_original = pitch;

    // Initialize the timing variables
    begin = std::chrono::high_resolution_clock::now();
    end = std::chrono::high_resolution_clock::now();

    // Start running
    while(true)
    {
        // If already surpassed the 10 seconds, stop the motor and exit the loop
        if ( std::chrono::duration_cast<std::chrono::milliseconds>(end-begin).count() > duration)
        {
            left.setMotorSpeedDirection(&gpio, 0, dir[0]);
            right.setMotorSpeedDirection(&gpio, 0, dir[1]);
            break;
        }
        // Else, calculate new angles and all the required procedure to obtain the new speeds (saturation, noise effect reducing, ...)
        else
        {
            // Overwrites imu_data with new data from IMU sensor
            imu_sensor.Read(&imu_data);

            roll = LowPassFilter(sampling_time/1000.0f, cutoff, roll, imu_data.roll);
            pitch = LowPassFilter(sampling_time/1000.0f, cutoff, pitch, imu_data.pitch);

            extra[0] = weight_pitch[0]*pitch + weight_roll[0]*roll;
            extra[1] = weight_pitch[1]*pitch + weight_roll[1]*roll;
            SpeedSaturation1(extra, 100, speed, dir);

            // Update speed in the motors
            if (abs(roll) > 1)
            {
                left.setMotorSpeedDirection(&gpio, speed[0] + extra[0], dir[0]);
                right.setMotorSpeedDirection(&gpio, speed[1] + extra[1], dir[1]);
            }
            else
            {
                left.setMotorSpeedDirection(&gpio, speed[0], 0);
                right.setMotorSpeedDirection(&gpio, speed[1], 0);
            }
        }
        
        // Recalculate end  variable.
        end = std::chrono::high_resolution_clock::now();
    }
}

void TrainRoll(Motor left, Motor right, matrix_hal::IMUData imu_data, float weight_roll[], float weight_pitch[], float learning_rate, int speed[], matrix_hal::GPIOControl gpio, matrix_hal::IMUSensor imu_sensor, float limit, int update_method, float sampling_time, float cutoff, int * iteration)
{
    //Variables required for the different calculations:
    float bias_roll;
    float mean_roll {0};
    float mean_pitch {0};
    int dir[2];
    float reflex {0};
    float extra[2];
    bool reflex_ON {false};
    std::ofstream file;
    auto finish = std::chrono::high_resolution_clock::now();
    auto start = std::chrono::high_resolution_clock::now();
    file.open("evolution_roll.txt", std::ios_base::app);

    auto beginning = std::chrono::high_resolution_clock::now();

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Increase the value of the iteration variable to acknowledge how many iterations have been accomplished
    *iteration++;

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //Initialize the filter:

    imu_sensor.Read(&imu_data);
    mean_roll = imu_data.roll;
    mean_pitch = imu_data.pitch;

    for (int i = 0; i < 50; i++){

        start = std::chrono::high_resolution_clock::now();
        
        // Overwrites imu_data with new data from IMU sensor
        imu_sensor.Read(&imu_data);

        mean_roll = LowPassFilter(sampling_time/1000.0f, cutoff , mean_roll, imu_data.roll);
        mean_pitch = LowPassFilter(sampling_time/1000.0f, cutoff , mean_pitch, imu_data.pitch);

        finish = std::chrono::high_resolution_clock::now();

        std::this_thread::sleep_for(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::milliseconds((int)sampling_time) - (finish - start)));
    }

    // The bias will be calculated suposing that the initial inclination is 0
    // It will be used depending on the line that it is used for updating the mean_roll in each iteration
    bias_roll = BiasRoll(imu_data, gpio, imu_sensor, 100, sampling_time/1000.0f, cutoff);
    bias_pitch = BiasPitch(imu_data, gpio, imu_sensor, 100, sampling_time/1000.0f, cutoff);

    beginning = std::chrono::high_resolution_clock::now();

    //Learning part (it will run until a break statement is detected)
    while (true){

        // Record start time
        start = std::chrono::high_resolution_clock::now();

        // Initialize the Dir variable to go backwards and the extra to 0
        dir[0] = 1;
        dir[1] = 1;
        extra[0] = 0;
        extra[1] = 0;

        // Overwrites imu_data with new data from IMU sensor
        imu_sensor.Read(&imu_data);

        // Alternative ways of updating the mean_roll variable
        // mean_roll = LowPassFilter(sampling_time/1000.0f, cutoff , mean_roll,imu_data.roll) - bias_roll;
        // mean_roll = LowPassFilter(sampling_time/1000.0f, cutoff , mean_roll,imu_data.roll - bias_roll);

        // Updating the mean_roll variable
        mean_roll = LowPassFilter(sampling_time/1000.0f, cutoff , mean_roll,imu_data.roll);
        mean_pitch = LowPassFilter(sampling_time/1000.0f, cutoff , mean_pitch, imu_data.pitch);

        // Threshold - if this value is surpassed, training finishes
        if (abs(mean_roll) > 50.0f){
            break;
        }

        // Check time limit:
        if ( std::chrono::duration_cast<std::chrono::milliseconds>(start - beginning).count() > 10000)
        {
            break;
        }

        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Weight update and speed saturation

        if (update_method == 1)
        {
            WeightUpdateR1(mean_roll, limit, learning_rate, weight_roll, &reflex, &reflex_ON);
        }
        else if (update_method == 2)
        {
            WeightUpdateR2(mean_roll, limit, learning_rate, weight_roll, &reflex, &reflex_ON);
        }
        else if (update_method == 3)
        {
            WeightUpdateR3(mean_roll, limit, learning_rate, weight_roll, &reflex, &reflex_ON);
        }

        // Calculate the extra value added to the speed
        extra[0] = weight_roll[0]*mean_roll + weight_pitch[0]*mean_pitch + reflex;
        extra[1] = weight_roll[1]*mean_roll + weight_pitch[1]*mean_pitch + reflex;


        //Saturate the extra value and check the direction in which it will go
        SpeedSaturation1(extra, 100, speed, dir);

        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Update speed in the motors (if the value is very low, it is considered noise)
        ((abs(mean_roll) > 1) | (abs(mean_pitch)  > 1))
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
        std::cout << "Roll angle: " << mean_roll << std::endl;
        
        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Writing in file

        file << weight_roll[0] << "," << weight_roll[1] << "," << imu_data.roll << "," << mean_roll << "," << speed[0]+extra[0] << "," << speed[1]+extra[1] << "," << reflex << "," << std::chrono::duration_cast<std::chrono::milliseconds>(start - beginning).count()<< ',' << reflex_ON << ',' << iteration<< std::endl;


        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Timing sample

        finish = std::chrono::high_resolution_clock::now();

        std::this_thread::sleep_for( std::chrono::milliseconds((int)sampling_time) - std::chrono::duration_cast<std::chrono::milliseconds>(finish - start));

    }
    
    std::cout << "Finished learning round! \n \n" << std::endl;
    
    // Stop the motor
    left.setMotorSpeedDirection(&gpio, 0, dir[0]);
    right.setMotorSpeedDirection(&gpio, 0, dir[1]);

    //Close the file
    file.close();
}

void TrainBoth(Motor left, Motor right, matrix_hal::IMUData imu_data, float weight_roll[], float weight_pitch[], float learning_rate, int speed[], matrix_hal::GPIOControl gpio, matrix_hal::IMUSensor imu_sensor, float limit, float sampling_time, float cutoff, int * iteration)
{
    //Variables required for the different calculations:
    float bias_roll, bias_pitch;
    float mean_roll, mean_pitch;
    int dir[2];
    float reflex {0};
    float extra[2];
    bool reflex_ON {false};
    std::ofstream file;
    auto finish = std::chrono::high_resolution_clock::now();
    auto start = std::chrono::high_resolution_clock::now();
    file.open("evolution_both.txt", std::ios_base::app);
    auto beginning = std::chrono::high_resolution_clock::now();

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Increase the value of the iteration variable to acknowledge how many iterations have been accomplished
    *iteration++;

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //Initialize the filter:

    imu_sensor.Read(&imu_data);
    mean_roll = imu_data.roll;
    mean_pitch = imu_data.pitch;

    for (int i = 0; i < 50; i++){

        start = std::chrono::high_resolution_clock::now();
        
        // Overwrites imu_data with new data from IMU sensor
        imu_sensor.Read(&imu_data);

        mean_roll = LowPassFilter(sampling_time/1000.0f, cutoff , mean_roll, imu_data.roll);
        mean_pitch = LowPassFilter(sampling_time/1000.0f, cutoff, mean_pitch, imu_data.pitch);

        finish = std::chrono::high_resolution_clock::now();

        std::this_thread::sleep_for(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::milliseconds((int)sampling_time) - (finish - start)));
    }

    // The bias will be calculated suposing that the initial inclination is 0
    // It will be used depending on the line that it is used for updating the mean_roll in each iteration
    bias_roll = BiasRoll(imu_data, gpio, imu_sensor, 100, sampling_time/1000.0f, cutoff);
    bias_pitch = BiasPitch(imu_data, gpio, imu_sensor, 100, sampling_time/1000.0f, cutoff);

    beginning = std::chrono::high_resolution_clock::now();

    //Learning part (it will run until a break statement is detected)
    while (true){

        // Record start time
        start = std::chrono::high_resolution_clock::now();

        // Initialize the Dir variable to go backwards and the extra to 0
        dir[0] = 1;
        dir[1] = 1;
        extra[0] = 0;
        extra[1] = 0;

        // Overwrites imu_data with new data from IMU sensor
        imu_sensor.Read(&imu_data);

        // Alternative ways of updating the mean_roll variable
        // mean_roll = LowPassFilter(sampling_time/1000.0f, cutoff , mean_roll,imu_data.roll) - bias_roll;
        // mean_roll = LowPassFilter(sampling_time/1000.0f, cutoff , mean_roll,imu_data.roll - bias_roll);

        // Updating the mean_roll variable
        mean_roll = LowPassFilter(sampling_time/1000.0f, cutoff , mean_roll,imu_data.roll);
        mean_pitch = LowPassFilter(sampling_time/1000.0f, cutoff, mean_pitch, imu_data.pitch);

        // Threshold - if this value is surpassed, training finishes
        if ((abs(mean_roll) > 50.0f) | (abs(mean_pitch) > 30.0f)){
            break;
        }

        // Check time limit:
        if ( std::chrono::duration_cast<std::chrono::milliseconds>(start - beginning).count() > 10000)
        {
            break;
        }

        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Weight update and speed saturation

        WeightUpdateB(mean_pitch, mean_roll, limit, learning_rate, weight_roll, weight_pitch, &reflex, &reflex_ON);

        // Calculate the extra value added to the speed
        extra[0] = weight_roll[0]*mean_roll + weight_pitch[0]*mean_pitch + reflex;
        extra[1] = weight_roll[1]*mean_roll + weight_pitch[1]*mean_pitch + reflex;


        //Saturate the extra value and check the direction in which it will go
        SpeedSaturation1(extra, 100, speed, dir);

        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Update speed in the motors (if the value is very low, it is considered noise)
        if ((abs(mean_roll) > 1) | (abs(mean_pitch)  > 1))
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
        std::cout << "Roll angle: " << mean_roll << std::endl;
        std::cout << "Pitch angle: " << mean_pitch << std::endl;
        
        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Writing in file

        file << weight_roll[0] << "," << weight_roll[1] << "," << weight_pitch[0] << "," << weight_pitch[1] << "," << imu_data.roll << "," << mean_roll << "," << imu_data.pitch << "," << mean_pitch << "," << speed[0]+extra[0] << "," << speed[1]+extra[1] << "," << reflex << "," << std::chrono::duration_cast<std::chrono::milliseconds>(start - beginning).count()<< ',' << reflex_ON << ',' << iteration<< std::endl;
        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Check time limit:

        if ( std::chrono::duration_cast<std::chrono::milliseconds>(start - beginning).count() > 10000)
        {
            break;
        }

        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Timing sample

        finish = std::chrono::high_resolution_clock::now();

        std::this_thread::sleep_for( std::chrono::milliseconds((int)sampling_time) - std::chrono::duration_cast<std::chrono::milliseconds>(finish - start));

    }
    
    std::cout << "Finished learning round! \n \n" << std::endl;
    
    // Stop the motor
    left.setMotorSpeedDirection(&gpio, 0, dir[0]);
    right.setMotorSpeedDirection(&gpio, 0, dir[1]);

    //Close the file
    file.close();
}
