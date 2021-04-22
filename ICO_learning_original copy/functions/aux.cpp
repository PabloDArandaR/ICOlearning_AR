#include <iostream>
#include <fstream>
#include <cstring>
#include <chrono>
#include "templates.hpp"

// Interfaces with IMU sensor
#include "matrix_hal/imu_sensor.h"
// Holds data from IMU sensor
#include "matrix_hal/imu_data.h"
// Communicates with MATRIX device
#include "matrix_hal/matrixio_bus.h"

// Weight update method 1
void WeightUpdateR1(float mean, float limit, float learning_rate, float weight[], float * reflex, bool *reflex_ON)
{
    float diff {.0f};

    // If the value of the signal surpasses the reflex signal threshold, the update will be commited.
    // If not, there won't be any update
    if (mean > limit){
        // Calculating the diff between the current value and the previous reflex signal ( the current signal value and the reflex signal are the same if the threshold is surpassed)
        diff = mean - *reflex;

        // Update of the weight using the given learning rule
        weight[0] += learning_rate*mean*diff;
        
        // Update the reflex related variables
        *reflex = mean;
        *reflex_ON = true;
    }
    // The sign of the roll signal is taken into consideration
    else if (mean < -limit){
        diff = mean - *reflex;
        weight[1] += learning_rate*mean*diff;

        *reflex = mean;
        *reflex_ON = true;
    }
    //If the mean is not over the threshold, the reflex will be equal to 0
    else 
    {
        *reflex = 0.0f;
        *reflex_ON = false;
    }
}

// Weight update method 2
void WeightUpdateR2(float mean, float limit, float learning_rate, float weight[], float * reflex, bool *reflex_ON)
{

    // Difference between current and previous reflex signal is calculated directly
    float diff {mean - *reflex};


    // If the absolute value of the signal is inferior to the module, the next reflex will be equal to 0
    if (abs(mean) < limit)
    {
        *reflex = 0;
        *reflex_ON = false;
    }
    else
    {
        // Both weights are updated following the learning rule, each one with a differrent sign because the reflex signal in this case is considered to be equal but with opposite sign
        weight[0] += learning_rate*mean*diff;
        weight[1] -= learning_rate*mean*diff;

        *reflex = mean;
        *reflex_ON = true;
    }
}

// Weight update method 3 -> it is the same as the method 1, but with the difference that the learning rule is applied to both weights in any of the cases in which the limit is surpassed
void WeightUpdateR3(float mean, float limit, float learning_rate, float weight[], float * reflex, bool *reflex_ON)
{
    float diff {.0f};

    if (mean > limit){
        diff = mean - *reflex;
        weight[0] += learning_rate*mean*diff;
        weight[1] -= learning_rate*mean*diff;
        
        *reflex = mean;
        *reflex_ON = true;
    }
    else if (mean < -limit){
        diff = mean - *reflex;
        weight[1] += learning_rate*mean*diff;
        weight[0] -= learning_rate*mean*diff;

        *reflex = mean;
        *reflex_ON = true;
    }
    else 
    {
        *reflex = 0.0f;
        *reflex_ON = false;
    }
}

// Weight update method 1
void WeightUpdateB1(float mean_pitch, float mean_roll, float limit, float learning_rate, float weight_roll[], float weight_pitch[], float * reflex, bool *reflex_ON)
{
    float diff {.0f};
    float mean {.0f};

    // If the value of the signal surpasses the reflex signal threshold, the update will be commited.
    // If not, there won't be any update
    // Possible interference in the learning process due to the sign of both angles
    if (abs(mean_roll) > limit){
        mean += mean_roll;
    }
    if (abs(mean_pitch) > limit){
        mean += mean_pitch;
    }

    if ((abs(mean_roll) > limit) & (abs(mean_pitch) > limit))
    {
        *reflex_ON = false;
    }
    else
    {
        *reflex_ON = true;
    }

    diff = mean - *reflex;


    // Update of the weight using the given learning rule
    weight_roll[0] += learning_rate*mean_roll*diff;
    weight_roll[1] -= learning_rate*mean_roll*diff;
    weight_pitch[0] += learning_rate*abs(mean_pitch)*diff;
    weight_pitch[1] += learning_rate*abs(mean_pitch)*diff;

    *reflex = mean;
}

void WeightUpdateB2(float mean_pitch, float mean_roll, float limit, float learning_rate, float weight_roll[], float weight_pitch[], float * reflex, bool *reflex_ON)
{
    float diff {.0f};
    float mean {.0f};

    // If the value of the signal surpasses the reflex signal threshold, the update will be commited.
    // If not, there won't be any update
    // Possible interference in the learning process due to the sign of both angles
    if (abs(mean_roll) > limit){
        mean += mean_roll;
    }
    if (abs(mean_pitch) > limit){
        mean += mean_pitch;
    }

    if ((abs(mean_roll) > limit) & (abs(mean_pitch) > limit))
    {
        *reflex_ON = false;
    }
    else
    {
        *reflex_ON = true;
    }

    diff = mean - *reflex;


    // Update of the weight using the given learning rule

    if (mean_roll > 0)
    {
        weight_roll[0] += learning_rate*mean_roll*diff;
    }
    else
    {
        weight_roll[1] += learning_rate*mean_roll*diff;
    }

    weight_pitch[0] -= learning_rate*abs(mean_pitch)*diff;
    weight_pitch[1] -= learning_rate*abs(mean_pitch)*diff;

    *reflex = mean;
}

void WeightUpdateB3(float mean_pitch, float mean_roll, float limit, float learning_rate, float weight_roll[], float weight_pitch[], float * reflex, bool *reflex_ON)
{
    float diff {.0f};
    float mean {.0f};

    // If the value of the signal surpasses the reflex signal threshold, the update will be commited.
    // If not, there won't be any update
    // Possible interference in the learning process due to the sign of both angles
    if (abs(mean_roll) > limit){
        mean += mean_roll;
    }
    if (abs(mean_pitch) > limit){
        mean += mean_pitch;
    }

    if ((abs(mean_roll) > limit) & (abs(mean_pitch) > limit))
    {
        *reflex_ON = false;
    }
    else
    {
        *reflex_ON = true;
    }

    diff = mean - *reflex;


    // Update of the weight using the given learning rule

    if (mean_roll > 0)
    {
        weight_roll[0] += learning_rate*mean_roll*diff;
    }
    else
    {
        weight_roll[1] += learning_rate*mean_roll*diff;
    }

    weight_pitch[0] -= learning_rate*mean_pitch*diff;
    weight_pitch[1] -= learning_rate*mean_pitch*diff;

    *reflex = mean;
}

void WeightUpdateB4(float mean_pitch, float mean_roll, float limit, float learning_rate, float weight_roll[], float weight_pitch[], float * reflex, bool *reflex_ON)
{
    float diff {.0f};
    float mean {.0f};

    // If the value of the signal surpasses the reflex signal threshold, the update will be commited.
    // If not, there won't be any update
    // Possible interference in the learning process due to the sign of both angles
    if (abs(mean_roll) > limit){
        mean += abs(mean_roll);
    }
    if (abs(mean_pitch) > limit){
        mean += abs(mean_pitch);
    }

    if ((abs(mean_roll) > limit) & (abs(mean_pitch) > limit))
    {
        *reflex_ON = false;
    }
    else
    {
        *reflex_ON = true;
    }

    diff = mean - *reflex;


    // Update of the weight using the given learning rule

    weight_roll[0] += learning_rate*mean_roll*diff;
    weight_roll[1] += learning_rate*mean_roll*diff;

    if (mean_pitch > 0)
    {
        weight_pitch[0] -= learning_rate*abs(mean_pitch)*diff;
        weight_pitch[1] -= learning_rate*abs(mean_pitch)*diff;
    }
    else
    {
        weight_pitch[2] -= learning_rate*abs(mean_pitch)*diff;
        weight_pitch[3] -= learning_rate*abs(mean_pitch)*diff;
    }

    *reflex = mean;
}


void PrintWeight(float weight_1[], float weight_2[])
{   
    std::cout << "Roll weights:  " << weight_1[0] << "  " << weight_1[1] << std::endl;
    std::cout << "Pitch weights:  " << weight_2[0] << "  " << weight_2[1] << std::endl;
}

void SpeedSaturation1(float * extra, float limit, int speed[], int dir[])
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

float * ExtraCalculation(float pitch, float roll, int speed[], float weight_roll[], float weight_pitch[], float limit, int dir[])
{
    int quadrant {0};
    float * extra;
    extra[0] = 0;
    extra[1] = 0;
    float weight_roll_L, weight_roll_R, weight_pitch_L, weight_pitch_R;

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Detection of the quadrant
    
    if (pitch > 0)
    {
        if (roll > 0)
        {
            quadrant = 1;
            weight_roll_L = -weight_roll[0];
            weight_roll_R = weight_roll[1];
            weight_pitch_L = weight_pitch[0];
            weight_pitch_R = weight_pitch[1];
        }
        else
        {
            quadrant = 2;
            weight_roll_L = weight_roll[0];
            weight_roll_R = -weight_roll[1];
            weight_pitch_L = weight_pitch[0];
            weight_pitch_R = weight_pitch[1];
        }
    }
    else
    {
        if (roll < 0)
        {
            quadrant = 3;
            weight_roll_L = weight_roll[0];
            weight_roll_R = -weight_roll[1];
            weight_pitch_L = weight_pitch[2];
            weight_pitch_R = weight_pitch[3];
        }
        else
        {
            weight_roll_L = -weight_roll[0];
            weight_roll_R = weight_roll[1];
            weight_pitch_L = weight_pitch[2];
            weight_pitch_R = weight_pitch[3];
        }
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Calculation of the speed
    extra[0] = roll*weight_roll_R + pitch*weight_pitch_R;
    extra[1] = roll*weight_roll_L + pitch*weight_pitch_L;

    SpeedSaturation1(extra, limit, speed, dir);
    
    return extra;
}

void InitialFilter(float * roll, float * pitch, matrix_hal::IMUData imu_data, matrix_hal::GPIOControl gpio, matrix_hal::IMUSensor imu_sensor, float sampling_time, float cutoff)
{

    auto begin = std::chrono::high_resolution_clock::now();
    auto end = std::chrono::high_resolution_clock::now();

    for (int i = 0; i < 10; i++)
    {
        // Capture the initial moment of the iteration
        begin = std::chrono::high_resolution_clock::now();

        // Overwrites imu_data with new data from IMU sensor
        imu_sensor.Read(&imu_data);

        // Calculate both roll and pitch based on the new reading
        //floa LowPassFilter(float,                 float,  float, float);
        *roll = LowPassFilter(sampling_time/1000.0f, cutoff, *roll,   imu_data.roll);
        *pitch = LowPassFilter(sampling_time/1000.0f, cutoff, *pitch, imu_data.pitch);

        //Capture the final moment of the iteration
        end = std::chrono::high_resolution_clock::now();

        // Check how much time you have to wait to accomplish the sampling time.
        std::this_thread::sleep_for(std::chrono::milliseconds((int)sampling_time) - std::chrono::duration_cast<std::chrono::milliseconds>(end - begin));
    }
}

void PrintWeight(float weight_1[], float weight_2[])
{   
    std::cout << "Roll weights:  " << weight_1[0] << "  " << weight_1[1] << std::endl;
    std::cout << "Pitch weights:  " << weight_2[0] << "  " << weight_2[1] << std::endl;
}

int CheckQuadrant(float pitch, float roll)
{
    if (pitch > 0)
    {
        if (roll > 0)
        {
            return 1;
        }
        else
        {
            return 2;
        }
    }
    else
    {
        if (roll < 0)
        {
            return 3;
        }
        else
        {
            return 4;
        }
    }
}

void WeightUpdateRobot(float roll, float pitch , float weight_roll[] , float weight_pitch[] , float learning_rate, int quadrant, float * reflex)
{

    float diff {0}, reduction_factor {0.01f}, new_reflex {0};

    new_reflex = abs(roll) + abs(pitch);

    diff = reduction_factor*new_reflex - *reflex;

    weight_roll[0] += learning_rate*diff*roll;
    weight_roll[1] += learning_rate*diff*roll;

    if ((quadrant == 1) | (quadrant == 2))
    {
        weight_pitch[0] += learning_rate*diff*pitch;
        weight_pitch[1] += learning_rate*diff*pitch;
    }
    else
    {
        weight_pitch[2] += learning_rate*diff*pitch;
        weight_pitch[3] += learning_rate*diff*pitch;
    }

    *reflex = reduction_factor*new_reflex;
}
