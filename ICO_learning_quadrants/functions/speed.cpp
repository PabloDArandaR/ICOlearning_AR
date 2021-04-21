#include "speed.hpp"
#include "aux.hpp"
#include "calibrate.hpp"

// Interfaces with IMU sensor
#include "matrix_hal/imu_sensor.h"
// Holds data from IMU sensor
#include "matrix_hal/imu_data.h"
// Communicates with MATRIX device
#include "matrix_hal/matrixio_bus.h"

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
