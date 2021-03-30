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


void WeightUpdate1(float mean, float limit, float learning_rate, float weight[], float * reflex, bool *reflex_ON)
{
    float diff {.0f};

    if (mean > limit){
        diff = mean - *reflex;
        weight[0] += learning_rate*mean*diff;
        
        *reflex = mean;
        *reflex_ON = true;
    }
    else if (mean < -limit){
        diff = mean - *reflex;
        weight[1] += learning_rate*mean*diff;

        *reflex = mean;
        *reflex_ON = true;
    }
    else 
    {
        *reflex = 0.0f;
        *reflex_ON = false;
    }
}

void WeightUpdate2(float mean, float limit, float learning_rate, float weight[], float * reflex, bool *reflex_ON)
{
    float diff {.0f};

    diff = mean - *reflex;
    weight[0] += learning_rate*mean*diff;
    weight[1] -= learning_rate*mean*diff;

    if (abs(mean) < limit)
    {
        *reflex = 0;
        *reflex_ON = false;
    }
    else
    {
        *reflex = mean;
        *reflex_ON = true;
    }
}

void WeightUpdate3(float mean, float limit, float learning_rate, float weight[], float * reflex, bool *reflex_ON)
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

void SpeedSaturation1(float * extra, float limit, int speed, int dir[])
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

void SpeedSaturation2(float * extra, float limit, int speed, int dir[])
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

