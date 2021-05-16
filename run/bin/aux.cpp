#include <fstream>
#include <iostream>

// Interfaces with IMU sensor
#include "matrix_hal/imu_sensor.h"
// Holds data from IMU sensor
#include "matrix_hal/imu_data.h"
// Communicates with MATRIX device
#include "matrix_hal/matrixio_bus.h"


void readWeights(std::string filename, float * weight_roll, float * weight_pitch)
{
    std::ifstream file;
    std::string str;

    file.open(filename);

    std::getline(file, str);
    weight_roll[0] = std::stof(str);
    weight_roll[1] = std::stof(str);
    std::getline(file, str);
    weight_pitch[0] = std::stof(str);
    weight_pitch[1] = std::stof(str);
    std::getline(file, str);
    weight_pitch[2] = std::stof(str);
    weight_pitch[3] = std::stof(str);

}

float LowPassFilter(float sampling_time, float cutoff_frequency, float signal, float new_value)
{
    // alpha term characteristic of the Low Pass Filter
    float alpha {sampling_time*cutoff_frequency/(1 + sampling_time*cutoff_frequency)};

    float new_signal;
    
    //Calculation of the new signal    
    new_signal = signal*(1 - alpha) + new_value*alpha;

    return new_signal;
}

float ExtraL(float pitch, float roll, int speed[], float weight_roll[], float weight_pitch[], float limit, int dir[])
{
    int quadrant {0};
    float extra;
    extra = 0;
    float weight_roll_L, weight_pitch_L;

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Detection of the quadrant

    if (pitch > 0)
    {
        if (roll < 0)
        {
            quadrant = 1;
            weight_roll_L = -weight_roll[0];
            weight_pitch_L = weight_pitch[0];
        }
        else
        {
            quadrant = 2;
            weight_roll_L = weight_roll[0];
            weight_pitch_L = weight_pitch[0];
        }
    }
    else
    {
        if (roll > 0)
        {
            quadrant = 3;
            weight_roll_L = weight_roll[0];
            weight_pitch_L = weight_pitch[2];
        }
        else
        {
            quadrant = 4;
            weight_roll_L = -weight_roll[0];
            weight_pitch_L = weight_pitch[2];
        }
    }


    std::cout << "Quadrant in ExtraL is:  " << quadrant << std::endl;

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Knowing which weights to take into consideration
    
    if (abs(pitch) < 3)
    {
        weight_pitch_L = 0;
    }
    if (abs(roll) < 3)
    {
        weight_roll_L = 0;
    }
    
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Calculation of the speed

    extra = abs(roll)*weight_roll_L + abs(pitch)*weight_pitch_L;
    
    return extra;

}

float ExtraR(float pitch, float roll, int speed[], float weight_roll[], float weight_pitch[], float limit, int dir[])
{
    int quadrant {0};
    float extra;
    float  weight_roll_R, weight_pitch_R;

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Detection of the quadrant

    if (pitch > 0)
    {
        if (roll < 0)
        {
            quadrant = 1;
            weight_roll_R = weight_roll[1];
            weight_pitch_R = weight_pitch[1];
        }
        else
        {
            quadrant = 2;
            weight_roll_R = -weight_roll[1];
            weight_pitch_R = weight_pitch[1];
        }
    }
    else
    {
        if (roll > 0)
        {
            quadrant = 3;
            weight_roll_R = -weight_roll[1];
            weight_pitch_R = weight_pitch[3];
        }
        else
        {
            weight_roll_R = weight_roll[1];
            weight_pitch_R = weight_pitch[3];
        }
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Knowing which weights to take into consideration
    
    
    if (abs(pitch) < 3)
    {
        weight_pitch_R = 0;
    }
    if (abs(roll) < 3)
    {
        weight_roll_R = 0;
    }
    

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Calculation of the speed

    extra = abs(roll)*weight_roll_R + abs(pitch)*weight_pitch_R;
    
    return extra;
}

void SpeedSaturation1(float * extra, float limit, int speed[], int dir[])
{
    for (int i = 0; i < sizeof(extra)/sizeof(extra[0]); i++)
    {
        if (extra[i] < 0){
            extra[i] = abs(extra[i]);
            std::cout << "-------------------------- IN HERE --------------------------" << std::endl;
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

void SpeedSaturation2(float * extra, float limit, int speed[], int dir[])
{
    for (int i = 0 ; i < 2; i++)
    {
        if (extra[i] < 0 )
        {
            extra[i] = - extra[i];
            dir[i] = 0;
        }
    }
}

void InitialFilter(float * roll, float * pitch, matrix_hal::IMUData imu_data, matrix_hal::GPIOControl gpio, matrix_hal::IMUSensor imu_sensor, float sampling_time, float cutoff)
{

    auto begin = std::chrono::high_resolution_clock::now();
    auto end = std::chrono::high_resolution_clock::now();

    for (int i = 0; i < 50; i++)
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




