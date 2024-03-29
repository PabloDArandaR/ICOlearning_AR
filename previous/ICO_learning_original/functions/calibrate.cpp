#include <iostream>
#include <fstream>
#include "../../motor_control/motor_class.hpp"
#include <cstring>
#include <chrono>
#include "templates.hpp"
#include "variety.hpp"
#include "calibrate.hpp"
#include <vector>

// Interfaces with IMU sensor
#include "matrix_hal/imu_sensor.h"
// Holds data from IMU sensor
#include "matrix_hal/imu_data.h"
// Communicates with MATRIX device
#include "matrix_hal/matrixio_bus.h"

float BiasRoll(matrix_hal::IMUData & imu_data, matrix_hal::GPIOControl gpio, matrix_hal::IMUSensor imu_sensor, int n_sample, float sampling_time, float cutoff)
{
    float bias {.0f};
    std::vector<float> sample;
    std::vector<float> sorted_sample;
    auto start = std::chrono::high_resolution_clock::now();
    auto finish = std::chrono::high_resolution_clock::now();

    // Resize the sample storing vector to a vector with the adequate number of spaces
    sample.resize(n_sample);

    // Sampling
    for (int i = 0; i < n_sample; i++)
    {
        start = std::chrono::high_resolution_clock::now();
        // Overwrites imu_data with new data from IMU sensor
        imu_sensor.Read(&imu_data);

        sample[i] = LowPassFilter(sampling_time, cutoff , sample[i-1],imu_data.roll);

        finish = std::chrono::high_resolution_clock::now();

        std::this_thread::sleep_for( std::chrono::milliseconds((int)sampling_time) - std::chrono::duration_cast<std::chrono::milliseconds>(finish - start));
    }
    
    //Sort the vector and return Median
    sorted_sample = sort_vector(sample);

    bias = Median(sorted_sample);

    return bias;
}

// The same as BiasRoll but with the Pitch Signal
float BiasPitch(matrix_hal::IMUData & imu_data, matrix_hal::GPIOControl gpio, matrix_hal::IMUSensor imu_sensor, int n_sample, float sampling_time, float cutoff)
{
    float bias {.0f};
    std::vector<float> sample,sorted_sample;
    auto start = std::chrono::high_resolution_clock::now();
    auto finish = std::chrono::high_resolution_clock::now();
    

    sample.resize(n_sample);

    for (int i = 0; i < n_sample; i++)
    {
        start = std::chrono::high_resolution_clock::now();
        // Overwrites imu_data with new data from IMU sensor
        imu_sensor.Read(&imu_data);

        sample[i] = imu_data.pitch;

        finish = std::chrono::high_resolution_clock::now();

        std::this_thread::sleep_for(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::milliseconds((int)sampling_time) - (finish - start)));
    }

    sorted_sample = sort_vector(sample);

    bias = Median(sorted_sample);

    return bias;
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
