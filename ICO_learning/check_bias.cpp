#include <iostream>
#include <fstream>
#include <cstring>
#include <chrono>
#include <thread>


#include "matrix_hal/gpio_control.h"
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

int main()
{
    int number_of_samples;
    std::ofstream file;
    auto finish = std::chrono::high_resolution_clock::now();
    auto start = std::chrono::high_resolution_clock::now();
    int sampling_time;

    // Create MatrixIOBus object for hardware communication
	matrix_hal::MatrixIOBus bus;
    
    // Initialize bus and exit program if error occurs
    if (!bus.Init())
    {
	    return false;
    }
    
    // Create GPIOControl object
	matrix_hal::GPIOControl gpio;
    // Create IMUData object
    matrix_hal::IMUData imu_data;
    // Create IMUSensor object
    matrix_hal::IMUSensor imu_sensor;
    
    std::cout << "Variables declared" << std::endl;

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///// Initialize the variables

	// Set gpio to use MatrixIOBus bus
	gpio.Setup(&bus);
    // Set imu_sensor to use MatrixIOBus bus
    imu_sensor.Setup(&bus);
    // Overwrites imu_data with new data from IMU sensor
    imu_sensor.Read(&imu_data);
    std::cout << " Sensor variables started" <<std::endl;
    // Initialize the file to be edited
    file.open("bias_check.csv");
    file << "#roll,pitch,yaw,a_x,a_y,a_z\n";

    number_of_samples = 100000;
    sampling_time = 10;

    for (int i = 0; i < number_of_samples; i++)
    {
        start = std::chrono::high_resolution_clock::now();

        // Overwrites imu_data with new data from IMU sensor
        imu_sensor.Read(&imu_data);

        file << imu_data.roll << "," << imu_data.pitch << "," << imu_data.yaw << "," << imu_data.accel_x << "," << imu_data.accel_y << "," << imu_data.accel_z << "\n";

        finish = std::chrono::high_resolution_clock::now();
        std::this_thread::sleep_for(std::chrono::milliseconds(sampling_time - std::chrono::duration_cast<std::chrono::milliseconds>(finish - start).count()));

        /*
        while (std::chrono::duration_cast<std::chrono::milliseconds>(finish - start).count() < sampling_time){
            finish = std::chrono::high_resolution_clock::now();
        }
        */
    }
    
}