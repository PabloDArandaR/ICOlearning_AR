#include "motor_class.hpp"
#include <iostream>
#include <chrono>
#include <thread>

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

// Function declaration in header file
void Setup(MatrixIOBus *bus);
// Function declaration in header file
bool Read(IMUData *data);

uint16_t GetGPIOValues();

int main() {

    // Create MatrixIOBus object for hardware communication
	matrix_hal::MatrixIOBus bus;
    int speed {0};
    int dir {0};
    
    // Initialize bus and exit program if error occurs
    if (!bus.Init())
    {
	    return false;
    }

    // Create GPIOControl object
	matrix_hal::GPIOControl gpio;

    // Create IMUData object
    matrix_hal::IMUData imu_data;

	// Set gpio to use MatrixIOBus bus
	gpio.Setup(&bus);
    // Set imu_sensor to use MatrixIOBus bus
    imu_sensor.Setup(&bus);

    // Overwrites imu_data with new data from IMU sensor
    imu_sensor.Read(&imu_data);

    // Declaring the motor variables
    Motor left(TB6612_LEFT_MOTOR_PWMB, TB6612_LEFT_MOTOR_BIN1, TB6612_LEFT_MOTOR_BIN2, &gpio);
    Motor right(TB6612_RIGHT_MOTOR_PWMA, TB6612_RIGHT_MOTOR_AIN1, TB6612_RIGHT_MOTOR_AIN2, &gpio);

    left.setMotorSpeedDirection(&gpio, 100, 1);
    right.setMotorSpeedDirection(&gpio, 100, 1);

    auto start = std::chrono::high_resolution_clock::now();
    auto end   = std::chrono::high_resolution_clock::now();
    std::chrono::duration<float> duration = end-start;

    // Accelerometer
    float accel_X = imu_data.accel_x;
    float accel_Y = imu_data.accel_y;
    float accel_Z = imu_data.accel_z;
    // Gyroscope
    float gyro_X = imu_data.gyro_x;
    float gyro_Y = imu_data.gyro_y;
    float gyro_Z = imu_data.gyro_z;
    // Yaw, Pitch, Roll Output
    float yaw = imu_data.yaw;
    float pitch = imu_data.pitch;
    float roll = imu_data.roll;
    // Magnetometer
    float mag_X = imu_data.mag_x;
    float mag_Y = imu_data.mag_y;
    float mag_Z = imu_data.mag_z;

    float minimum = -45.0f;
    float maximum = 45.0f;

    while ((imu_data.pitch > minimum ) && (imu_data.pitch < maximum)){
        // Overwrites imu_data with new data from IMU sensor
        imu_sensor.Read(&imu_data);
        std::cout << "Value of pitch : " << imu_data.pitch << std::endl;
    }

    //std::this_thread::sleep_for(std::chrono::seconds(10));

    left.setMotorSpeedDirection(&gpio, 0, 1);
    right.setMotorSpeedDirection(&gpio, 0, 1);

}
