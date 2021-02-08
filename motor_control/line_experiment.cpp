#include "motor_class.hpp"
#include <iostream>
#include <chrono>
#include <thread>

// GPIO via Matrix Creator
#define  TB6612_RIGHT_MOTOR_PWMA        15 // (Orange)
#define  TB6612_LEFT_MOTOR_PWMB         14 // (Green)
#define  TB6612_RIGHT_MOTOR_AIN1        13 // (Blue)
#define  TB6612_RIGHT_MOTOR_AIN2        12 // (Brown)
#define  TB6612_LEFT_MOTOR_BIN1         11 // (Grey)
#define  TB6612_LEFT_MOTOR_BIN2         10 // (Pink)

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

	// Set gpio to use MatrixIOBus bus
	gpio.Setup(&bus);

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

    while (yaw > -45f){
        float yaw = imu_data.yaw;
    }

    std::this_thread::sleep_for(std::chrono::seconds(10));


    left.setMotorSpeedDirection(&gpio, 0, 1);
    right.setMotorSpeedDirection(&gpio, 0, 1);

}
