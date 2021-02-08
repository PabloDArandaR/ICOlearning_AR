#include "motor_class.hpp"
#include <iostream>
#include <chrono>

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

    left.setMotorSpeedDirection(&gpio, 100, 0);
    right.setMotorSpeedDirection(&gpio, 100, 0);

    std::chrono::high_resolution_clock initial = std::chrono::high_resolution_clock::now();

    auto end = std::chrono::high_resolution_clock::now();

    while ((end-initial).count() < 1000){
        auto end = std::chrono::high_resolution_clock::now();
    }


    left.setMotorSpeedDirection(&gpio, 0, 0);
    right.setMotorSpeedDirection(&gpio, 0, 0);

}