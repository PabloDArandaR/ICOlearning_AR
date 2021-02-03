#include "motor_class.hpp"
#include <iostream>

// GPIO via Matrix Creator
#define  TB6612_RIGHT_MOTOR_PWMA        15 // (Orange)
#define  TB6612_LEFT_MOTOR_PWMB         14 // (Green)
#define  TB6612_RIGHT_MOTOR_AIN1        13 // (Blue)
#define  TB6612_RIGHT_MOTOR_AIN2        12 // (Brown)
#define  TB6612_LEFT_MOTOR_BIN1         11 // (Grey)
#define  TB6612_LEFT_MOTOR_BIN2         10 // (Pink)

int main() {

    // Create MatrixIOBus object for hardware communication
	matrix_hal::MatrixIOBus bus;
    int speed {0};
    int dir {0};
    
    // Initialize bus and exit program if error occurs
    if (!bus.Init())
	return false;

    // Create GPIOControl object
	matrix_hal::GPIOControl gpio;

	// Set gpio to use MatrixIOBus bus
	gpio.Setup(&bus);

    Motor left(TB6612_LEFT_MOTOR_PWMB, TB6612_LEFT_MOTOR_BIN1, TB6612_LEFT_MOTOR_BIN2, bus, gpio);
    Motor right(TB6612_RIGHT_MOTOR_PWMB, TB6612_RIGHT_MOTOR_AIN1, TB6612_RIGHT_MOTOR_AIN2, bus, gpio);

    speed = 50;
    dir = 1;

    left.setMotorSpeedDirection(speed, dir);

    std::cin.get();

    speed = 50;
    dir = 0;

    left.setMotorSpeedDirection(speed, dir);

    std::cin.get();

    speed = 50;
    dir = 1;

    right.setMotorSpeedDirection(speed , dir);

    std::cin.get();

    speed = 50;
    dir = 0;

    right.setMotorSpeedDirection(speed , dir);

    std::cin.get();

    return 0;
}