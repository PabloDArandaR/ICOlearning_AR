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
    
    // Initialize bus and exit program if error occurs
    if (!bus.Init())
	return false;

    // Create GPIOControl object
	matrix_hal::GPIOControl gpio;

	// Set gpio to use MatrixIOBus bus
	gpio.Setup(&bus);

    Motor left(PWM = TB6612_LEFT_MOTOR_PWMB, IN1 = TB6612_LEFT_MOTOR_BIN1, IN2 = TB6612_LEFT_MOTOR_BIN2, BUS = bus, GPIO = gpio);
    Motor right(PWM = TB6612_RIGHT_MOTOR_PWMB, IN1 = TB6612_RIGHT_MOTOR_AIN1, IN2 = TB6612_RIGHT_MOTOR_AIN2, BUS = bus, GPIO = gpio);

    left.setMotorSpeedDirection(speed = 50, dir = 1);

    std::cin.get();

    left.setMotorSpeedDirection(speed = 50, dir = 0);

    std::cin.get();

    right.setMotorSpeedDirection(speed = 50, dir = 1);

    std::cin.get();

    right.setMotorSpeedDirection(speed = 50, dir = 0);

    std::cin.get();

    return 0;
}