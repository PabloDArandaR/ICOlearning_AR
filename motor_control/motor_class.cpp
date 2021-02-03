#include "motor_class.hpp"

#include <iostream>
#include <unistd.h>
#include <matrix_hal/gpio_control.h>
#include <matrix_hal/matrixio_bus.h>

//#include <pigpio.h>
//#include <cmath>

// GPIO via Matrix Voice
#define  TB6612_RIGHT_MOTOR_PWMA        15 // (Orange)
#define  TB6612_LEFT_MOTOR_PWMB         14 // (Green)
#define  TB6612_RIGHT_MOTOR_AIN1        13 // (Blue)
#define  TB6612_RIGHT_MOTOR_AIN2        12 // (Brown)
#define  TB6612_LEFT_MOTOR_BIN1         11 // (Grey)
#define  TB6612_LEFT_MOTOR_BIN2         10 // (Pink)

using namespace std;

Motor::Motor(){
	motor_PWM = 0;
	motor_IN1 = 1;
	motor_IN2 = 3;
}

Motor::Motor(int PWM, int IN1, int IN2, matrix_hal::MatrixIOBus BUS, matrix_hal::GPIOControl GPIO){
	motor_PWM = PWM;
	motor_IN2 = IN2;
	motor_IN1 = IN1;
	motor_bus = BUS;
	motor_gpio = GPIO;
	initGPIOPins();
}

void Motor::initGPIOPins()
{
	*motor_gpio->SetMode(motor_PWM,1); //Pin mode as output
    *motor_gpio->SetFunction(motor_PWM,1); // Pin function as PWM
    *motor_gpio->SetMode(motor_IN1,1);
    *motor_gpio->SetMode(motor_IN2,1);
}

// Set speed and direction of LEFT motor
// Directiom -> 1 = forward, 0 = reverse
// Speed -> 0-100% in steps of 1%
void Motor::setMotorSpeedDirection(int speed, int dir)
{
	if (dir == 0) // Reverse
        {
                *motor_gpio->SetGPIOValue(motor_IN1,0); // Rotate left motor clockwise
                *motor_gpio->SetGPIOValue(motor_IN2,1);
        }
    if ( dir == 1 ) // Forward
        {
                *motor_gpio->SetGPIOValue(TB6612_LEFT_MOTOR_BIN1,1); // Rotate left motor clockwise
                *motor_gpio->SetGPIOValue(TB6612_LEFT_MOTOR_BIN2,0);
        }

	// Set motor speed via PWM signal (min. = 0, max. = 100)
        if (speed > 100)
                speed = 100;
        if (speed < 0)
                speed = 0;

	*motor_gpio->SetPWM(1000,speed,motor_PWM);
}