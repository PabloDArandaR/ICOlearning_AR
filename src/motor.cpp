// System calls
#include <unistd.h>
// Input/output streams and functions
#include <iostream>

// Interfaces with GPIO
#include "matrix_hal/gpio_control.h"
// Communicates with MATRIX device
#include "matrix_hal/matrixio_bus.h"
// Interfaces with humidity sensor
#include "matrix_hal/humidity_sensor.h"
// Holds data from humidity sensor
#include "matrix_hal/humidity_data.h"


////////////////////////
// MOTOR PORTS //
//////////////////////

#define R_MOTOR_PWM 15 //(ORANGE)
#define L_MOTOR_PWM 16 //(GREEN)
#define R_MOTOR_IN1 13 //(ORANGE)
#define L_MOTOR_IN1 12 //(GREEN)
#define R_MOTOR_IN2 11 //(ORANGE)
#define L_MOTOR_IN2 10 //(GREEN)

// Set speed and direction of LEFT motor
// Directiom -> 1 = forward, 0 = reverse
// Speed -> 0-100% in steps of 1%
void setLeftMotorSpeedDirection(matrix_hal::GPIOControl* gpio, int speed, int dir)
{
	if (dir <= 0) // Reverse
        {
                gpio->SetGPIOValue(L_MOTOR_IN1,0); // Rotate left motor clockwise
                gpio->SetGPIOValue(L_MOTOR_IN2,1);
        }
        if ( (dir > 0)  || (dir >= 1) ) // Forward
        {
                gpio->SetGPIOValue(L_MOTOR_IN1,1); // Rotate left motor clockwise
                gpio->SetGPIOValue(L_MOTOR_IN2,0);
        }

	// Set motor speed via PWM signal (min. = 0, max. = 100)
        if (speed > 100)
                speed = 100;
        if (speed < 0)
                speed = 0;

	gpio->SetPWM(1000,speed,L_MOTOR_PWM);
}