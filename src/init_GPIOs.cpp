// System calls
#include <unistd.h>
// Input/output streams and functions
#include <iostream>

// Interfaces with GPIO
#include "matrix_hal/gpio_control.h"
// Communicates with MATRIX device
#include "matrix_hal/matrixio_bus.h"

#include "init_GPIOs.hpp"

void initGPIOPins(matrix_hal::GPIOControl* gpio){

    gpio->SetMode(R_MOTOR_PWM,1); //Pin mode -> output
    gpio->SetFunction(R_MOTOR_PWM,1); //Pin Function -> PWM
    gpio->SetMode(R_MOTOR_IN1,1);
    gpio->SetMode(R_MOTOR_IN2,1);

    gpio->SetMode(L_MOTOR_PWM,1); //Pin mode -> output
    gpio->SetFunction(L_MOTOR_PWM,1); //Pin Function -> PWM
    gpio->SetMode(L_MOTOR_IN1,1);
    gpio->SetMode(L_MOTOR_IN2,1);
}

