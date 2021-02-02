// System calls
#include <unistd.h>
// Input/output streams and functions
#include <iostream>

// Interfaces with GPIO
#include "matrix_hal/gpio_control.h"
// Communicates with MATRIX device
#include "matrix_hal/matrixio_bus.h"

////////////////////////
// MOTOR PORTS //
//////////////////////

#define R_MOTOR_PWM 15 //(ORANGE)
#define L_MOTOR_PWM 16 //(GREEN)
#define R_MOTOR_IN1 13 //(ORANGE)
#define L_MOTOR_IN1 12 //(GREEN)
#define R_MOTOR_IN2 11 //(ORANGE)
#define L_MOTOR_IN2 10 //(GREEN)

void initGPIOPins(matrix_hal::GPIOControl*);
