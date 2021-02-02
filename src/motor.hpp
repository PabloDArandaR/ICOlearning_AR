// System calls
#include <unistd.h>
// Input/output streams and functions
#include <iostream>

// Interfaces with GPIO
#include "matrix_hal/gpio_control.h"
// Communicates with MATRIX device
#include "matrix_hal/matrixio_bus.h"



void setLeftMotorSpeedDIrection(matrix_hal::GPIOControl*, int, int);