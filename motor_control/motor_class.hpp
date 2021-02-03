#include <iostream>
#include <unistd.h>
#include <matrix_hal/gpio_control.h>
#include <matrix_hal/matrixio_bus.h>

class Motor{

    public:
        Motor();
        Motor(int, int, int, matrix_hal::GPIOControl*);
        void initGPIOPins(matrix_hal::GPIOControl*);
        void setMotorSpeedDirection(int, int);


    private:
        int motor_PWM;
        int motor_IN1;
        int motor_IN2;

};