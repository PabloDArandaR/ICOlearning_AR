#include <iostream>
#include <unistd.h>
#include <matrix_hal/gpio_control.h>
#include <matrix_hal/matrixio_bus.h>

class Motor{

    public:
        Motor();
        Motor(int, int, int, matrix_hal::MatrixIOBus, matrix_hal::GPIOControl);
        void initGPIOPins();
        void setMotorSpeedDirection(int, int)


    private:
        int motor_PWM;
        int motor_IN1;
        int motor_IN2;
        matrix_hal::GPIOControl motor_gpio;
        matrix_hal::MatrixIOBus motor_bus;

};