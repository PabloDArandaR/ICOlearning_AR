#include <iostream>
#include <fstream>
#include "../motor_control/motor_class.hpp"
#include <cstring>

// Interfaces with IMU sensor
#include "matrix_hal/imu_sensor.h"
// Holds data from IMU sensor
#include "matrix_hal/imu_data.h"
// Communicates with MATRIX device
#include "matrix_hal/matrixio_bus.h"

// GPIO via Matrix Creator
#define  TB6612_RIGHT_MOTOR_PWMA        15 // (Orange)
#define  TB6612_LEFT_MOTOR_PWMB         14 // (Green)
#define  TB6612_RIGHT_MOTOR_AIN1        13 // (Blue)
#define  TB6612_RIGHT_MOTOR_AIN2        12 // (Brown)
#define  TB6612_LEFT_MOTOR_BIN1         11 // (Grey)
#define  TB6612_LEFT_MOTOR_BIN2         10 // (Pink)

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///// Function declarations
template <typename T>
T mean( T array_of_values[]){
    T mean {0};
    int number_of_values {sizeof(array_of_values)/sizeof(array_of_values[0])};

    for (int i = 0; i < number_of_values; i++)
    {
        mean += array_of_values[i];
    }


    return mean/(float)number_of_values;
}

template <typename T>
void roll_and_add(T add, T array_of_values[]){
    int number_of_values {sizeof(array_of_values)/sizeof(array_of_values[0])};

    for (int i = (number_of_values - 1); i >= 0 ; i --){
        std::cout << "Value in position i+1: " << array_of_values[i + 1] << std::endl;
        std::cout << "Value in position i: "   << array_of_values[i] << std::endl;
        array_of_values[i + 1] = array_of_values[i];

        std::cout << "-------------------------------------------------------------------------------------------------" << std::endl;
        
     }
    array_of_values[0] = add;
}

template <typename T>
void print_array(T array_of_values []){
    int number_of_values {sizeof(array_of_values) / (sizeof(array_of_values[0]))};

    for (int i = 0; i <= number_of_values; i++){
        std::cout <<  array_of_values[i] << "  ";
    }

    std::cout << std::endl;
}

void train_roll(std::ofstream file, Motor left, Motor right, matrix_hal::IMUData imu_data, float & weight_roll_R, float & weight_roll_L, float learning_rate, int speed_L, int speed_R)
{
    //Variables required for the different calculations:

    bool round {true};
    float roll_data[5];
    int mean_roll {0};
    int dir_L {1};
    int dir_R {1};

    //Stabilize measurements part:

    for (int i = 0; i < sizeof(roll_data)/sizeof(roll_data[0]); i++){
        roll_data[i] = imu_data.roll;
    }

    mean_roll = mean(roll_data);

    //Learning part
    while (round){
        float reflex {0};
        float diff {0};
        float extra_R {0}, extra_L {0};

        roll_and_add(imu_data.roll, roll_data);
        mean_roll = mean(roll_data);

        if (abs(mean_roll) > 15.0f){
            round = false;
            break;
        }

        //Reflex signal calculation -> Depends on the roll angle sign to see which weight is updated

        if (mean_roll > 2.0f){
            diff = mean_roll - reflex;
            weight_roll_R += learning_rate*mean_roll*diff;

            reflex = mean_roll;
        }
        else if (mean_roll < -2.0f){
            diff = mean_roll - reflex;
            weight_roll_L += learning_rate*mean_roll*diff;

            reflex = mean_roll;
        }
        else {
            reflex = 0;
        }

        extra_L = weight_roll_L*mean_roll;
        extra_R = weight_roll_R*mean_roll;

        if ((speed_L + extra_L) > 100)
        {
            extra_L = 100 - speed_L;
            dir_L = 1;
        }
        else if ((speed_L + extra_L) < 0){
            extra_L = abs(extra_L) - speed_L;
            dir_L = 0;
        }
        else{
            dir_L = 1;
        }

        if ((speed_R + extra_R) > 100)
        {
            extra_R = 100 - speed_R;
            dir_R = 1;
        }
        else if ((speed_R + extra_R) < 0){
            extra_R = abs(extra_R) - speed_R;
            dir_R = 0;
        }
        else{
            dir_R = 1;
        }

        left.setMotorSpeedDirection(&gpio, speed_L + extra_L, dir_L);
        right.setMotorSpeedDirection(&gpio, speed_R + extra_R, dir_R);

        file >> weight_roll_L >> " " >> weight_roll_R >> std::endl;
    }
}

int main(int argc, char* argv[]) {

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///// Declaring global variables
    bool training;
    std::ofstream evolution;
    int speed_L, speed_R;
    char next;
    float roll, pitch, yaw, weight_roll;

    // Create MatrixIOBus object for hardware communication
	matrix_hal::MatrixIOBus bus;
    // Create GPIOControl object
	matrix_hal::GPIOControl gpio;
    // Create IMUData object
    matrix_hal::IMUData imu_data;
    // Create IMUSensor object
    matrix_hal::IMUSensor imu_sensor;

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///// Initialize the variables
    training = true;
    evolution.open("evolution.txt");
    next = 'y';
    weight_roll = 0.0f;

	// Set gpio to use MatrixIOBus bus
	gpio.Setup(&bus);
    // Set imu_sensor to use MatrixIOBus bus
    imu_sensor.Setup(&bus);
    // Overwrites imu_data with new data from IMU sensor
    imu_sensor.Read(&imu_data);

    // Yaw, Pitch, Roll Output
    float yaw = imu_data.yaw;
    float pitch = imu_data.pitch;
    float roll = imu_data.roll;

    // Initialize bus and exit program if error occurs
    if (!bus.Init())
    {
	    return false;
    }

    if (argc == 2)
    {
        for (int i = 0; i < strlen(argv[1]) ; i++){
            speed_L = speed_L*10 + ((int)argv[1][i] - 48);
            speed_R = speed_R*10 + ((int)argv[1][i] - 48);
        }
    }
    else if (argc == 3)
    {
        for (int i = 0; i < strlen(argv[1]) ; i++){
            speed_L = speed_L*10 + ((int)argv[1][i] - 48);
        }
        for (int i = 0; i < strlen(argv[2]) ; i++){
            speed_R = speed_R*10 + ((int)argv[2][i] - 48);
        }
    }
    else
    {
        int speed_L = 50;
        int speed_R = 50;
    }
    
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///// Declaring the motor variables
    Motor left(TB6612_LEFT_MOTOR_PWMB, TB6612_LEFT_MOTOR_BIN1, TB6612_LEFT_MOTOR_BIN2, &gpio);
    Motor right(TB6612_RIGHT_MOTOR_PWMA, TB6612_RIGHT_MOTOR_AIN1, TB6612_RIGHT_MOTOR_AIN2, &gpio);

    while (training){

        switch(next){
            case 'y':
                //Introduce learning code
                train_roll(evolution, left, right, imu_data)

                next = '?';
                break;

            case 'n':
                training = false;
                break;

            case '?':
                bool correct = true;
                while (!correct){
                    std::cout << "Keep training?(y/n) " << std::endl;
                    std::cin >> next;
                    if ((next == 'Y') | (next == 'y')){
                        next = 'y';
                        correct = true;
                    }
                    if ((next == 'N') | (next == 'n')){
                        next = 'n';
                        correct = true;
                    }
                    else{
                        std::cout << "Inadequate response. Again: " << std::endl;
                    }
                }
        }

    }

    evolution.close();

}

