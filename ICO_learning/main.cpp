#include <iostream>
#include <fstream>
#include "../motor_control/motor_class.hpp"
#include <cstring>
#include <chrono>
#include "functions/training_functions.cpp"


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
/// 0 -> left ; 1 -> right


int main(int argc, char* argv[]) {

    std::cout << "In the main function " << std::endl;

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///// Declaring global variables
    bool training;
    int speed[2]; 
    char next;
    float roll, pitch, yaw, learning_rate;
    float weight_roll[2];

    // Create MatrixIOBus object for hardware communication
	matrix_hal::MatrixIOBus bus;
    
    // Initialize bus and exit program if error occurs
    if (!bus.Init())
    {
	    return false;
    }
    
    // Create GPIOControl object
	matrix_hal::GPIOControl gpio;
    // Create IMUData object
    matrix_hal::IMUData imu_data;
    // Create IMUSensor object
    matrix_hal::IMUSensor imu_sensor;
    
    std::cout << "Variables declared" << std::endl;

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///// Initialize the variables
    training = true;
    next = 'y';

	// Set gpio to use MatrixIOBus bus
	gpio.Setup(&bus);
    std::cout << " After setup bus" <<std::endl;
    // Set imu_sensor to use MatrixIOBus bus
    imu_sensor.Setup(&bus);
    std::cout << " After setup imu_sensor" <<std::endl;
    // Overwrites imu_data with new data from IMU sensor
    imu_sensor.Read(&imu_data);
    std::cout << " Sensor variables started" <<std::endl;


    // Yaw, Pitch, Roll Output
    yaw = imu_data.yaw;
    pitch = imu_data.pitch;
    roll = imu_data.roll;

    // Weights:
    weight_roll[0] = 0;
    weight_roll[1] = 0;

    // Speeds and learning rate
    speed[0] = 0;
    speed[1] = 0;
    learning_rate = 0;

    if (argc == 2)
    {
        for (int i = 0; i < strlen(argv[1]) ; i++){
            speed[0] = speed[0]*10 + ((int)argv[1][i] - 48);
            speed[1] = speed[1]*10 + ((int)argv[1][i] - 48);
        }
    }
    else if (argc == 3)
    {
        std::cout << "In argc == 3" << std::endl;
        
        for (int i = 0; i < strlen(argv[1]) ; i++){
            speed[0] = speed[0]*10 + ((int)argv[1][i] - 48);
            speed[1] = speed[1]*10 + ((int)argv[1][i] - 48);
        }
        
        bool after_comma {false};
        int position_of_comma = 0;
        int ending_position = strlen(argv[2]) - 1;
        for (int i = 0; i < strlen(argv[2]) ; i++){
            if (!after_comma){
                if ((argv[2][i] == '.') | (argv[2][i] == ',')){
                    after_comma = true;
                    position_of_comma = i;
                }
            }
            else{
                learning_rate = learning_rate*10 + ((float)((int)argv[2][i] - 48));
            }
        }
        
        for (int i = 0; i < (ending_position - position_of_comma) ; i ++){
            learning_rate *= 0.1;
        }
    }
    else
    {
        speed[0] = 50;
        speed[1] = 50;
    }
    
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///// Declaring the motor variables
    Motor left(TB6612_LEFT_MOTOR_PWMB, TB6612_LEFT_MOTOR_BIN1, TB6612_LEFT_MOTOR_BIN2, &gpio);
    Motor right(TB6612_RIGHT_MOTOR_PWMA, TB6612_RIGHT_MOTOR_AIN1, TB6612_RIGHT_MOTOR_AIN2, &gpio);

    std::cout << "Before the loop" << std::endl;

    while (training){

        switch(next){
            case 'y':
                //Introduce learning code
                train_roll(left, right, imu_data, weight_roll, learning_rate, speed, gpio, imu_sensor);
                std::cout << "Out of the training function" << std::endl;
                next = '?';
                break;

            case 'n':
                training = false;
                break;

            case '?':
                bool correct = false;
                
                std::cout << "IN ? case" << std::endl;
                
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
}
