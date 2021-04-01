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
    bool training;                                                  // Stablish if the training should continue
    int speed[2];                                                   // Stores the value of base speed
    char next;                                                      // Switch case variable
    float roll, pitch, learning_rate, limit_roll;                   // Variables related to some learning parameters
    float weight_roll[2], weight_pitch[2];                          // Stores the weights related to each one of the signals taken into consideration
    int update_method, iteration;                                   // Selection of the update function and number of training sessions done
    float sampling_time, cutoff;                                    // Sampling time and cutoff frequency. Necessary for the Low Pass Filter
    std::ofstream file;                                             // File to store data    
    auto begin = std::chrono::high_resolution_clock::now();         // Beginning of the program


    // Put the header of the data files
    file.open("evolution_roll.txt", std::ios_base::trunc);
    file << "Weight_roll 0" << "," << "Weight_roll 1" << "," << "Roll raw" << "," << "Roll filtered" << "," << "speed 0" << "," << "speed 1" << "," << "reflex" << "Time" << "," << "Reflex ON" << "," << "iteration\n" ;
    file.close();
    file.open("evolution_pitch.txt", std::ios_base::trunc);
    file << "Weight_pitch 0" << "," << "Weight_pitch 1" << "," << "pitch raw" << "," << "pitch filtered" << "," << "speed 0" << "," << "speed 1" << "," << "reflex" << "Time" << "," << "Reflex ON" << "," << "iteration\n" ;
    file.close();
    file.open("evolution_both.txt", std::ios_base::trunc);
    file << "Weight_roll 0" << "," << "Weight_roll 1" << "," << "Weight_pitch 0" << "," << "Weight_pitch 1" << "," << "Roll raw" << "," << "Roll filtered" << "," << "Pitch raw" << "," << "Pitch filtered" << "," << "speed 0" << "," << "speed 1" << "," << "reflex" << "Time" << "," << "Reflex ON" << "," << "iteration\n" ;
    file.close();

    /////////////////////////////////////////////////////////////////////////////////////////////////
    // Initialization of Matrix Creator related variables and initialization

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
    
	// Set gpio to use MatrixIOBus bus
	gpio.Setup(&bus);
    // Set imu_sensor to use MatrixIOBus bus
    imu_sensor.Setup(&bus);
    // Overwrites imu_data with new data from IMU sensor
    imu_sensor.Read(&imu_data);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///// Initialize the variables
    training = true;
    next = '1';             // Start in training pitch case

    // Pitch, Roll Output
    pitch = imu_data.pitch;
    roll = imu_data.roll;

    // Weights:
    weight_roll[0] = 0;
    weight_roll[1] = 0;
    weight_pitch[0]= 0;
    weight_pitch[1]= 0; 

    // Limit:
    std::cout << "Insert the limit roll angle value: ";
    std::cin >> limit_roll;

    // Introduce cutoff frequency
    std::cout << "Cutoff frequency? (rad/s) " ;
    std::cin >> cutoff;
    if ( cutoff <= 0)
    {
        std::cout << "Wrong value -> Repeat: ";
        std::cin >> cutoff;
    }

    // Introduce Sampling time
    std::cout << "sampling_time? (ms)  " ;
    std::cin >> sampling_time;

    if ( sampling_time <= 0)
    {
        std::cout << "Wrong value -> Repeat: ";
        std::cin >> sampling_time;
    }


    // Speeds and learning rate
    speed[0] = 0;
    speed[1] = 0;
    learning_rate = 0;

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////7
    // If only 2 arguments are inserted (name of the file + int), the speed will be updated
    if (argc == 2)
    {
        for (int i = 0; i < strlen(argv[1]) ; i++){
            speed[0] = speed[0]*10 + ((int)argv[1][i] - 48);
            speed[1] = speed[1]*10 + ((int)argv[1][i] - 48);
        }
    }
    // If 3 arguments are inserted (name of the file + int + float), speed will be updated and learning rate read
    else if (argc == 3)
    {
        
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
    // Base speed -> 50
    else
    {
        speed[0] = 50;
        speed[1] = 50;
    }
    
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///// Declaring the motor variables (more info on the motor_control.cpp file)
    /// Ports in order: PWM signal, input 1, input 2, gpio address of the Matrix Creator
    Motor right(TB6612_LEFT_MOTOR_PWMB, TB6612_LEFT_MOTOR_BIN1, TB6612_LEFT_MOTOR_BIN2, &gpio);
    Motor left(TB6612_RIGHT_MOTOR_PWMA, TB6612_RIGHT_MOTOR_AIN1, TB6612_RIGHT_MOTOR_AIN2, &gpio);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///// Beginning of the training and running

    while (training){

        switch(next){
            case '1':               // Train the weights of the roll signal
            {
                // Update method(can be seen in the aux.cpp and aux.hpp files):

                while (( update_method < 1) & ( update_method > 3))
                {
                    std::cout << "Wrong method seleted. \n" ;
                    std::cout << "Again: \n";
                    std::cin >> update_method;
                    // update_method -= 48;
                    std::cout << "Value introduced: " << update_method << std::endl;
                }
                
                TrainRoll(left, right, imu_data, weight_roll, learning_rate, speed, gpio, imu_sensor, limit_roll, update_method, sampling_time, cutoff, &iteration, begin);
                next = '?';
                break;
            }

            case '2':
            {
                TrainBoth(left,right, imu_data, weight_roll, weight_pitch, learning_rate, speed, gpio, imu_sensor, limit_roll, update_method, sampling_time, cutoff, &iteration, begin);
                next = '?';
                break;
            }

            case '3':               // Run with the calculted weights
            {
                // Create Run function with the calculated weights for a given time
                Run(weight_roll, weight_pitch, left, right, imu_data, gpio, imu_sensor, sampling_time, cutoff, speed);
                next = '?';
                break;
            }

            case '4':                // Exit training
            {
                training = false;
                break;
            }

            case '?':                // Select option
            {
                bool correct = false;
                
                while (!correct){
                    std::cout << "Keep training roll?(1) " << std::endl;
                    std::cout << "Keep training both?(2) " << std::endl;
                    std::cout << "See robot with calculated weights? (3)" << std::endl;
                    std::cout << "Exit? (4)" << std::endl;
                    std::cin >> next;
                    if ((next == '1') | (next == '2') | (next == '3') | (next == '4'))
                    {
                        correct = true;
                    }
                    else
                    {
                        std::cout << "Inadequate response. Again: " << std::endl;
                    }
                }
                break;
            }
        }
    }
}


