#include <iostream>
#include <fstream>
#include "../motor_control/motor_class.hpp"
#include <cstring>
#include <chrono>
#include "functions/training_functions.hpp"


// Interfaces with IMU sensor
#include "matrix_hal/imu_sensor.h"
// Holds data from IMU sensor
#include "matrix_hal/imu_data.h"
// Communicates with MATRIX device
#include "matrix_hal/matrixio_bus.h"

using namespace std::chrono_literals;

// GPIO via Matrix Creator
#define  TB6612_LEFT_MOTOR_PWMB        15 // (Orange)
#define  TB6612_RIGHT_MOTOR_PWMA         14 // (Green)
#define  TB6612_LEFT_MOTOR_BIN1        13 // (Blue)
#define  TB6612_LEFT_MOTOR_BIN2        12 // (Brown)
#define  TB6612_RIGHT_MOTOR_AIN1         11 // (Grey)
#define  TB6612_RIGHT_MOTOR_AIN2         10 // (Pink)

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// 0 -> left ; 1 -> right


int main(int argc, char* argv[]) {

    std::cout << "In the main function " << std::endl;

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///// Declaring global variables
    bool training;                                                  // Stablish if the training should continue
    int speed[2];                                                   // Stores the value of base speed
    char next;                                                      // Switch case variable
    float roll, pitch, learning_rate, limit, threshold;                        // Variables related to some learning parameters
    float weight_roll[2], weight_pitch[4];                          // Stores the weights related to each one of the signals taken into consideration
    int update_method, iteration;                                   // Selection of the update function and number of training sessions done
    float sampling_time, cutoff;                                    // Sampling time and cutoff frequency. Necessary for the Low Pass Filter
    std::ofstream file;                                             // File to store data    
    auto begin = std::chrono::high_resolution_clock::now();         // Beginning of the program


    // Put the header of the data files
    file.open("evolution_roll.csv", std::ios_base::trunc);
    file << "Weight_roll 0" << "," << "Weight_roll 1" << "," << "Roll raw" << "," << "Roll filtered" << "," << "speed 0" << "," << "speed 1" << "," << "reflex" << "Time" << "," << "Reflex ON" << "," << "iteration\n" ;
    file.close();
    file.open("evolution_pitch.csv", std::ios_base::trunc);
    file << "Weight_pitch 0" << "," << "Weight_pitch 1" << "," << "pitch raw" << "," << "pitch filtered" << "," << "speed 0" << "," << "speed 1" << "," << "reflex" << "Time" << "," << "Reflex ON" << "," << "iteration\n" ;
    file.close();
    file.open("evolution_both.csv", std::ios_base::trunc);
    file << "Weight_roll 0" << "," << "Weight_roll 1" << "," << "Weight_pitch 0" << "," << "Weight_pitch 1" <<  "," << "Weight_pitch 2" << "," << "Weight_pitch 3" << "," << "Roll raw" << "," << "Roll filtered" << "," << "Pitch raw" << "," << "Pitch filtered" << "," << "speed 0" << "," << "speed 1" << "," << "reflex" << "," << "Time" << "," << "Reflex ON" << "," << "iteration\n" ;
    file.close();
    file.open("evolution_run.csv", std::ios_base::trunc);
    file << "Weight_roll 0" << "," << "Weight_roll 1" << "," << "Weight_pitch 0" << "," << "Weight_pitch 1" <<  "," << "Weight_pitch 2" << "," << "Weight_pitch 3" << "," << "Roll raw" << "," << "Roll filtered" << "," << "Pitch raw" << "," << "Pitch filtered" << "," << "speed 0" << "," << "speed 1" << "," << "reflex\n" ;
    file.close();

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
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

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///// Initialize the variables
    training = true;
    next = '?';             // Start in selection phase

    // Pitch, Roll Output
    pitch = imu_data.pitch;
    roll = imu_data.roll;

    ////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Weights:
    weight_roll[0] = 0;
    weight_roll[1] = 0;
    weight_pitch[0]= 0;
    weight_pitch[1]= 0; 
    weight_pitch[2]= 0;
    weight_pitch[3]= 0; 

    ////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Limit:
    /*
    std::cout << "Insert the limit angle value: ";
    std::cin >> limit;
    */
    ////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Introduce cutoff frequency
    std::cout << "Cutoff frequency? (rad/s) " ;
    std::cin >> cutoff;
    if ( cutoff <= 0)
    {
        std::cout << "Wrong value -> Repeat: ";
        std::cin >> cutoff;
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Introduce Sampling time
    std::cout << "sampling_time? (ms)  " ;
    std::cin >> sampling_time;

    if ( sampling_time <= 0)
    {
        std::cout << "Wrong value -> Repeat: ";
        std::cin >> sampling_time;
    }


    ////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Introduce Threshold
    std::cout << "Angle threshold? " ;
    std::cin >> threshold;

    if ( threshold <= 0)
    {
        std::cout << "Wrong value -> Repeat: ";
        std::cin >> threshold;
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Speeds and learning rate
    speed[0] = 0;
    speed[1] = 0;
    learning_rate = 0;

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
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
    
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///// Declaring the motor variables (more info on the motor_control.cpp file)
    /// Ports in order: PWM signal, input 1, input 2, gpio address of the Matrix Creator
    Motor left(TB6612_LEFT_MOTOR_PWMB, TB6612_LEFT_MOTOR_BIN1, TB6612_LEFT_MOTOR_BIN2, &gpio);
    Motor right(TB6612_RIGHT_MOTOR_PWMA, TB6612_RIGHT_MOTOR_AIN1, TB6612_RIGHT_MOTOR_AIN2, &gpio);


    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///// Test motors

    /*
    std::cout << "Test motors.\n\n";

    left.setMotorSpeedDirection(&gpio, 20 , 0);
    right.setMotorSpeedDirection(&gpio, 20 , 0);

    std::this_thread::sleep_for(2s);

    std::cin.get();

    left.setMotorSpeedDirection(&gpio, 0 , 0);
    right.setMotorSpeedDirection(&gpio, 0 , 0);

    */

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///// Beginning of the training and running

    while (training){

        switch(next){
            ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            // Train the both signals
            case '1':              
            {
                TrainBothRobot(left, right, imu_data, weight_roll, weight_pitch, learning_rate, speed, gpio, imu_sensor, limit, sampling_time, cutoff, &iteration, threshold);
                next = '?';
                break;
            }

            /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            // Run with the calculated weights
            case '2':               // Run with the calculted weights
            {
                RunRobot2(weight_roll, weight_pitch, left, right, imu_data, gpio, imu_sensor, sampling_time, cutoff, speed, limit, threshold);
                next = '?';
                break;
            }

            /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            // Show the weights
            case '3':
            {
                std::cout << "Roll weights:  " << weight_roll[0] << "  " << weight_roll[1] << std::endl;
                std::cout << "Pitch weights positive:  " << weight_pitch[0] << "  " << weight_pitch[1] << std::endl;
                std::cout << "Pitch weights negative:  " << weight_pitch[2] << "  " << weight_pitch[3] << std::endl;
                std::cout << "------------------------------------------------------------------------------------------------------------------\n";
                next = '?';
                break;
            }

            ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            // Change the sampling time
            case '4':
            {
                std::cout << "New sampling time: ";
                std::cin >> sampling_time;
                if (sampling_time <= 0)
                {
                    std::cout << "[ERROR] Wrong value -> Again, new sampling time:  ";
                    std::cin >> sampling_time;
                }

                std::cout << "\n\n";
                next = '?';
                break;
            }

            ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            // Change the curoff frequency
            case '5':
            {
                std::cout << "New cutoff frequency: ";
                std::cin >> cutoff;
                if (sampling_time <= 0)
                {
                    std::cout << "[ERROR] Wrong value -> Again, new cutoff frequency:  ";
                    std::cin >> cutoff;
                }

                std::cout << "\n\n";
                next = '?';
                break;
            }

            ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            // Restart weights
            case '6':                
            {
                weight_roll[0] = 0;
                weight_roll[1] = 0;
                weight_pitch[0] = 0;
                weight_pitch[1] = 0;
                weight_pitch[2] = 0;
                weight_pitch[3] = 0;
                next = '?';
                break;
            }

            ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            // Change learning rate
            case '7':                
            {
                std::cout << "New learning_rate: ";
                std::cin >> learning_rate;
                if (learning_rate <= 0)
                {
                    std::cout << "[ERROR] Wrong value -> Again, new cutoff frequency:  ";
                    std::cin >> learning_rate;
                }

                std::cout << "\n\n";
                next = '6';
                break;
            }

            ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            // Exit the program
            case '8':                // Exit training
            {
                training = false;
                break;
            }

            ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            // Decision section
            case '?':                // Select option
            {
                bool correct = false;
                
                while (!correct){
                    std::cout << "\nKeep training both?(1) " << std::endl;
                    std::cout << "See robot with calculated weights? (2)" << std::endl;
                    std::cout << "Print the weights? (3)" << std::endl;
                    std::cout << "Change sampling time? (4)" << std::endl;
                    std::cout << "Change cutoff frequency? (5)" << std::endl;
                    std::cout << "Restart weights? (6)" << std::endl;
                    std::cout << "Change learning rate? (7)" << std::endl;
                    std::cout << "Exit? (8): " << std::endl;
                    std::cout << "\n Answer: " ;
                    std::cin >> next;
                    std::cout << "\n\n";
                    if ((next == '1') | (next == '2') | (next == '3') | (next == '4')| (next == '5')| (next == '6')| (next == '7')| (next == '8'))
                    {
                        correct = true;
                    }
                    else
                    {
                        std::cout << "Inadequate response. Again: " << std::endl;
                    }
                }
                std::cout << "------------------------------------------------------------------------------------------------------------------\n";

                break;
            }
        }
    }
}


