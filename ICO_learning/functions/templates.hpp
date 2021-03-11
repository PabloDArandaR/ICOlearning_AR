#include <iostream>
#include <fstream>
#include <cstring>
#include <chrono>

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
        array_of_values[i + 1] = array_of_values[i];

        std::cout << "-------------------------------------------------------------------------------------------------" << std::endl;
        
     }
    array_of_values[0] = add;
}
