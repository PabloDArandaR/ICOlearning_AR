#ifndef _template_GUARD
#define _template_GUARD

#include <iostream>
#include <fstream>
#include <cstring>
#include <chrono>

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///// Function declarations

template <typename T>
T mean( T array_of_values[]){
    T mean {0};
    int number_of_values {sizeof(array_of_values)/sizeof(array_of_values[0])};  //Number of values in the input array

    for (int i = 0; i < number_of_values; i++)
    {
        // Add each term of the array
        mean += array_of_values[i];
    }

    // Return the previous calculation divided by the number of values
    return mean/(float)number_of_values;
}

template <typename T>
void roll_and_add(T add, T array_of_values[]){
    int number_of_values {sizeof(array_of_values)/sizeof(array_of_values[0])};

    for (int i = (number_of_values - 1); i >= 0 ; i --){
        // Puts each value of the vector in the next position
        array_of_values[i + 1] = array_of_values[i];
    }
    
    // Puts at the beginning the new_value
    array_of_values[0] = add;
}


#endif