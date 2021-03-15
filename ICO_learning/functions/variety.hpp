#ifndef _variety_GUARD
#define _variety_GUARD

#include <vector>
#include <iostream>
#include <thread>
#include <chrono>

using namespace std::chrono_literals;

template <typename T>
void print_vector(T data_vector ){

    unsigned int number_of_values {data_vector.size()};

    for (int i = 0; i <= number_of_values; i++){
        printf("%f",data_vector[i]);
        std::cout << " ";
        //std::cout <<  data_vector[i] << "  ";
    }

    std::cout << std::endl;
}

template <typename T>
void print_vector_pointer(T data_vector ){

    unsigned int number_of_values {data_vector->size()};

    for (int i = 0; i <= number_of_values; i++){
        printf("%f",data_vector[i]);
        std::cout << " ";

        std::this_thread::sleep_for(10s);
        //std::cout <<  data_vector[i] << "  ";
    }

    std::cout << std::endl;
}

template <typename  T>
void print(T print_value)
{
    std::cout << print_value << std::endl;
}

template <typename T>
void print_array(T array_of_values []){
    int number_of_values {sizeof(array_of_values) / (sizeof(array_of_values[0]))};

    for (int i = 0; i <= number_of_values; i++){
        std::cout <<  array_of_values[i] << "  ";
    }

    std::cout << std::endl;
}

int CalculateMiddle(int);

void DivideVector(std::vector<float>, std::vector<float> & out_1, std::vector<float> & out_2, int);

void CalculateSizing(int);

void Median(std::vector<float>);

std::vector<float> merge_vector(std::vector<float> & vector_1, std::vector<float> & vector_2);

std::vector<float> sort_vector(std::vector<float> & array_to_sort);

float bias();

#endif