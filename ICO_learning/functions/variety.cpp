#include <vector>
#include <iostream>
#include "variety.hpp"
#include <thread>
#include <chrono>

using namespace std::chrono_literals;

float Median(std::vector<float> vector)
{
    int middle_position = CalculateMiddle(vector.size());
    int median = 0;

    if((vector.size()%2) == 0)
    {
        return (vector[middle_position] + vector[middle_position+1])/2.0;
    }
    else
    {
        return vector[middle_position];
    }
}

float Mean(std::vector<float> vector)
{
    float sum = 0;
    for (int i = 0; i<vector.size(); i++)
    {
        sum += vector[i];
    }
    return sum/vector.size();
}

int CalculateMiddle(int n_values)
{
    int middle {0};

    middle = (n_values/2) + (n_values%2) - 1;

    return middle;
}

void DivideVector(std::vector<float> original, std::vector<float> &out_1, std::vector<float> &out_2, int number_of_values)
{
    int middle_position = CalculateMiddle(number_of_values);

    //Resizing vector
    out_1.resize(middle_position + 1);
    out_2.resize(number_of_values - (middle_position + 1));

    for (int i = 0; i < number_of_values; i++)
    {

        if (i <= middle_position)
        {
            out_1[i] = original[i];
        }
        else
        {
            out_2[i - (middle_position+1)]=original[i];
        }
    }
}

std::vector<float> merge_vector(std::vector<float> & vector_1, std::vector<float> & vector_2){
    int number_of_values;
    int middle_position;
    std::vector<float> final;
    int i,j;

    i = 0;
    j = 0;
    final.resize(vector_1.size() + vector_2.size());

    // This loop merges the two sorted input vectors
    for (int k = 0; k < (vector_1.size() + vector_2.size()); k++)
    {   
        //If the value of the vector_1 is lower it will insert it into the vector
        if ((i!=vector_1.size()) && (j!=vector_2.size()))
        {
            if (vector_1[i] <= vector_2[j]){
                final[k] = vector_1[i];
                i += 1;
                
            }
            else
            {
                final[k] = vector_2[j];
                j += 1;
            }
        }
        else if (i!=vector_1.size())
        {
            final[k] = vector_1[i];
            i += 1;
        }
        else if (j!=vector_2.size())
        {
            final[k] = vector_2[j];
            j += 1;   
        }
    }

    return final;
}

std::vector<float> sort_vector(std::vector<float> & array_to_sort)
{
    int number_of_values,j,k,middle_position;
    std::vector<float> int_1;
    std::vector<float> int_2;
    std::vector<float> result;

    j = 0;
    k = 0;
    middle_position = 0;

    number_of_values = array_to_sort.size();

    // If the number of values is 1, it will directly return the value
    if (number_of_values == 1)
    {
        result = array_to_sort;
    }
    else
    {
        DivideVector(array_to_sort, int_1, int_2, number_of_values);

        int_1 = sort_vector(int_1);
        int_2 = sort_vector(int_2);

        result = merge_vector(int_1, int_2);
    }

    return result;

}