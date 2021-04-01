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

    // If the vector has an even number of values, it will return the middle points between the two center values
    if((vector.size()%2) == 0)
    {
        return (vector[middle_position] + vector[middle_position+1])/2.0;
    }
    // else, it will return the value in the middle position
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

    // The middle value will be value in the position (n/2)(int division) + the remainder - 1
    // To see this, calculate use the 2 cases of odd input and even input and see that the result obtained is this one
    middle = (n_values/2) + (n_values%2) - 1;

    return middle;
}

void DivideVector(std::vector<float> original, std::vector<float> &out_1, std::vector<float> &out_2, int number_of_values)
{
    int middle_position = CalculateMiddle(number_of_values);

    //Resizing of vectors to store the two slices of the vector (will be of equal size if the vector is even)
    out_1.resize(middle_position + 1);
    out_2.resize(number_of_values - (middle_position + 1));

    for (int i = 0; i < number_of_values; i++)
    {
        if (i <= middle_position)
       {
           //the first values go directly to the  out_1 vector, no specfic calculations are required to know the position in which they are
            out_1[i] = original[i];
        }
        else
        {
            // The next values are set to the out_2 vector, adding an offset of the value of the middle position +1
            out_2[i - (middle_position+1)]=original[i];
        }
    }
}

// Implementation of the merge function of the merge sort algorithm for vectors
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
        // While both pivot variables haven't acquired the value of the size of each of the correspondant vectors
        if ((i!=vector_1.size()) && (j!=vector_2.size()))
        {
        //If the value of the vector_1 is lower it will insert it into the vector
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
        // Conditions in case one of the pivot variables didn't got to the maximum value yet
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

// Merge sort algorithm
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
        // Divide the vector in 2 sepparate vectors
        DivideVector(array_to_sort, int_1, int_2, number_of_values);

        //Store the sorted vectors
        int_1 = sort_vector(int_1);
        int_2 = sort_vector(int_2);

        // Merge both vectors
        result = merge_vector(int_1, int_2);
    }

    return result;

}