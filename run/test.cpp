#include <iostream>
#include <cstring>
#include <fstream>
#include <string>
//#include "aux.cpp"

int main(int argc, char* argv[])
{
    std::ifstream file;
    file.open("test.txt");
    std::string str;
    int i{0};
    float weight_roll[2], weight_pitch[4];

    //readWeights(argv[1], weight_roll, weight_pitch);

    std::cout << "Value of weight roll is:  " << weight_roll[0] << std::endl;
    std::cout << "Value of weight pitch front is:  " << weight_pitch[0] << std::endl;
    std::cout << "Value of weight pitch back is:  " << weight_pitch[2] << std::endl;
    while(true)
    {
        i ++;
    }
    

    return 0;
}