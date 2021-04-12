import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import os
from numpy import genfromtxt
from scipy import fft, ifft

def ReplaceChar(filename, old = ' ', new = ','):
    
    with open(filename,mode = 'r+') as f:
        file = f.read().splitlines()

    #Modify the lines
    i = 0
    for line in file:
        line = line.replace(' ',',')
        file[i] = line
        i += 1
    
    #Write each new line
    with open(filename, mode = 'w') as f:
        for line in file:
            f.write(line + "\n")        

def ChangeDir():
    directories = [ name for name in os.listdir() if os.path.isdir(os.path.join(os.getcwd(), name)) ]
    i = 0
    for dir in directories:
        print(str(i+1) + ". " + dir)
        i += 1

    pos_correct = False

    while (not pos_correct):
        position = (int)(input('Enter a nubmber of the list: ')) - 1
        if (position < i) and (position >= 0):
            pos_correct = True

    os.chdir(directories[position])

def SelectFile():
    
    files = [ name for name in os.listdir() if not os.path.isdir(os.path.join(os.getcwd(), name)) ]
    i = 0
    for file in files:
        print(str(i+1) + ". " + file)
        i += 1

    pos_correct = False

    while (not pos_correct):
        position = (int)(input('Enter a number of the list: ')) - 1
        if (position < i) and (position >= 0):
            pos_correct = True

    return files[position]
   
def LowPassFilter(sample_time, cutoff, data_file, initial_value = 0):
    result = np.zeros((len(data_file),1))
    result[0] = initial_value;
    alpha = sample_time*cutoff/(1+ sample_time*cutoff)
    
    for i in range(1, len(data_file)):
        result[i] = result[i-1]*(1-alpha) + data_file[i]*alpha
        
    return result

if __name__ == '__main__':

    os.chdir('..')
    os.chdir("../Experiments/")

    ChangeDir()

    data_file_name = SelectFile()

    print("File selected is: " + data_file_name)

    #ReplaceChar(data_file_name, old = ' ', new= ',')

    data = genfromtxt(data_file_name, delimiter = ',')
    
    Plot(np.linspace(start = 0, stop = data.shape[0], num = data.shape[0]), data[:,1], data[:,2])
    
     #file << 
     #0. weight_roll[0]
     #1. weight_roll[1]
     #2. imu_data.roll
     #3. mean_roll
     #4. speed[0]+extra[0]
     #5. speed[1]+extra[1]
     #6. reflex 
     
     #######################################################################################
     #### Dividing Input data in different arrays
     
     time_ = data[2003:3003,]
     original = data[1:1001,]
     filtered = data[1002:2002]
     
     
     
     plt.figure()
     plt.plot(np.linspace(start = 0, stop = data.shape[0], num = data.shape[0]), data[:,3])
     plt.show()
     plt.savefig("roll_mean.png")