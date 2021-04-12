import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import os
from numpy import genfromtxt
from scipy import fft, ifft

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

def DetectEndIteration(iteration_vector):

    position = list()

    for i in range(1, len(iteration_vector)):
        if (iteration_vector[i] != iteration_vector[i-1]):
            position.append(iteration_vector[i-1])
        
    return position

def PlotAll8(data, start,end):
        
    #######################################################################################################################
    # Linspace vector of data

    time_vector = np.linspace(start = 0, stop = end-start, num = end-start)

    #######################################################################################################################
    # Create plots

    plot1 , ax_1= plt.subplots(1,1)

    plot2, (ax_21, ax_22) = plt.subplots(2,1)

    plot3, (ax_31, ax_32, ax_33) = plt.subplots(3,1)

    ################################
    # Plot weight evolution

    ax_1.set_ylabel = "Weight"
    ax_1.set_xlabel = "Time"
    ax_1.set_title = "Weight evolution"

    ax_1.plot(time_vector, data.loc[start:end,"Weight0"], label="Weight 0")
    ax_1.plot(time_vector, data.loc[start:end,"Weight1"], label="Weight 1")

    ax_1.grid(b = True)
    plot1.legend()
    plot1.show()

    input()
    ax_1.clear()

    ################################
    # Plot speed_0 

    color = 'tab:red'
    ax_21.set_ylabel = "Speed"
    ax_21.set_xlabel = "Time"
    ax_21.set_title = "Speed evolution"
    ax_21.plot(time_vector, data.loc[start:end,"Speed_0"], label="Speed 0", color = color)

    color = 'tab:blue'
    ax_22.set_ylabel = "Roll angle"
    ax_22.set_xlabel = "Time"
    ax_22.set_title = "Speed evolution"
    ax_22.plot(time_vector, data.loc[start:end,"Roll_filtered"], label="Roll angle", color = color)

    ax_21.grid(b = True)
    plot2.legend()
    plot2.tight_layout()
    plot2.show()

    input()
    ax_21.clear()
    ax_22.clear()
    
    ################################
    # Plot speed_1

    color = 'tab:red'
    ax_21.set_ylabel = "Speed"
    ax_21.set_xlabel = "Time"
    ax_21.set_title = "Speed evolution"
    ax_21.plot(time_vector, data.loc[start:end,"Speed_1"], label="Speed 1", color = color)

    color = 'tab:blue'
    ax_22.set_ylabel = "Roll angle"
    ax_22.set_xlabel = "Time"
    ax_22.set_title = "Speed evolution"
    ax_22.plot(time_vector, data.loc[start:end,"Roll_filtered"], label="Roll angle", color = color)

    ax_21.grid(b = True)
    ax_22.grid(b = True)
    plot2.legend()
    plot2.tight_layout()
    plot2.show()

    input()
    ax_21.clear()
    ax_22.clear()

    ################################
    # Plot both speeds and Roll

    color = 'tab:red'
    ax_31.set_ylabel = "Speed"
    ax_31.set_xlabel = "Time"
    ax_31.set_title = "Speed evolution"
    ax_31.plot(time_vector, data.loc[start:end,"Speed_0"], label="Speed 0", color = color)

    color = 'tab:red'
    ax_32.set_ylabel = "Speed"
    ax_32.set_xlabel = "Time"
    ax_32.set_title = "Speed evolution"
    ax_32.plot(time_vector, data.loc[start:end,"Speed_1"], label="Speed 1", color = color)

    color = 'tab:blue'
    ax_33.set_ylabel = "Roll angle"
    ax_33.set_xlabel = "Time"
    ax_33.set_title = "Speed evolution"
    ax_33.plot(time_vector, data.loc[start:end,"Roll_filtered"], label="Roll angle", color = color)

    ax_31.grid(b = True)
    ax_32.grid(b = True)
    ax_33.grid(b = True)
    plot3.legend()
    plot3.tight_layout()
    plot3.show()

    input()
    ax_31.clear()
    ax_32.clear()

