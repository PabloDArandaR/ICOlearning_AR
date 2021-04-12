import os
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from roll_functions import SelectFile
from roll_functions import ChangeDir
from roll_functions import PlotAll8



if __name__ == '__main__':

    ######################################################################################################################
    # Change the current directiry to the one with the data
    os.chdir('/home/pablo/Desktop/MY_THESIS/Experiments/ICO_roll')
    ChangeDir()
    
    print(os.getcwd())

    ######################################################################################################################
    # Select file
    data_filename = SelectFile()
    print(data_filename)

    #######################################################################################################################
    # Open File and write columns
    data = pd.read_csv(data_filename, delimiter=',', header=None)

    base_columns = ["Weight0", "Weight1","Roll_sensor","Roll_filtered","Speed_0","Speed_1","reflex","time(ms)"]

    if (len(data.columns) == 8):
        # weight_roll[0] << weight_roll[1] << imu_data.roll << mean_roll << speed[0]+extra[0] << speed[1]+extra[1] << reflex << Time(ms)
        column_names = ["Weight0", "Weight1","Roll_sensor","Roll_filtered","Speed_0","Speed_1","reflex","time(ms)"]
    elif (len(data.columns) == 9):
        column_names = ["Weight0", "Weight1","Roll_sensor","Roll_filtered","Speed_0","Speed_2","reflex","time(ms)", "reflex_ON"]
    elif (len(data.columns) < 8):
        column_names = base_columns[:(len(data.columns)-1)]
    
    print(data)

    data.columns = column_names

    print("New data:")

    print(data)

    print("First column: ")

    print(data.loc[:,"Weight0"])

    if(len(data.columns) == 8):
        PlotAll8(data, 0, len(data))