#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Mar 10 13:08:33 2021

@author: pablo
"""
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import os
from numpy import genfromtxt
import statistics

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

if __name__ == '__main__':
    
    
    os.chdir('/home/pablo/Desktop/MY_THESIS/Experiments')
    
    data_file_name = SelectFile()
    
    print("File selected is: " + data_file_name)
    
    data = pd.read_csv(data_file_name, sep=',', header=0)
    
    print(data.head())
    
    to_calculate_bias = [1,1,0,0,0,0]
    unbias = [0,0,0]
    bias = np.zeros((data.shape[1]))
    
    _check = input('Readjust? [y/n] ')
    if ((_check != 'y') | (_check != 'n')):
        print(_check)
        _check = input('Readjust? [y/n]')
        
    if _check == 'y':
        for col in data.columns:
            data[col] = data[col]/100
        
    i = 0
    for col in data.columns:
        bias[i] = statistics.median(data[col])
        print(bias[i])
        i += 1