import numpy as np
import statistics

Q = np.matrix([0])
R = np.matrix([0])

P_min = np.plus([0])
P_plus = np.matrix([0])

F = np.matrix([0])
G = np.matrix([0])

def Kalman():
    pass

def MovingMean(data, n_var):
    result = np.zeros((n_var,1))
    for i in range(0,n_var):
        result[i] = statistics.mean(data[i,:])    

    return result

    