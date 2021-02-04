import numpy as np
import statistics

Q = np.matrix([0])
R = np.matrix([0])

P_min = np.matrix([0])
P_plus = np.matrix([0])

F = np.matrix([0])
G = np.matrix([0])

def Kalman():
    pass

def MovingMean(data, n_var, n_states):
    result = np.zeros((n_var,1))
    data_slice = data[:,(-(n_states + 1)):]
    for i in range(0,n_var):
        result[i] = statistics.mean(data_slice[i,:])    

    return result

    