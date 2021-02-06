import numpy as np
import statistics
import matplotlib.pyplot as plt
import time

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

data = np.load('sensor_data.npy')
data_processed = np.zeros((data.shape[0], data.shape[1] - 4))

for i in range(4,data.shape[1] ):
    for j in range(0, data.shape[0] - 1):
        data_processed[j,i-4] = np.mean(data[j,i-4:i])
    
    # The mean of the time won't be executed
    data_processed[data.shape[0],i-4] = data[data.shape[0],i]

print(data_processed)

plt.plot(data[data.shape[0] - 1, :], data[0,:])

time.time(2)


plt.plot(data_processed[data_processed.shape[0] - 1, :], data_processed[0,:])