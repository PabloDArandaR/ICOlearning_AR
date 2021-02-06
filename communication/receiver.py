import sys # Necessaty to insert the path of files which are in other folders
from communication_class import Server
import pickle
import threading
import time

sys.path.append('/home/pablo/Desktop/MY_THESIS/MasterThesisFiles_AR/measurement_processing/')

import filtering

computer = Server() # Needs to be initialised before-hand so it can be used in the 
computer.ChangePort(5050)

def DataEvaluation(n_var, n_states):
    print("Inside Data Evaluation")
    while True:
        print(computer.data)
        time.sleep(1)

#data_thread = threading.Thread(target = DataEvaluation, args = (9, 5))
#data_thread.start()

computer.create_socket()
computer.create_server()
computer.start(computer.handle_readings)

'''
    current_shape = computer.data.shape
    previous_shape = (0,0)
    
    while computer.data.shape[1] < n_states + 1:  # + 1 because the first column  of data is all zeros
        if current_shape != previous_shape:
            print("Before evaluating")
            print(computer.data.shape)
    
    while True:
        current_shape = computer.data.shape
        if ( current_shape != previous_shape):
            last_mean = MovingMean(data = computer.data, n_var = n_var, n_states = n_states)
            print(last_mean)
            previous_shape = current_shape
    '''