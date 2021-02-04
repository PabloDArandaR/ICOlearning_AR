import sys # Necessaty to insert the path of files which are in other folders
from communication_class import Server
import pickle
import threading

sys.path.append('/home/pablo/Desktop/MY_THESIS/MasterThesisFiles_AR/measurement_processing/')

import filtering

computer = Server()

def DataEvaluation(n_var, n_states):

    while computer.data.shape[1] < n_states + 1:  # + 1 because the first column  of data is all zeros
        print(computer.data.shape)

    current_shape = computer.data.shape
    previous_shape = (0,0)

    while True:
        current_shape = computer.data.shape
        if ( current_shape != previous_shape):
            last_mean = MovingMean(data = computer.data, n_var = n_var, n_states = n_states)
            print(last_mean)
            previous_shape = current_shape
        
start_thread = threading.Thread(target = DataEvaluation, args = (9, 5))

computer.create_socket()
computer.create_server()
computer.start()
