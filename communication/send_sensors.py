from communication_class import Client
from matrix_lite import sensors
import pickle
import time

board = Client()

board.create_socket()
board.create_connection()

dict_data = dict()
dict_data

while True:
    _to_send = sensors.imu.read()
    print(type(_to_send))
    print(_to_send)
    print(_to_send.accel_x)
    
    x = input()
    
    if x == "exit":
        board.send(board.DISCONNECT_MESSAGE)
        break
    
    try:
        message = pickle.dumps(_to_send)
        board.send(message)
    except:
        board.send(board.DISCONNECT_MESSAGE)
        break