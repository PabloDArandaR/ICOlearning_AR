from communication_class import Client
from matrix_lite import sensors
import pickle
import time

board = Client()

board.create_socket()
board.create_connection()

while True:
    _to_send = sensors.imu.read()
    message = pickle.dumps(_to_send)
    board.send(message)
    time.sleep(1)