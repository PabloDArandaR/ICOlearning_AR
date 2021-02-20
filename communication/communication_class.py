import socket
import threading
import time
import pickle
import numpy as np
from datetime import datetime

class Server:

    def __init__(self):
        self.HEADER = 64
        self.PORT = 5050
        self.FORMAT = 'utf-8'
        self.DISCONNECT_MESSAGE = "!DISCONNECT"
        self.SERVER = "192.168.0.10"
        self.ADDR = (self.SERVER, self.PORT)
        self.on_server = False
        self.ON = False
        self.data = np.zeros((13,0))
        self.connection_on = False

    def create_socket(self):
        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # AF_INET -> IPv4 protocol of communication
        # SOCK_STREAM -> Way of transmiting the data
    
    def create_server(self):
        self.server.bind(self.ADDR)
        self.on_server = True

    def start(self, func):
        self.ON = True
        self.server.listen()
        print(f"[LISTENING] Server is listening on {self.SERVER}")
        while True:
            conn, addr = self.server.accept()    # Will wait until a new connection has been received
            thread = threading.Thread(target=func, args = (conn, addr))
            thread.start()
            print(f"[ACTIVE CONNECTIONS] {threading.activeCount() - 1}")

    def handle_client(self, conn, addr):
        print(f"[NEW CONNECTION] {addr} connected.")

        connected = True
        while connected:
            msg_length = conn.recv(self.HEADER).decode(self.FORMAT)
            if msg_length:
                msg_length = int(msg_length)
                msg = conn.recv(msg_length).decode(self.FORMAT)
                if msg == self.DISCONNECT_MESSAGE:
                    connected = False

                print(f"[{addr}] {msg}")
                conn.send("Msg received".encode(self.FORMAT))
        
        conn.close()
    
    def handle_readings(self, conn, addr):
        #print(f"[NEW CONNECTION] {addr} connected.")

        self.connection_on = True

        value = np.zeros((13,1))
        
        while self.connection_on:
            msg_length = conn.recv(self.HEADER).decode(self.FORMAT)
            if msg_length:
                msg_length = int(msg_length)
                msg = conn.recv(msg_length)
                if msg == self.DISCONNECT_MESSAGE.encode(self.FORMAT):
                    self.connection_on = False
                else:
                    dict_reading = pickle.loads(msg)
                    i = 0
                    for key in dict_reading.keys():
                        value[i] = dict_reading[key]
                        i += 1

                    self.data = np.append(arr = self.data, values = value, axis=1)
                    

                #print(f"[{addr}] {msg}")
                conn.send("Msg received".encode(self.FORMAT))
                print(self.data)

        now = datetime.now()
        print(self.data.shape)
        np.savetxt('data_' + now.strftime("%H:%M:%S") + '.csv', np.transpose(self.data), delimiter=',', header="accel_x,accel_y,accel_z,gyro_x,giro_y,gito_z,mag_x,mag_y,mag_z,yaw,pitch,roll,time")
    
        
        conn.close()

    def handle_files(self, conn, addr):
        i = 0
        self.connection_on = True

        while self.connection_on:
            msg_length = conn.recv(self.HEADER).decode(self.FORMAT)
            if msg_length:
                msg_length = int(msg_length)
                msg = conn.recv(msg_length)
                if msg == self.DISCONNECT_MESSAGE.encode(self.FORMAT):
                    self.connection_on = False
                else:
                    decoded_file = pickle.loads(msg)
                    np.savetxt('np_' + i + '.csv', decoded_file, delimiter=',', header="accel_x,accel_y,accel_z,gyro_x,giro_y,gito_z,mag_x,mag_y,mag_z,yaw,pitch,roll,time")

                #print(f"[{addr}] {msg}")
                conn.send("Msg received".encode(self.FORMAT))
                print(self.data)
                i += 1

    def ChangePort(self, new_port):
        self.PORT = new_port
        self.ADDR = (self.SERVER, self.PORT)

        if self.ON:
            self.create_server()
            self.start()
        elif self.on_server:
            self.create_server()
    
    def ChangeHeader(self, new_header):
        self.HEADER = new_header
    
    def ChangeFormat(self, new_format):
        self.FORMAT = new_format
    
    def ChangeDisconnect(self, new_disconnect):
        self.DISCONNECT_MESSAGE = new_disconnect
    
    def ChangeServer(self, new_server):
        self.SERVER = new_server

class Client:

    def __init__(self):
        self.HEADER = 64
        self.PORT = 5050
        self.FORMAT = 'utf-8'
        self.DISCONNECT_MESSAGE = "!DISCONNECT"
        self.SERVER = "192.168.0.10"
        self.ADDR = (self.SERVER, self.PORT)
        self.on_socket = False
        self.ON = False

    def create_socket(self):
        self.client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.on_socket = True
    
    def create_connection(self):
        self.client.connect(self.ADDR)
        self.ON = True

    def send(self,msg):

        print("In send")
        msg_length = len(msg)
        send_length = str(msg_length).encode(self.FORMAT)
        # We need to check if the msg is 64 bytes lengths
        send_length += b' '*(self.HEADER - len(send_length))

        print("Before sending length")
        self.client.send(send_length)
        self.client.send(msg)
        print("After sending everything")
        print(self.client.recv(2048).decode(self.FORMAT))
    
    def ChangePort(self, new_port):
        self.PORT = new_port
        self.ADDR = (self.SERVER, self.PORT)

        if self.ON:
            self.create_socket()
            self.create_connection()
        elif self.on_socket:
            self.create_socket()
    
    def ChangeHeader(self, new_header):
        self.HEADER = new_header
    
    def ChangeFormat(self, new_format):
        self.FORMAT = new_format
    
    def ChangeDisconnect(self, new_disconnect):
        self.DISCONNECT_MESSAGE = new_disconnect
    
    def ChangeServer(self, new_server):
        self.SERVER = new_server
