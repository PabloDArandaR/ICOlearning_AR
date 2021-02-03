import socket
import threading
import time
import pickle
import numpy as np

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

    def create_socket(self):
        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    
    def create_server(self):
        self.server.bind(self.ADDR)
        self.on_server = True
        # AF_INET -> IPv4 protocol of communication
        # SOCK_STREAM -> Way of transmiting the data

    def start(self):
        self.ON = True
        self.server.listen()
        print(f"[LISTENING] Server is listening on {self.SERVER}")
        while True:
            conn, addr = self.server.accept()    # Will wait until a new connection has been received
            thread = threading.Thread(target=self.handle_readings, args = (conn, addr))
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
        print(f"[NEW CONNECTION] {addr} connected.")

        data = np.zeros((9,1))

        connected = True
        while connected:
            msg_length = conn.recv(self.HEADER).decode(self.FORMAT)
            if msg_length:
                msg_length = int(msg_length)
                msg = conn.recv(msg_length)
                if msg == self.DISCONNECT_MESSAGE.encode(self.FORMAT):
                    connected = False
                else:
                    dict_reading = pickle.loads(msg)
                    np.append(data, list(dict_reading.values()))
                    print(dict_reading)

                print(f"[{addr}] {msg}")
                conn.send("Msg received".encode(self.FORMAT))
        
        conn.close()

    
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
