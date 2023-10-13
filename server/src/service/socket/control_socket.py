import sys
import pickle
from socket import *

sys.path.append('src/')
from service.service_module import ServiceModule
from common.utils import handle_full_queue


class ControlSocket(ServiceModule): 
    def __init__(self) -> None: 
        self.server_socket = socket(AF_INET, SOCK_STREAM) 

        self.host_name = '0.0.0.0' 
        self.port = 8080
        self.done = False 

    def terminate(self): 
        print("Stopping socket...")
        self.done = True 
        self.server_socket.close() 
    
    def run(self, data_queue, response_queue) -> None: 
        self.server_socket.bind((self.host_name, self.port))
        self.server_socket.listen(1) 

        while not self.done: 
            print("The server is ready to accept information...") 
            connection_socket, address = self.server_socket.accept()
            print(f"Connected to {address}") 

            running = True 
            while running: 
                try: 
                    data = connection_socket.recv(1024)                  
                    received_data = pickle.loads(data)
                    
                    handle_full_queue(data_queue, received_data) 
                    response_data = response_queue.get()
                    connection_socket.sendall(pickle.dumps(response_data)) 

                except Exception as e: 
                    print(e)
                    running = False 
            
            connection_socket.close()  
