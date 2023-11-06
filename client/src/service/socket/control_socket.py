import sys 
import pickle
from socket import * 

sys.path.append('src/') 
from common.service_module import ServiceModule
from common.utils import handle_full_queue  

class ControlSocket(ServiceModule): 
    def __init__(self, ip, port) -> None: 
        self.host_name = ip 
        self.port = port  
        self.done = False 
        self.socket = socket(AF_INET, SOCK_STREAM)  # tcp protocal 

    def terminate(self) -> None:
        self.done = True 

    def is_valid(self) -> bool:
        if self.host_name is None or self.port is None: 
            return False 
        return True 
    
    def run(self, queue_lock, control_queue, response_queue) -> None:
        print("activating control socket...")
        try:  
            self.socket.connect((self.host_name, int(self.port))) 
            print("Successfully connected to the QCar")

            while not self.done: 
                queue_lock.acquire() 

                try: 
                    if not control_queue.empty(): 
                        data = control_queue.get() # get dict object
                        queue_lock.release() 
                        self.clientSocket.sendall(pickle.dumps(data)) 

                        response = self.clientSocket.recv(1024)
                        responseData = pickle.loads(response) 
                        handle_full_queue(response_queue, responseData) 
                    else: 
                        queue_lock.release()
                except ConnectionResetError:
                        # Handle the case where the server closes the connection unexpectedly
                        print("Server connection reset. Reconnecting...")
                        self.terminate()  # Close the socket
                        self.clientSocket = socket(AF_INET, SOCK_STREAM)
                        self.clientSocket.connect((self.host_name, self.port))
        except Exception as e:
            print("An error occurred: {}".format(e))
            self.terminate()