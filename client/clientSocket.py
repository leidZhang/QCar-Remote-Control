import pickle
from socket import * 

from utils import handleFullQueue

class clientSocket: 
    def __init__(self) -> None:
        self.clientSocket = socket(AF_INET, SOCK_STREAM) 
        self.hostName = '10.0.0.3' 
        self.port = 8081 
        self.done = False 

    def terminate(self): 
        self.done = True 
        print("Socket stopped")

    def run(self, queueLock, dataQueue, responseQueue) -> None: 
        self.clientSocket.connect((self.hostName, self.port)) 

        while not self.done: 
            queueLock.acquire() 

            if not dataQueue.empty(): 
                data = dataQueue.get() # get dict object
                queueLock.release() 
                self.clientSocket.sendall(pickle.dumps(data)) 

                response = self.clientSocket.recv(1024)
                responseData = pickle.loads(response) 
                handleFullQueue(responseQueue, responseData) 
            else: 
                queueLock.release()
                