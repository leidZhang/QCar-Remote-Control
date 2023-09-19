import os 
from socket import * 

from utils import handleFullQueue 

class clientSocket: 
    def __init__(self) -> None:
        self.clientSocket = socket(AF_INET, SOCK_STREAM)  
        self.hostName = gethostname() # qcar ip 
        self.port = 8080 
        self.stopFlag = False 
        
    def terminate(self) -> None:  
        self.stopFlag = True 
        print("socket stopped") 

    def run(self, queueLock, dataQueue, responseQueue) -> None: 
        self.clientSocket.connect((self.hostName, self.port)) 

        while not self.stopFlag: 
            queueLock.acquire() 

            if not dataQueue.empty(): 
                data = dataQueue.get() 
                queueLock.release() 
                self.clientSocket.send(data.encode()) 
                response = self.clientSocket.recv(1024).decode() 

                handleFullQueue(responseQueue, response) 
            else: 
                queueLock.release()