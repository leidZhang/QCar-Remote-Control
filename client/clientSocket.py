import pickle
import os 
from socket import * 

class clientSocket: 
    def __init__(self) -> None:
        self.clientSocket = socket(AF_INET, SOCK_STREAM) 
        self.hostName = '10.0.0.3' 
        self.port = 8081 

    def terminate(self): 
        self.clientSocket.close() 
        print("Socket stopped")

    def run(self, queueLock, dataQueue) -> None: 
        self.clientSocket.connect((self.hostName, self.port)) 

        while True: 
            queueLock.acquire() 

            if not dataQueue.empty(): 
                data = dataQueue.get() # get dict object
                queueLock.release() 
                self.clientSocket.sendall(pickle.dumps(data)) 

                response = self.clientSocket.recv(1024)
                responseData = pickle.loads(response) 

                os.system("cls") 
                print(f"Linear Speed: {responseData['linearSpeed']}\nRemaining battery Capacity: {responseData['batteryCapacity']}\nMotor Throttle: {responseData['motorThrottle']}\nSteering: {responseData['steering']}\n")
                
            else: 
                queueLock.release()
                