from socket import *
import queue 
import pickle  

class ServerSocket: 
    def __init__(self) -> None: 
        self.serverSocket = socket(AF_INET, SOCK_STREAM) 
        self.hostname = '0.0.0.0' 
        self.port = 8081 

    def terminate(self): 
        self.serverSocket.close() 
        print("Socket Stopped")
    
    def run(self, dataQueue, responseQueue) -> None: 
        self.serverSocket.bind((self.hostname, self.port))
        self.serverSocket.listen(1) 

        while True: 
            print("The server is ready to accept information...") 
            connectionSocket, address = self.serverSocket.accept()
            print(f"Connected to {address}") 
            running = True 

            while running: 
                try: 
                    data = connectionSocket.recv(1024)                  
                    receivedData = pickle.loads(data)
                    
                    dataQueue.put(receivedData) 
                    modifiedData = responseQueue.get()
                    connectionSocket.sendall(modifiedData.encode()) 

                except Exception as e: 
                    print(e)
                    running = False 
            
            connectionSocket.close() 

if __name__ == "__main__": 
    s = ServerSocket() 
    s.run() 
