from socket import *
import pickle  

class ServerSocket: 
    def __init__(self) -> None: 
        self.serverSocket = socket(AF_INET, SOCK_STREAM) 
        self.hostname = '0.0.0.0' 
        self.port = 8080 
    
    def run(self) -> None: 
        self.serverSocket.bind((self.hostname, self.port))
        self.serverSocket.listen(1) 

        while True: 
            print("The server is ready to accept information...") 
            connectionSocket, address = self.serverSocket.accept() 
            running = True 

            while running: 
                print(f"Connected to {address}")

                try: 
                    data = connectionSocket.recv(1024)
                    if not data: break  
                    
                    receivedDict = pickle.loads(data)
                    # placeholder service
                    xVal = receivedDict['x'] 
                    modifiedData = str(xVal)
                    if xVal < 0: 
                        modifiedData = 'left: ' + modifiedData 
                    elif xVal > 0: 
                        modifiedData = 'right: ' + modifiedData 
                    else: 
                        modifiedData = 'center ' + modifiedData 
                    
                    connectionSocket.sendall(modifiedData.encode()) 
                except Exception as e: 
                    print(e)
                    running = False 
            
            connectionSocket.close() 

if __name__ == "__main__": 
    s = ServerSocket() 
    s.run() 
