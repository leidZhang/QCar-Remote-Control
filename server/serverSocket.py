import queue 
from socket import * 

class ServerSocket: 
    def __init__(self) -> None: 
        self.serverSocket = socket(AF_INET, SOCK_STREAM) 
        self.hostname = gethostname() 
        self.port = 8080 
    
    def run(self, dataQueue, responseQueue) -> None: 
        self.serverSocket.bind((self.hostname, self.port))
        self.serverSocket.listen(1) 

        while True: 
            print("The server is ready to accept information...") 
            connectionSocket, address = self.serverSocket.accept() 
            print(f"Connect to {address}") 

            running = True 
            while running: 
                try: 
                    message = connectionSocket.recv(1024).decode() 
                    if not message: break 
                    print(f"Got the message: " + message) 
                    dataQueue.put(message) 

                    try: 
                        modifiedMessage = responseQueue.get(timeout=1)
                        connectionSocket.send(modifiedMessage.encode()) 
                    except queue.Empty:
                        print("Response queue is empty, cannot send response.")
                except Exception as e: 
                    running = False 
            
            connectionSocket.close() 