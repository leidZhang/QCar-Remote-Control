from socket import * 

class ServerSocket: 
    def __init__(self) -> None: 
        self.serverSocket = socket(AF_INET, SOCK_STREAM) 
        self.hostname = gethostname() 
        self.port = 8080 
    
    def run(self) -> None: 
        self.serverSocket.bind((self.hostname, self.port))
        self.serverSocket.listen(1) 

        while True: 
            print("The server is ready to accept information...") 
            connectionSocket, address = self.serverSocket.accept() 

            running = True 
            while running: 
                try: 
                    message = connectionSocket.recv(1024).decode() 
                    if not message: break 
                    print(f"Got the message from {address}: " + message) 
                    
                    modifiedMessage = message.upper().encode() 
                    connectionSocket.send(modifiedMessage) 
                except Exception as e: 
                    running = False 
            
            connectionSocket.close() 

if __name__ == "__main__": 
    s = ServerSocket() 
    s.run() 