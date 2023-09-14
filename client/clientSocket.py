from socket import * 

class clientSocket: 
    def __init__(self) -> None:
        self.clientSocket = socket(AF_INET, SOCK_STREAM) 
        self.hostName = gethostname() 
        self.port = 8080 

    def run(self, queueLock, dataQueue) -> None: 
        self.clientSocket.connect((self.hostName, self.port)) 

        while True: 
            queueLock.acquire() 

            if not dataQueue.empty(): 
                data = dataQueue.get() 
                queueLock.release() 
                self.clientSocket.send(data.encode()) 
                response = self.clientSocket.recv(1024).decode() 
                print(response)
            else: 
                queueLock.release()