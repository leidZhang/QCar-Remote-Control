import queue 
import threading 
import os 

import serverSocket 
import qcarControl 

class ServerThread: 
    def __init__(self): 
        self.server = serverSocket.ServerSocket() 
        self.qcarControl = qcarControl.QcarControl() 

        self.dataQueue = queue.Queue(10) 
        self.responseQueue = queue.Queue(10) 
        self.queueLock = threading.Lock()
    
    def terminate(self): 
        self.server.terminate() 
        self.qcarControl.terminate() 
        print("Server stopped") 
        os._exit(0)
    
    def run(self): 
        self.thread1 = threading.Thread(
            target=self.qcarControl.run, 
            name="Thread-1", 
            args=(self.queueLock, self.dataQueue, self.responseQueue)
        )
        self.thread2 = threading.Thread(
            target=self.server.run, 
            name="Thread-2", 
            args=(self.dataQueue, self.responseQueue)
        ) 

        self.thread1.start() 
        self.thread2.start() 


