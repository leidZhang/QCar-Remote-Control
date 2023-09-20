import sys 
import queue 
import threading 

import serverSocket 
import qcarControl 

class ServerThread: 
    def __init__(self): 
        self.server = serverSocket.ServerSocket() 
        self.qcarControl = qcarControl.QcarControl() 

        self.dataQueue = queue.Queue(10) 
        self.responseQueue = queue.Queue(10) 
        self.queueLock = threading.Lock()
    
    def terminate(self, signal, frame): 
        print("Stopping server...") 

        self.server.terminate() 
        self.qcarControl.terminate() 
        
        sys.exit()  
    
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

        self.thread1.start() # activate qcarControl
        self.thread2.start() # activate socket 


