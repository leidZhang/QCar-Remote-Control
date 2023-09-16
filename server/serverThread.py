import queue 
import threading 
import serverSocket 
import qcarControl 

class ServerThread: 
    def __init__(self): 
        self.server = serverSocket.ServerSocket() 
        self.qcarController = qcarControl.QcarControl() 

        self.dataQueue = queue.Queue() 
        self.responseQueue = queue.Queue() 
        self.queueLock = threading.Lock() 
    
    def run(self): 
        self.thread1 = threading.Thread(
            target=self.qcarController.run, 
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