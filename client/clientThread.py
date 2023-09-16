import queue 
import threading 
import clientSocket
import controller

class ClientThread: 
    def __init__(self): 
        self.client = clientSocket.clientSocket() 
        self.controller = controller.Controller() 

        self.dataQueue = queue.Queue() 
        self.queueLock = threading.Lock() 
    
    def run(self): 
        self.thread1 = threading.Thread( 
            target=self.controller.run, 
            name="Thread-1", 
            args=(self.queueLock, self.dataQueue)
        ) 
        self.thread2 = threading.Thread(
            target=self.client.run, 
            name="Thread-2", 
            args=(self.queueLock, self.dataQueue)
        ) 

        self.thread1.start() 
        self.thread2.start()


    