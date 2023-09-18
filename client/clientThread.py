import queue 
import threading 
import clientSocket
import controller 
import sys 

class ClientThread: 
    def __init__(self): 
        self.client = clientSocket.clientSocket() 
        self.controller = controller.Controller() 

        self.dataQueue = queue.Queue(10) 
        self.queueLock = threading.Lock() 

    def terminate(self): 
        print("Attempting to stop the client")
        self.controller.terminate() 
        self.client.terminate() 
        sys.exit()
    
    def run(self): 
        self.thread1 = threading.Thread(target=self.controller.run, name="Thread-1", args=(self.queueLock, self.dataQueue)) 
        self.thread2 = threading.Thread(target=self.client.run, name="Thread-2", args=(self.queueLock, self.dataQueue)) 

        self.thread1.start() 
        self.thread2.start()


    