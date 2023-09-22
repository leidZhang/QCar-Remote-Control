import sys
import queue 
import threading 

import clientSocket
import controller 
import ui

class ClientThread: 
    def __init__(self): 
        self.client = clientSocket.clientSocket() 
        self.controller = controller.Controller() 
        self.ui = ui.UI() 

        self.dataQueue = queue.Queue(10) 
        self.responseQueue = queue.Queue(10) 
        self.queueLock = threading.Lock() 

    def terminate(self, signal, frame): 
        print("Attempting to stop the client")
        
        self.controller.terminate() 
        self.client.terminate() 
        self.ui.terminate() 

        sys.exit()
    
    def run(self): 
        self.thread1 = threading.Thread(target=self.controller.run, name="Thread-1", args=(self.queueLock, self.dataQueue)) 
        self.thread2 = threading.Thread(target=self.client.run, name="Thread-2", args=(self.queueLock, self.dataQueue, self.responseQueue)) 
        self.thread3 = threading.Thread(target=self.ui.run, name="Thread-3", args=(self.responseQueue, ))

        self.thread1.start() # activate controller 
        self.thread2.start() # activate client socket 
        self.thread3.start() # activate ui 


    