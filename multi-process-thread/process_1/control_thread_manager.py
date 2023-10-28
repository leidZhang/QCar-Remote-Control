import sys 
import os 
import time 
from threading import Thread, Lock

sys.path.append('process_1/')
from mock_thread import MockThread 

class ControlThreadManager: 
    _instance = None

    def __new__(cls, *args, **kwargs) -> _instance:
        if cls._instance is None: 
            cls._instance = super().__new__(cls, *args, **kwargs)
        return cls._instance 

    def __init__(self) -> None:
        self.lock = Lock() 
        self.services = [
            MockThread(1), 
            MockThread(2), 
            MockThread(3),  
        ]
        self.threads = [] 

    def terminate(self) -> None: 
        for service in self.services: 
            service.terminate() 
        for thread in self.threads: 
            thread.join() 
        print('threads terminated')

    def run(self) -> None: 
        for service in self.services: 
            thread = Thread(target=service.run) 
            self.threads.append(thread) 
            
        for thread in self.threads: 
            thread.start() 
        
        print('threads activated') 

if __name__ == "__main__": 
    try: 
        c = ControlThreadManager() 
        c.run() 
        while True: 
            time.sleep(100)
    except KeyboardInterrupt: 
        c.terminate() 