import threading 
from thread1 import Thread1 
from thread2 import Thread2 

class ThreadManager: 
    def __init__(self) -> None:
        self.thread_candidates = {
            'thread-1': Thread1(), 
            'thread-2': Thread2(), 
        }

        self.thread = [] 

    def run(self) -> None: 
        pass 