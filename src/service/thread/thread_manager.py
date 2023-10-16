import os 
import sys 
import queue 
import threading 

sys.path.append('src/')
from service.service_module import ServiceModule
from strategies.thread_strategies import QCarControlStrategy
from strategies.thread_strategies import ControlSocketStrategy 


class ThreadManager(ServiceModule): 
    def __init__(self) -> None: 
        self.locks = { # add new locks here if needed 
            'control_lock': threading.Lock(), # Lock for control input 
        } 

        self.queues = { # add new queues here if needed 
            'control_data_queue': queue.Queue(10), # queue for received data for control 
            'control_response_queue': queue.Queue(10), # queue for response data for control 
        }

        self.init_strategies = [
            ControlSocketStrategy(), 
            QCarControlStrategy(),  
        ]

        self.threads = [] 
    
    def terminate(self) -> None: 
        print("Stopping server...") 
        # terminate modules and thread join 
        for thread in self.threads: 
            thread.terminate() 
            thread.join() 
        # exit the program  
        os._exit(0)  
    
    def run(self) -> None: 
        print("Activating threads...") 
        
        # register threads
        for strategie in self.init_strategies: 
            thread = strategie.register()
            if thread != None: 
                self.threads.append(thread)  
        # activate threads       
        for thread in self.threads:  
            thread.start()

        print("threads have activated")

