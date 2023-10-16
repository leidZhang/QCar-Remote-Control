import os 
import sys 
import queue
import threading 

sys.path.append('src/') 
from service.service_module import ServiceModule 
from service.ui.init_ui import InitUI 
from strategies.thread_strategies import ControlSocketStrategy 
from strategies.thread_strategies import WheelControllerStrategy 
from strategies.thread_strategies import VirtualControlStrategy 
from strategies.thread_strategies import KeyboardControllerStrategy 

class ThreadManager(ServiceModule): 
    def __init__(self) -> None:  
        self.init_strategies = None 
        self.threads = [] 
        self.queues = { # add more queues if needed 
            'remote': queue.Queue(10), 
            'local': queue.Queue(10), 
            'response': queue.Queue(10), 
        }
        self.locks = { # add more locks if needed 
            'control': threading.Lock()
        }

    def terminate(self) -> None:
        for thread in self.init_strategies: 
            thread.terminate() 
            thread.join() 
        os._exit(0) # exit the program 

    def is_valid(self) -> bool:
        pass 
    
    def run(self) -> None:
        init_ui = InitUI() 
        init_ui.run() # wait here until user apply the settings 
        print("settings applied!")

        settings = init_ui.settings 
        print(settings['port']) 
        self.init_strategies = [ # add more strategies here if needed 
            ControlSocketStrategy(
                ip=settings['ip'], 
                port=settings['port'], 
                args=(self.locks['control'], self.queues['remote'], self.queues['response'])
            ), 
            WheelControllerStrategy(
                mode=settings['controller'],
                index=settings['device'], 
                args=(self.locks['control'], self.queues['remote'], self.queues['local'])
            ), 
            VirtualControlStrategy(
                start_node=settings['spawn_point'],  
                args=(self.queues['local'], )
            ), 
            KeyboardControllerStrategy(
                mode=settings['controller'],
                args=(self.locks['control'], self.queues['remote'], self.queues['local'])
            )
        ]

        for strategie in self.init_strategies: 
            thread = strategie.register()
            if thread != None: 
                self.threads.append(thread)        
        for thread in self.threads:  
            thread.start() 
        
        print("threads have activated")
         
# if __name__ == "__main__": 
#     t = ThreadManager() 
#     t.run() 