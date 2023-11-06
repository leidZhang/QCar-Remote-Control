import sys 
from threading import Thread 
from abc import ABC, abstractmethod 

sys.path.append('src/') 
from service.control.qcar_control import QCarControl 
from service.socket.control_socket import ControlSocket 

# abstract class for thread strategies 
class ThreadStrategy(ABC): 
    @abstractmethod 
    def __init__(self, *args) -> None: 
        pass  
    
    def register(self) -> Thread: 
        try: 
            thread = Thread(target=self.target.run, 
                            name=self.name, 
                            args=self.args) 
            return thread
        except Exception: 
            print("Error happened on ", self.name) 

# implement thread strategies 
class QCarControlStrategy(ThreadStrategy): 
    def __init__(self, args) -> None:
        self.name = 'QCar-Control' 
        self.target = QCarControl() 
        self.args = args 

class ControlSocketStrategy(ThreadStrategy): 
    def __init__(self, args) -> None:
        self.target = ControlSocket()  
        self.name = "Control-Socket"
        self.args = args 

