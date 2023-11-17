import sys 
from threading import Thread
from abc import ABC, abstractmethod 

sys.path.append('src/')
from service.socket.control_socket import ControlSocket 
from service.virtual_environment.virtual_control import VirtualControl 
from service.virtual_environment.virtual_spawn import VirtualSpawn 
from service.controller.impl.wheel_controller import WheelController 
from service.controller.impl.keyboard_controller import KeyboardController

class ThreadStrategy(ABC): 
    @abstractmethod 
    def __init__(self, *args) -> None: 
        pass 

    def register(self) -> Thread: 
        try: 
            if self.target.is_valid():
                thread = Thread(target=self.target.run, 
                                name=self.name, 
                                args=self.args) 
                return thread 
            else: 
                print(self.name, "will not activate")
        except Exception: 
            print("Error happened on ", self.name) 

class LightThreadStrategy(ThreadStrategy): 
    def register(self) -> Thread:
        try: 
            if self.target.is_valid():
                thread = Thread(target=self.target.run, 
                                name=self.name) 
                return thread 
            else: 
                print(self.name, "will not activate")
        except Exception: 
            print("Error happened on ", self.name)

class KeyboardControllerStrategy(ThreadStrategy): 
    def __init__(self, mode, args) -> None:
        self.name = 'Keyboard-Controller' 
        self.target = KeyboardController(mode) 
        self.args = args 
    
class WheelControllerStrategy(ThreadStrategy): 
    def __init__(self, mode, index, args) -> None:
        self.name = 'Wheel-Controller'
        self.target = WheelController(mode, index) 
        self.args = args 

class ControlSocketStrategy(ThreadStrategy): 
    def __init__(self, mode, ip, port, args) -> None: 
        self.name = 'Control-Socket' 
        self.target = ControlSocket(mode, ip, port) 
        self.args = args 

class VirtualSpawnStrategy(LightThreadStrategy): 
    def __init__(self, mode, traffic, start_node, end_node) -> None:
        self.name = "Virtual-Spawn" 
        self.target = VirtualSpawn(mode, traffic, start_node, end_node) 

class VirtualControlStrategy(ThreadStrategy): 
    def __init__(self, mode, args) -> None:
        self.name = "Virtual-Control" 
        self.target = VirtualControl(mode) 
        self.args = args 

