from threading import Thread
from abc import ABC, abstractmethod 

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