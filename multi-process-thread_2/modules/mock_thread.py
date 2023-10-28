import sys 

class MockThread: 
    def __init__(self, num) -> None:
        self.num = num 
        self.message = None  
        self.done = False 

    def terminate(self) -> None: 
        self.done = True 

    def run(self) -> None: 
        self.message = 'running'

        while not self.done: 
            print("thread", self.num, self.message)  