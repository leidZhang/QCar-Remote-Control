class MockSensor: 
    def __init__(self, char) -> None:
        self.done = False 
        self.char = char 

    def terminate(self) -> None: 
        self.done = True 

    def run(self) -> None: 
        while not self.done: 
            print("sensor", self.char, "running") 