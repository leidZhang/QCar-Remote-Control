import time 

class Controller: 
    def __init__(self) -> None:
        self.message = ["aaa", "aaa", "bbb"] 
        self.counter = 0 
        self.stopFlag = False 

    def terminate(self) -> None: 
        self.stopFlag = True
        print("controller stopped") 

    def run(self, queueLock, dataQueue) -> None: 
        # place holder controller 
        while not self.stopFlag: 
            queueLock.acquire() 
            
            if self.counter == 3: self.counter = 0 
            data = self.message[self.counter] 
            self.counter += 1 

            dataQueue.put(data) 

            queueLock.release()

            time.sleep(0.01)