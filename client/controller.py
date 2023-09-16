import time 

class Controller: 
    def __init__(self) -> None:
        self.message = "aaa" 

    def run(self, queueLock, dataQueue) -> None: 
        # place holder controller 
        while True: 
            queueLock.acquire() 
            dataQueue.put(self.message) 
            queueLock.release()

            time.sleep(0.01)