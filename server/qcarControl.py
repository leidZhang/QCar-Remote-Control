class QcarControl: 
    def __init__(self) -> None: 
        self.input = "ccc" 

    def run(self, queueLock, dataQueue, responseQueue) -> None: 
        while True: 
            queueLock.acquire() 

            if not dataQueue.empty(): 
                data = dataQueue.get()
                
                # place holder service
                modifiedData = data.upper()
                responseQueue.put(modifiedData)
                
                queueLock.release()
            else: 
                queueLock.release()