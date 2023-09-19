class QcarControl: 
    def __init__(self) -> None: 
        self.stopFlag = False 
        self.reverseFlag = False 
        self.cruiseFlag = False 
        self.buttonPress = [False] * 10 
        self.buttonTrigger = [False] * 10 

    def termiante(self) -> None: 
        self.stopFlag = True 

    def run(self, queueLock, dataQueue, responseQueue) -> None: 
        while not self.stopFlag: 
            queueLock.acquire() 

            if not dataQueue.empty(): 
                data = dataQueue.get()
                
                # place holder service
                modifiedData = data.upper()
                responseQueue.put(modifiedData)
                
                queueLock.release()
            else: 
                queueLock.release() 

    def handleButton(self) -> None:
        if self.pressedStatus[0]: 
            self.reverseFlag = not self.reverseFlag 
            val = 10 * (1 if self.reverseFlag else -1)
        
        if self.pressedStatus[1]: 
            self.curseFlag = not self.curseFlag 