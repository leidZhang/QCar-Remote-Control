from queue import Full 

def handleFullQueue(dataQueue, data): 
    try: 
        dataQueue.put_nowait(data) 
    except Full:
        dataQueue.get() 
        dataQueue.put_nowait(data) 

def statusToDict(linearSpeed, batteryCapacity, motorThrottle, steering): 
    dict = {
        'linearSpeed': linearSpeed,  
        'batteryCapacity': batteryCapacity, 
        'motorThrottle': motorThrottle, 
        'steering': steering
    }
    
    return dict 

