from queue import Full

def handleFullQueue(dataQueue, data): 
    try: 
        dataQueue.put_nowait(data)
    except Full: 
        dataQueue.get() 
        dataQueue.put_nowait(data)