from queue import Full

def handle_full_queue(data_queue, data): 
    try: 
        data_queue.put_nowait(data)
    except Full: 
        data_queue.get() 
        data_queue.put_nowait(data)