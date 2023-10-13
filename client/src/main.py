import sys 
import time 

sys.path.append('src/') 
from service.thread.thread_manager import ThreadManager 

if __name__ == "__main__": 
    try: 
        thread_manager = ThreadManager() 
        thread_manager.run() 
        
        done = False
        while not done: # waiting SIGINT signal
            time.sleep(100)
    except KeyboardInterrupt: 
        thread_manager.terminate()  
    finally: 
        print("Client stopped")