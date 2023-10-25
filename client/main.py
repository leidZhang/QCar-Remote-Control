import os 
import sys 
import time 
sys.path.append('src/') 
from service.thread.thread_manager import ThreadManager 

if __name__ == "__main__": 
    try: 
        t = ThreadManager() 
        t.run() 
        
        done = False
        while not done: # waiting SIGINT signal
            time.sleep(100)
    except KeyboardInterrupt: 
        done = True 
        t.terminate()  
    finally: 
        print("System stopped") 
        os._exit(0) # exit the program 