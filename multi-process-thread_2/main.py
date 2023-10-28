import sys 
import time 

sys.path.append('manager/')
from service_manager import ServiceManager

if __name__ == "__main__": 
    try: 
        c = ServiceManager() 
        c.run() 
        while True: 
            time.sleep(100)
    except KeyboardInterrupt: 
        c.terminate() 