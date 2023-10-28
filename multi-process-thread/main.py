import time 
from manager.process_manager import ProcessManager 

if __name__ == "__main__": 
    try: 
        p = ProcessManager() 
        p.run() 
        while True: 
            time.sleep(100)
    except KeyboardInterrupt: 
        p.termiante() 
    finally: 
        print('system terminated')