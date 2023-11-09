import time 

from service.thread.thread_manager import ThreadManager

if __name__ == "__main__": 
    print("Starting server...")

    try: 
        theard_manager = ThreadManager() 
        theard_manager.run() 

        done = False 
        while not done: # maintain mainThread 
            time.sleep(100) 
    except KeyboardInterrupt: 
        theard_manager.terminate() 
    finally: 
        print("QCar stopped")

    
