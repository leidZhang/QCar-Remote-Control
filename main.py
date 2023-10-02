import signal 
import time 

import clientThread 

if __name__ == '__main__': 
    print("Starting clientThread...")
    c = clientThread.ClientThread()
    signal.signal(signal.SIGINT, c.terminate)  
    c.run() 

    while True: # maintain mainThread 
        time.sleep(100) 
    
