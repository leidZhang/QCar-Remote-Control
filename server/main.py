import sys 
import time 
import signal

from serverThread import ServerThread 

if __name__ == "__main__": # main 
    print("Starting server...")

    s = ServerThread() 
    signal.signal(signal.SIGINT, s.terminate)
    s.run() 

    while True: # maintain mainThread 
        time.sleep(100) 

    
