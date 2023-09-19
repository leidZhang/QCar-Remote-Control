import time 
import signal 

from clientThread import ClientThread 

if __name__ == '__main__':   
    c = ClientThread()
    print("starting client")
    signal.signal(signal.SIGINT, c.terminate)
    c.run() 

    while True:
        time.sleep(100) 