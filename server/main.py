import time 
import signal 

from serverThread import ServerThread 

if __name__ == '__main__':   
    s = ServerThread()
    print("starting client")
    signal.signal(signal.SIGINT, s.terminate)
    s.run() 

    while True:
        time.sleep(100) 