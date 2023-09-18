import signal
import sys 

from serverThread import ServerThread 

def quit(sig, frame):
    sys.exit(0)

if __name__ == "__main__": 
    signal.signal(signal.SIGINT, quit)

    try: 
        s = ServerThread() 
        print("Starting server...") 
        # signal.signal(signal.SIGINT, s.terminate())
        s.run() 
    except: 
        s.terminate() 
    
