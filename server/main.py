import signal 

from serverThread import ServerThread 

if __name__ == "__main__": 
    try: 
        s = ServerThread() 
        print("Starting server...") 
        # signal.signal(signal.SIGINT, s.terminate())
        s.run() 
    except: 
        s.terminate() 
    
