import signal 
import clientThread 

if __name__ == '__main__': 
    try: 
        c = clientThread.ClientThread()
        print("Running clientThread") 
        c.run() 
    except: 
        c.terminate()
    
    
