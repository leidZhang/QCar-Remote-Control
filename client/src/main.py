import os 
import sys 
import time 
import json 
from multiprocessing import Process

sys.path.append('src/') 
from service.ui.init_ui import InitUI 
from service.manager.service_manager import ServiceManager

if __name__ == "__main__": 
    try: 
        # init_ui = InitUI() 
        # init_ui.run() # wait here until user apply the settings 
        with open("setting.json", "r") as json_file: 
            settings = json.load(json_file) 

        service_manager = ServiceManager(settings) 
        service_manager.run() 
        
        done = False
        while not done: # waiting SIGINT signal
            time.sleep(100)
    except KeyboardInterrupt: 
        done = True 
        service_manager.terminate()  
    finally: 
        print("System stopped") 
        os._exit(0) # exit the program 