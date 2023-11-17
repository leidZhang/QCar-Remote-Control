import os 
import sys 
import time 
import json 

sys.path.append('src/') 
from ui.init_ui import InitUI
from service.manager.service_manager import ServiceManager

if __name__ == "__main__": 
    try: 
        file_path = "src/ui/json/setting.json"
        with open(file_path, "r") as json_file: 
            data = json.load(json_file)

            print("The following is your initial setting: ")
            for key, val in data.items(): 
                print(key + ":", val) 
        
        ipt = "" 
        while (ipt != "y" or ipt != "n"): 
            ipt = input("Do you wish to use this setting? [y/n]") 
            if (ipt == "n"): 
                ui = InitUI() 
                ui.initialize() # wait here until user apply the settings 
            elif (ipt == "y"): 
                service_manager = ServiceManager(data) 
                service_manager.run() 
        
                done = False
                while not done: # waiting SIGINT signal
                    time.sleep(100)
            else: 
                print("Invalid input!") 
    except KeyboardInterrupt: 
        done = True 
        service_manager.terminate()  
    # finally: 
    #     print("System stopped") 
    #     os._exit(0) # exit the program 