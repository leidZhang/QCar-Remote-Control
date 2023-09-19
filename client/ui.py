import os 
import time

class UI: 
    def __init__(self) -> None: 
        self.data = None 
        self.stopFlag = False 
        self.display_threshold = 0.1  # Adjust this threshold as needed
        self.last_display_time = 0 

    def terminate(self) -> None: 
        self.stopFlag = True 
        print("ui stopped") 

    def run(self, responseQueue) -> None: 
        while not self.stopFlag: 
            try: 
                responseData = responseQueue.get() 
                if self.data != responseData: 
                    self.data = responseData 
                    current_time = time.time()

                    if current_time - self.last_display_time >= self.display_threshold: 
                        os.system("cls") 
                        print(self.data) 
                        self.last_display_time = current_time
            except Exception as e: 
                print(e) 
                break 
