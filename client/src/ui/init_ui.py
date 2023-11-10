import os 
import json 
import webview 

class Api: 
    def __init__(self) -> None:
        self.data = {} 
        self.file_path = "src/ui/json/setting.json"
        self.window = None 

    def apply_setting(self, data) -> None: 
        self.data = data 
        with open(self.file_path, "w") as json_file: 
            json.dump(self.data, json_file) 

        print("Settings applied, please restart the program") 
        self.window.destroy() 
        os._exit(0) 
 
    def load_json(self) -> dict: 
        try: 
            if os.path.exists(self.file_path): 
                with open(self.file_path, "r") as json_file: 
                    self.data = json.load(json_file) 
        except: 
            print("file is corrupted")
            return; 

        return {'data': self.data}

class InitUI: 
    def __init__(self) -> None:
        self.api = Api() 

    def initialize(self) -> None: 
        window = webview.create_window(
            'QCar Setting', 
            'ui/web/index.html', 
            js_api=self.api, 
            width=650, 
            height=750,
        ) 

        self.api.window = window
        webview.start()

        

if __name__ == "__main__": 
    ui = InitUI() 
    ui.initialize() 

    print(ui.api.data)  