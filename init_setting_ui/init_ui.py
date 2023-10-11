import os
import json 
import tkinter as tk 
from tkinter import ttk 
from tkinter import messagebox

class InitUI: 
    def __init__(self) -> None:
        self.root = tk.Tk() 
        self.values = {
            'operation_mode': None, 
            'ip': None, 
            'port': None, 
            'spawn_point': None, 
            'controller': None, 
            'vedio': None 
        }
        self.settings = {
            'operation_mode': None, 
            'ip': None, 
            'port': None, 
            'spawn_point': None, 
            'controller': None, 
            'vedio': None 
        }

    def initialize_ui(self) -> None: 
        self.root.title("QCar remote control") 
        # set panel size 
        self.root.geometry("650x300") 
        
        # operating mode selection 
        self.values['operation_mode'] = tk.StringVar() 
        operation_mode_label = tk.Label(self.root, text="Operation Mode", font=("Arial", 14)) 
        remote_button = ttk.Radiobutton(self.root, text="Remote Control", variable=self.values['operation_mode'], value="remote") 
        local_button = ttk.Radiobutton(self.root, text="Local Virtual Environment Only", variable=self.values['operation_mode'], value="local") 
        operation_mode_label.place(x=50, y=20)
        remote_button.place(x=50, y=50) 
        local_button.place(x=180, y=50) 

        # QCar variable setting 
        ip_label = tk.Label(self.root, text="QCar IP", font=("Arial", 10))
        self.values['ip'] = tk.Entry(self.root, width=20) 
        port_label = tk.Label(self.root, text="Port", font=("Arial", 10)) 
        self.values['port'] = tk.Entry(self.root, width=10) 
        spawn_label = tk.Label(self.root, text="Spawn ", font=("Arial", 10)) 
        self.values['spawn_point'] = tk.Entry(self.root, width=5)
        ip_label.place(x=50, y=80) 
        self.values['ip'].place(x=110, y=80) 
        port_label.place(x=280, y=80) 
        self.values['port'].place(x=320, y=80)
        spawn_label.place(x=420, y=80)
        self.values['spawn_point'].place(x=480, y=80) 

        # controller selection 
        self.values['controller'] = tk.StringVar() 
        controller_label = tk.Label(self.root, text="Controller Selection", font=("Arial", 14)) 
        wheel_controller_button = ttk.Radiobutton(self.root, text="Steering Wheel Controller", variable=self.values['controller'], value="wheel") 
        logi_gamepad_button = ttk.Radiobutton(self.root, text="Logitech Gamepad Controller", variable=self.values['controller'], value="gamepad") # this is a place holder 
        keyboard_button = ttk.Radiobutton(self.root, text="Keyboard Control", variable=self.values['controller'], value="keyboard") # this is a place holder 
        controller_label.place(x=50, y=110)
        wheel_controller_button.place(x=50, y=140) 
        logi_gamepad_button.place(x=240, y=140)
        keyboard_button.place(x=450, y=140)

        # vedio stream source
        self.values['vedio'] = tk.StringVar() 
        vedio_stream_label = tk.Label(self.root, text="Vedio Stream Source", font=("Arial", 14)) 
        qlab_sensor_button = ttk.Radiobutton(self.root, text="QLab", variable=self.values['vedio'], value="qlab") 
        qcar_sensor_button = ttk.Radiobutton(self.root, text="QCar", variable=self.values['vedio'], value="qcar") 
        no_sensor_button = ttk.Radiobutton(self.root, text="Run Without Vedio Stream Source", variable=self.values['vedio'], value="none") 
        vedio_stream_label.place(x=50, y=170) 
        qcar_sensor_button.place(x=50, y=200) 
        qlab_sensor_button.place(x=120, y=200) 
        no_sensor_button.place(x=190, y=200) 

        # apply button 
        apply_button = tk.Button(self.root, text="Apply", command=self.handle_apply) 
        apply_button.place(x=300, y=240)

    def open_last_setting(self) -> None: 
        if os.path.exists("setting.json"): 
            with open("setting.json", "r") as json_file: 
                self.settings = json.load(json_file) 

        for key, val in self.settings.items(): 
            if isinstance(self.values[key], tk.Entry): 
                self.values[key].insert(0, val) 
            else: 
                self.values[key].set(val)  

    def handle_apply(self) -> None: 
        for key, val in self.values.items(): 
            if val.get() == "": 
                messagebox.showerror("Error", "Illegal input!")
                return; 
            self.settings[key] = val.get() 

        with open("setting.json", "w") as json_file: 
            json.dump(self.settings, json_file) 

        print("Setting saved to setting.json") 
        self.root.destroy() 

    def run(self) -> None: 
        self.initialize_ui() 
        self.open_last_setting() 
        self.root.mainloop() 

if __name__ == "__main__": 
    init_ui = InitUI() 
    init_ui.run() 
    # print("gui destroyed")
    # print(init_ui.settings['operation_mode'])