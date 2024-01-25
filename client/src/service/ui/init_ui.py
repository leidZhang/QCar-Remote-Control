import os
import sys 
import json 
import tkinter as tk 
from tkinter import ttk
from PIL import ImageTk, Image 
from tkinter import messagebox

sys.path.append('src/service/') 
from ui.components.controller_selection_section import ControllerSelectionSection 
from ui.components.operation_mode_section import OperationModeSection 
from ui.components.qcar_control_setting_section import QCarControlSettingSection 
from ui.components.video_stream_section import VideoStreamSection 
from ui.components.virtual_spawn_section import VirtualSpawnSection
from ui.components.sensor_setting_section import SensorSettingSection 

class InitUI: 
    def __init__(self) -> None:
        self.root = tk.Tk() 
        self.settings = {}
        self.image = ImageTk.PhotoImage(Image.open('images/Nodes_Right.png').resize((670, 335)))
        
        self.values = { # add more input here if needed 
            'operation_mode': tk.StringVar(), 
            'ip': tk.Entry(self.root, width=20), 
            'port': tk.Entry(self.root, width=10), 
            'traffic': tk.StringVar(), 
            'controller': tk.StringVar(), 
            'video': tk.StringVar(),
            'device': tk.Entry(self.root, width=5), 
            'csi_camera': tk.IntVar(), 
            'rgbd_camera': tk.IntVar(), 
            'lidar': tk.IntVar(), 
        }

        self.sections = { # add more sections here if needed 
            'operation_mode_section': OperationModeSection(self.root, self.values, 20), 
            'virtual_spawn_section': VirtualSpawnSection(self.image, self.root, self.values, 70),  
            'controller_selection_section': ControllerSelectionSection(self.root, self.values, 480), 
            'qcar_setting_section': QCarControlSettingSection(self.root, self.values, 530),
            'video_stream_section': VideoStreamSection(self.root, self.values, 580), 
            'sensor_settting_section': SensorSettingSection(self.root, self.values, 630), 
        }

    def initialize_ui(self) -> None: 
        # set panel title 
        self.root.title("QCar Control") 
        # set panel size 
        self.root.geometry("780x750") 

        # create sections 
        for section in self.sections.values(): 
            section.create_widgets() 

        # apply button 
        apply_button = tk.Button(self.root, text="Apply", command=self.handle_apply) 
        apply_button.place(x=340, y=700)

    def on_closing(self) -> None: 
        print("Window is being closed!") 
        self.root.destroy() 
        os._exit(0)  

    def apply_initial_states(self) -> None: # add more if needed 
        self.sections['operation_mode_section'].change_qcar_entry_state()
        self.sections['controller_selection_section'].change_device_entry_state() 
        self.sections['virtual_spawn_section'].change_state() 
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing) 

    def open_last_setting(self) -> None: 
        try: 
            if os.path.exists("setting.json"): 
                with open("setting.json", "r") as json_file: 
                    self.settings = json.load(json_file) 

                for key, val in self.settings.items(): 
                    if isinstance(self.values[key], tk.Entry): 
                        self.values[key].insert(0, val) 
                    else: 
                        self.values[key].set(val)  
        except Exception: 
            print("file is corrupted")
            return; 

    def handle_apply(self) -> None: 
        for key, val in self.values.items(): 
            self.settings[key] = val.get() 
        # save user settings 
        with open("setting.json", "w") as json_file: 
            json.dump(self.settings, json_file) 
        # filter disabled data 
        for key, val in self.settings.items(): 
            if isinstance(self.values[key], tk.Entry) and self.values[key].cget('state') == 'disabled': 
                self.settings[key] = None 

        print("Setting saved to setting.json") 
        self.root.destroy() 

    def run(self) -> None: 
        self.initialize_ui() 
        self.open_last_setting() 
        self.apply_initial_states() 
        self.root.mainloop() 

if __name__ == "__main__": 
    init_ui = InitUI() 
    init_ui.run() 
    print("gui destroyed")