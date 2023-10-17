import sys 
import tkinter as tk 
from tkinter import ttk

sys.path.append('src/ui/') 
from components.ui_component import UIComponent 

class ControllerSelectionSection(UIComponent): 
    def change_device_entry_state(self) -> None: 
        value = self.values['controller'].get() 
        if value == "keyboard": 
            self.values['device'].config(state='disabled') 
        else: 
            self.values['device'].config(state='normal') 

    def create_widgets(self) -> None:
        self.values['controller'] = tk.StringVar() 
        controller_label = tk.Label(self.root, text="Controller Selection", font=("Arial", 14)) 
        wheel_controller_button = ttk.Radiobutton(self.root, 
                                                  text="Steering Wheel Controller", 
                                                  variable=self.values['controller'], 
                                                  value="wheel",
                                                  command=self.change_device_entry_state) 
        logi_gamepad_button = ttk.Radiobutton(self.root, 
                                              text="Logitech Gamepad Controller", 
                                              variable=self.values['controller'], 
                                              value="gamepad",
                                              command=self.change_device_entry_state) # this is a place holder 
        keyboard_button = ttk.Radiobutton(self.root, 
                                          text="Keyboard Control", 
                                          variable=self.values['controller'], 
                                          value="keyboard",
                                          command=self.change_device_entry_state) # this is a place holder 
        controller_label.place(x=50, y=self.pos_y)
        wheel_controller_button.place(x=50, y=30+self.pos_y) 
        logi_gamepad_button.place(x=240, y=30+self.pos_y)
        keyboard_button.place(x=450, y=30+self.pos_y)