import sys 
import tkinter as tk 
from tkinter import ttk

sys.path.append('src/service/ui/') 
from components.ui_component import UIComponent 

class OperationModeSection(UIComponent):  
    def change_qcar_entry_state(self) -> None: 
        value = self.values['operation_mode'].get() 
        if value == "local": 
            self.values['ip'].config(state='disabled') 
            self.values['port'].config(state='disabled') 
        else: 
            self.values['ip'].config(state='normal') 
            self.values['port'].config(state='normal')
        
        if value == "remote": 
            self.values['spawn_node'].config(state='disabled') 
            self.values['destination_node'].config(state='disabled') 
        else: 
            self.values['spawn_node'].config(state='normal') 
            self.values['destination_node'].config(state='normal') 

    def create_widgets(self) -> None:
        operation_mode_label = tk.Label(self.root, text="Operation Mode", font=("Arial", 14)) 
        mix_button = ttk.Radiobutton(self.root, 
                                        text="Mix Control", 
                                        variable=self.values['operation_mode'], 
                                        value="mix", 
                                        command=self.change_qcar_entry_state) 
        local_button = ttk.Radiobutton(self.root, 
                                       text="Local Virtual Environment Only", 
                                       variable=self.values['operation_mode'], 
                                       value="local", 
                                       command=self.change_qcar_entry_state) 
        remote_button = ttk.Radiobutton(self.root, 
                                       text="Remote Control Only", 
                                       variable=self.values['operation_mode'], 
                                       value="remote", 
                                       command=self.change_qcar_entry_state) 
        operation_mode_label.place(x=50, y=self.pos_y)
        mix_button.place(x=50, y=30+self.pos_y) 
        local_button.place(x=180, y=30+self.pos_y)
        remote_button.place(x=400, y=30+self.pos_y)