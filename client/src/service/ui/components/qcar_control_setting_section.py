import sys 
import tkinter as tk 

sys.path.append('src/ui/') 
from components.ui_component import UIComponent 

class QCarControlSettingSection(UIComponent): 
    def create_widgets(self) -> None:
        ip_label = tk.Label(self.root, text="QCar IP", font=("Arial", 10))
        self.values['ip'] = tk.Entry(self.root, width=20) 
        port_label = tk.Label(self.root, text="Port", font=("Arial", 10)) 
        self.values['port'] = tk.Entry(self.root, width=10) 
        spawn_label = tk.Label(self.root, text="Spawn ", font=("Arial", 10)) 
        self.values['spawn_point'] = tk.Entry(self.root, width=5)
        device_label = tk.Label(self.root, text="Device", font=("Arial", 10))
        self.values['device'] = tk.Entry(self.root, width=5)
        ip_label.place(x=50, y=self.pos_y) 
        self.values['ip'].place(x=110, y=self.pos_y) 
        port_label.place(x=280, y=self.pos_y) 
        self.values['port'].place(x=320, y=self.pos_y)
        spawn_label.place(x=420, y=self.pos_y)
        self.values['spawn_point'].place(x=480, y=self.pos_y) 
        device_label.place(x=550, y=self.pos_y)
        self.values['device'].place(x=600, y=self.pos_y)