import sys 
import tkinter as tk 

sys.path.append('src/service/ui/')  
from components.ui_component import UIComponent 

class QCarControlSettingSection(UIComponent): 
    def create_widgets(self) -> None:
        title_label = tk.Label(self.root, text="QCar Settings", font=("Arial", 14)) 
        ip_label = tk.Label(self.root, text="QCar IP", font=("Arial", 10))
        self.values['ip'] = tk.Entry(self.root, width=20) 
        port_label = tk.Label(self.root, text="Port", font=("Arial", 10)) 
        self.values['port'] = tk.Entry(self.root, width=10) 
        device_label = tk.Label(self.root, text="Device", font=("Arial", 10))
        self.values['device'] = tk.Entry(self.root, width=5)

        title_label.place(x=50, y=self.pos_y)
        ip_label.place(x=50, y=self.pos_y+30) 
        self.values['ip'].place(x=110, y=self.pos_y+30) 
        port_label.place(x=280, y=self.pos_y+30) 
        self.values['port'].place(x=320, y=self.pos_y+30)
        device_label.place(x=430, y=self.pos_y+30)
        self.values['device'].place(x=480, y=self.pos_y+30)