import sys 
import tkinter as tk 
from tkinter import ttk

sys.path.append('src/ui/') 
from components.ui_component import UIComponent 

class VideoStreamSection(UIComponent): 
    def create_widgets(self) -> None:
        self.values['video'] = tk.StringVar() 
        video_stream_label = tk.Label(self.root, text="Video Stream Source", font=("Arial", 14)) 
        qlab_sensor_button = ttk.Radiobutton(self.root, text="QLab", variable=self.values['video'], value="qlab") 
        qcar_sensor_button = ttk.Radiobutton(self.root, text="QCar", variable=self.values['video'], value="qcar") 
        no_sensor_button = ttk.Radiobutton(self.root, text="Run Without Video Stream Source", variable=self.values['video'], value="none") 
        video_stream_label.place(x=50, y=self.pos_y) 
        qcar_sensor_button.place(x=50, y=30+self.pos_y) 
        qlab_sensor_button.place(x=120, y=30+self.pos_y) 
        no_sensor_button.place(x=190, y=30+self.pos_y) 