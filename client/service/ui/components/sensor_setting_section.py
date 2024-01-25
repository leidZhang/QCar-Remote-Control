import sys 
import tkinter as tk 

sys.path.append('src/service/ui/')  
from components.ui_component import UIComponent 

class SensorSettingSection(UIComponent): 
    def create_widgets(self) -> None:
        title_label = tk.Label(self.root, text="Sensor Settings", font=("Arial", 14)) 
        csi_camera_checkbox = tk.Checkbutton(self.root, text="CSI Camera", variable=self.values['csi_camera'])
        rgbd_camera_checkbox = tk.Checkbutton(self.root, text="RGBD Camera", variable=self.values['rgbd_camera'])
        lidar_checkbox = tk.Checkbutton(self.root, text="Lidar", variable=self.values['lidar'])

        title_label.place(x=50, y=self.pos_y) 
        csi_camera_checkbox.place(x=50, y=self.pos_y+30) 
        rgbd_camera_checkbox.place(x=150, y=self.pos_y+30)
        lidar_checkbox.place(x=250, y=self.pos_y+30)