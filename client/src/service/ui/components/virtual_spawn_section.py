import sys 
import tkinter as tk 
from tkinter import ttk
from PIL import ImageTk, Image

sys.path.append('src/service/ui/') 
from components.ui_component import UIComponent 

class VirtualSpawnSection(UIComponent): 
    def __init__(self, image, root, values, pos_y) -> None:
        super().__init__(root, values, pos_y)
        self.image = image
        self.image_label = tk.Label(self.root, image=self.image) 

    def change_state(self) -> None: 
        value = self.values['traffic'].get() 
        if value == "left": 
            self.image = ImageTk.PhotoImage(Image.open('images/Nodes_Right.png').resize((670, 335)))
        else: 
            self.image = ImageTk.PhotoImage(Image.open('images/Nodes_Left.png').resize((670, 335)))
        
        self.image_label.config(image=self.image)

    def create_widgets(self) -> None:
        title_label = tk.Label(self.root, text="Virtual Spawn Setting", font=("Arial", 14)) 
        left_button = ttk.Radiobutton(self.root, 
                                      text="Left Hand Traffic", 
                                      variable=self.values['traffic'], 
                                      value="left", 
                                      command=self.change_state) 
        right_button = ttk.Radiobutton(self.root, 
                                      text="Right Hand Traffic", 
                                      variable=self.values['traffic'], 
                                      value="right", 
                                      command=self.change_state) 
        spawn_label = tk.Label(self.root, text="Spawn Node", font=("Arial", 10)) 
        self.values['spawn_node'] = tk.Entry(self.root, width=5)
        destination_label = tk.Label(self.root, text="Destination Node", font=("Arial", 10)) 
        self.values['destination_node'] = tk.Entry(self.root, width=5)
        
        title_label.place(x=50, y=self.pos_y)
        left_button.place(x=50, y=30+self.pos_y) 
        right_button.place(x=180, y=30+self.pos_y)
        spawn_label.place(x=330, y=self.pos_y+30)
        self.values['spawn_node'].place(x=420, y=self.pos_y+30) 
        destination_label.place(x=480, y=self.pos_y+30)
        self.values['destination_node'].place(x=600, y=self.pos_y+30)
        self.image_label.place(x=50, y=60+self.pos_y)
