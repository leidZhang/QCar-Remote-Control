import os 
import keyboard 

class KeyboardController: # this is a prototype class, may violate SOLID principle
    def __init__(self) -> None: 
        self.done = False
        self.state = {
            'throttle': 0, 
            'steering': 0, 
            'cruise_throttle': 0, 
            'flags': {
                'safe_flag': True, 
                'reverse_flag': False, 
                'light_flag': False, 
                'cruise_flag': False, 
            }
        }
         
        self.reverse_flag = False 

    def brake(self, keyborad_event) -> None: 
        if keyborad_event.event_type == 'down': 
            self.state['throttle'] = 0 

    def reverse(self, keyborad_event) -> None: 
        if keyborad_event.event_type == 'down':
            self.state['flags']['reverse_flag'] = not self.state['flags']['reverse_flag']
    
    def cruise(self, keyborad_event) -> None: 
        if keyborad_event.event_type == 'down':
            self.state['flags']['cruise_flag'] = not self.state['flags']['cruise_flag'] 
            self.state['cruise_throttle'] = self.state['throttle'] 

    def light_toggle(self, keyborad_event) -> None: 
        if keyborad_event.event_type == 'down':
             self.state['flags']['light_flag'] = not self.state['flags']['light_flag']
        
    def safe_toggle(self, keyborad_event) -> None: 
        if keyborad_event.event_type == 'down':
            self.state['flags']['safe_flag'] = not self.state['flags']['safe_flag'] 

    def run(self) -> None: 
        keyboard.on_press_key('s', self.brake)  
        keyboard.on_press_key('e', self.reverse) 
        keyboard.on_press_key('q', self.cruise) 
        keyboard.on_press_key('l', self.light_toggle)
        keyboard.on_press_key('/', self.safe_toggle) 

        while not self.done: 
            if keyboard.is_pressed('w'): 
                self.state['throttle'] += 5 * (1 if not self.self.state['flags']['reverse_flag'] else -1)
                if self.state['throttle'] > 3000: 
                    self.state['throttle'] = 3000   
                if self.state['throttle'] < -3000:  
                    self.state['throttle'] = -3000 
            elif not self.reverse_flag and self.state['throttle'] > 0: 
                self.state['throttle'] -= 5 
                if self.state['throttle'] < 0: 
                    self.state['throttle'] = 0  
            elif self.reverse_flag and self.state['throttle'] < 0: 
                self.state['throttle'] += 5 
                if self.state['throttle'] > 0: 
                    self.state['throttle'] = 0 

            if keyboard.is_pressed('a') and self.state['steering'] >= -300: 
                self.state['steering'] -= 5 
                if self.state['steering'] < -300: 
                    self.state['steering'] = -300 
            elif keyboard.is_pressed('d') and self.state['steering'] <= 300: 
                self.state['steering'] += 5 
                if self.state['steering'] > 300: 
                    self.state['steering'] = 300 

            if keyboard.is_pressed('`'): 
                break 
            
            os.system("cls")
            print("throttle: ", self.state['throttle']) 
            print("steering: ", self.state['steering'])  

if __name__ == "__main__": 
    k = KeyboardController() 
    k.run() 