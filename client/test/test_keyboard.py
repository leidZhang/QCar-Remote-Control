import os 
import keyboard 

def normalize_steering(y_axis_signal) -> float:
        return y_axis_signal / 100 
    
def normalize_throttle(x_axis_signal) -> float:
    return x_axis_signal / 1000 

def run(): 
    steering, throttle = 0, 0 
    while True: 
        if keyboard.is_pressed('w'): 
            throttle += 5 
            if throttle > 1000: 
                throttle = 1000  
        else: 
            throttle -= 5 
            if throttle <= 0: 
                throttle = 0 

        if keyboard.is_pressed('s'): 
            throttle = 0  

        if keyboard.is_pressed('a'): 
            steering += 5 
            if steering > 100: 
                steering = 100 
        elif keyboard.is_pressed('d'): 
            steering -= 5 
            if steering < -100: 
                steering = -100 

        if keyboard.is_pressed('`'): 
            break 

        # steering = normalize_steering(steering) 
        # throttle = normalize_throttle(throttle) 
        print("throttle:", normalize_throttle(throttle))
        print("steering:", normalize_steering(steering))  

        os.system('cls')

if __name__ == '__main__': 
    run() 