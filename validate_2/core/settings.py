import queue
import threading 

INIT_STATE = {
    'throttle': 0, 
    'steering': 0, 
    'cruise_throttle': 0, 
    'control_flags': {
        'safe': True, 
        'reverse': False, 
        'light': False, 
        'cruise': False, 
    }
}

INIT_QUEUES = { # add more queues if needed 
    'remote': queue.Queue(10), 
    'local': queue.Queue(10), 
    'response': queue.Queue(10), 
}

INIT_LOCKS = { # add more locks if needed 
    'control': threading.Lock()
}

INIT_SETTINGS = None 
