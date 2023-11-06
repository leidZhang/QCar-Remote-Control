from queue import Full

def handle_full_queue(data_queue, data): 
    try: 
        data_queue.put_nowait(data)
    except Full: 
        data_queue.get() 
        data_queue.put_nowait(data)

def copy_state(state) -> dict: 
        copy_state = {
            'throttle': state['throttle'], 
            'steering': state['steering'], 
            'cruise_throttle': state['cruise_throttle'], 
            'control_flags': {
                'safe': state['control_flags']['safe'], 
                'reverse': state['control_flags']['reverse'], 
                'light': state['control_flags']['light'], 
                'cruise': state['control_flags']['cruise'], 
            }
        }

        return copy_state 