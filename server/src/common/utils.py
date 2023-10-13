from queue import Full 

def handle_full_queue(data_queue, data):
    try:
        data_queue.put_nowait(data)
    except Full:
        data_queue.get()
        data_queue.put_nowait(data)

def status_to_dict(linear_speed, battery_capacity, motor_throttle, steering):
    dict = {
        'linear_speed': linear_speed,
        'battery_capacity': battery_capacity,
        'motor_throttle': motor_throttle,
        'steering': steering
    }
    
    return dict 

