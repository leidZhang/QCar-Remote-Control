import cv2 
import numpy as np 
from socket import * 

print("Starting server...")
s = socket(AF_INET, SOCK_DGRAM) 
address = ('127.0.0.1', 8080) 
s.bind(address) 
print("Server started")

while True: 
    data, _ = s.recvfrom(921600) 
    receivedData = np.frombuffer(data, dtype='uint8') 
    receivedImage = cv2.imdecode(receivedData, 1) 
    receivedImage = receivedImage.reshape(480, 640, 3) 
    # receivedImage = cv2.resize(receivedImage, (1600, 1200), interpolation=cv2.INTER_LINEAR) # resize to 1280 * 960 

    cv2.putText(receivedImage, "server", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2) 
    cv2.imshow('server_frame', receivedImage) 
    if cv2.waitKey(1) & 0xFF == ord('q'): 
        break 

cv2.destroyAllWindows() 