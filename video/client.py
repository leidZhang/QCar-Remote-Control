import cv2  
import threading 
from socket import *  

def sendImage(): 
    s.sendto(sendData, address) 
    print(f"{image.size} Byte data has been sent") 
    s.close() 

address = ('127.0.0.1', 8080) 
capture = cv2.VideoCapture(0) 
while True: 
    _, image = capture.read() 
    image = cv2.flip(image, 1) # not needed in the QCar's code  

    s = socket(AF_INET, SOCK_DGRAM) # UDP protocol
    thread1 = threading.Thread(target=sendImage) 
    thread1.setDaemon(True) 

    _, sendData = cv2.imencode('.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, 75]) # compress frame quality 
    thread1.start() 
    cv2.putText(image, "client", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2) 
    cv2.imshow('client_frame', image) 
    if cv2.waitKey(1) & 0xFF == ord('q'): # quit if q is pressed 
        break 

cv2.destroyAllWindows() 