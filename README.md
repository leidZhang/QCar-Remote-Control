# QCarSteeringControl 
## Introduction 
This Python-based project designed to enable remote control of a Quanser QCar using a Logitech steering wheel controller. The QCar operates as a server, processing data sent from the client, while a windows workstation act as the client, transmitting input from the Logitech steering wheel controller and displaying response from the QCar. 

This project not only encompasses fundamental functionalities but has also successfully implemented advanced features such as cruise control, reverse functionality, and the ability to toggle the QCar's light on and off. 

This project was tested with the Logitech G920 Driving Force Steering Wheel controller, but should work with other Logitech devices as specified in the [Supported Devices](#supported-devices) section. 

## Supported Devices 
### Logitech 
This project was tested with the Logitech G920 Driving Force Racing Wheel controller, but according to Logitech document, it should also work with the following devices:  
- G29
- G920
- Driving Force GT
- G27
- G25
- Driving Force
- Formula Force GP
- Formula Force
### Quanser 
- Quanser QCar

## Installation 
To install and run this project, you need to have logidrivepy package and Logitech G Hub installed on your workstation. 
- logidrivepy: `pip install logidrivepy`
- Logitech G Hub: https://www.logitechg.com/en-ca/innovation/g-hub.html

Then, follow these steps: 
1. Clone this repository to your workstation and your QCar. You can use the following comand:
```
git clone https://github.com/leidZhang/QCarSteeringControl.git
```

The project directory layout is organized as follows: 
```
QCarSteeringControl
|   LICENSE.txt
|   README.md
|
+---client
|   |   clientSocket.py
|   |   clientThread.py
|   |   controller.py
|   |   constants.py
|   |   main.py
|   |   ui.py
|   |   utils.py
|
+---server
    |   Quanser 
    |   constants.py
    |   main.py
    |   qcarControl.py
    |   serverSocket.py
    |   serverThread.py
    |   utils.py 
```

2. On your QCar, navigate to the project folder and use the following command to run the server:
```
cd server
sudo python3 main.py
```
3. On your workstation, navigate to the project folder and use the following command to run the client:
```
cd client
python3 main.py
```
4. Control your QCar using Logitech steering wheel controller.

### Note 
1. Before you start the client, you have to change the value of self.hostname (QCar's ip address) in the clientSocket.py. Alternatively, uncomment `# self.hostName = input("Enter QCar IP address: ")` to manually input the ip address after the client starts.
2. Sometimes the index steering wheel controller is identified as 1, make sure the input of the correct device is listened. 

## Usage 
Once the server and client are running, ensure your Logitech steering wheel controller is properly connected to your workstation.
- Steering: 
  <br>Turn the steering wheel left or right to steer the QCar in the respective direction.
- Throttle:
  <br>Gently press the accelerator pedal on the Logitech steering wheel controller to increase speed. You can stop the QCar by release the pedal.
- Cruise Control:
  <br>Press Down button on the steering wheel controller to engage and disengage cruise control, maintaining a steady speed.
- Reverse:
  <br>Press Up button on the steering wheel controller to enable and diable the reverse function.
- Toggle Lights On/Off:
  <br>Press A button on the steering wheel controller to turn the QCar's light on and off, enhancing visibility during different conditions.

## Future Work 
Implement video data streaming from the QCar to the workstation.

## License
This project is licensed under the Apache-2.0 license - see the LICENSE file for details.
