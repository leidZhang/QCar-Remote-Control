# QCarSteeringControl 
## Introduction 
This Python-based project designed to enable remote control of a Quanser QCar or/and control Virtual QCar in the QLab. The QCar operates as a server, processing data sent from the client, while a windows workstation act as the client, transmitting input from the controller and displaying response from the QCar. 

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
To install and run this project, you need to have some packages, QLab and Logitech G Hub installed on your workstation. 
- numpy: `pip install numpy`
- keyboard: `pip install keyborad`
- tkinter: `pip install tkinter`
- logidrivepy: `pip install logidrivepy`
- Logitech G Hub: https://www.logitechg.com/en-ca/innovation/g-hub.html
- quanser package and QLab are provided by Quanser

Then, follow these steps: 
1. Clone this repository to your workstation and your QCar. You can use the following comand:
```
git clone https://github.com/leidZhang/QCarSteeringControl.git
```

2. On your QCar, navigate to the project folder and use the following command to run the server:
```
cd server
./run.sh
```
3. On your workstation, navigate to the project folder and use the following command to run the client:
```
cd client
cd src
python3 main.py
```
4. Configure the settings using the GUI.
5. Control your QCar.

### Note 
1. Sometimes the device connected to the workstation is identified as device 1, make sure the correct device is being listened.
2. The spawn point 19 is not available
3. Virtual LiDAR is not available for now
4. A dedicated graphics card with at least Nvidia GTX660 is recommended to enjoy the project as intended.
5. Gamepad module is not implemented yet 

## Usage 
You can choose to only control QCar in the virtual environment or control the QCar in both virtual environment and the real world. 

For Logitech steering wheel controller
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
- Safe Mode: 
  <br>Press XBOX button on the steering wheel controller to lock/unlock the QCar.
  
<br>For Keyboard controller
- Steering:
  <br>Press 'A' or 'D' to steer the QCar in the respective direction.
- Throttle:
  <br>Press 'W' to increase speed. You can stop the QCar by pressing 'S'. 
- Cruise Control:
  <br>Press 'Q' to engage and disengage cruise control, maintaining a steady speed.
- Reverse:
  <br>Press 'E' to enable and diable the reverse function.
- Toggle Lights On/Off:
  <br>Press 'L' to turn the QCar's light on and off, enhancing visibility during different conditions.
- Safe Mode:
  <br>Press '/' to lock/unlock the QCar.
## Update 
- <b>2023-09-26:</b> Refactored code to satisfy SOLID principle
- <b>2023-10-11:</b> Initial setting GUI
- <b>2023-10-13:</b> Keyboard control
- <b>2023-10-30:</b> sensors on virtual QCar 
## Future Work 
Implement video data streaming from the physical QCar to the workstation.

## License
This project is licensed under the Apache-2.0 license - see the LICENSE file for details.
