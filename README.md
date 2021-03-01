# Motor Controllers
This library contains three parts that are composed to create nodes to connect to different types of controllers and accept various interfaces.

There are three parts:
 - Communication: communicate between the nodes and the controller, this is the lowest level, directly in contact with the hardware.
 - Controller: one level above, the controllers offer the possibility to implement constrol strategies onto the signal to the motors.
 - Nodes: offer ROS2 interfaces to the controllers

## General requirements
In order to use this library, you need to have set up ROS2. The controllers implemented here work on hardware that have direct I/O to microcontrollers such as a RaspberryPi.

## Communications
The library has a set of communication classes which can connect to motor controllers. Each interface implements the CommunicationInterface class. 

List of supported controllers:
 - PCA9685

### PCA9685

The datasheet can be found @ https://www.nxp.com/docs/en/data-sheet/PCA9685.pdf

This controller is found on ready to use RaspberryPi hat to controller servo motors. This code is tested on RaspberryPi 4 with the "Adafruit Servo/PWM Pi HAT!".

### BCM2835
```
sudo apt install libcap2 libcap-dev
```

#### Requirements 
The following dependencies need to be installed on the host:
```
sudo apt install i2c-tools libi2c-dev
```

## Controllers

Not implemented


## Nodes

Not implemented