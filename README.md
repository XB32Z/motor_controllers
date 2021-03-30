# Motor Controllers
This library allows to connect to low level hardware from a host that has I/O capabilities to interface with motors and control them.
It contains all the level in between: communication interface, binary and PWM channels, encoders, motors and controllers.
Each level has a clearly defined CPP interface that can easily be extended to support new implementation. 
On the communication interface level, currently, the BCM2835 (present on all Rapsberry Pis for example) and PCA9685 chips are implemented. 

The package contains:
 - The libray: containing all the levels and interfaces as well as some implementation that should allow to control DC and servo motors on a RaspberryPi
 - ROS2 nodes: NOT IMPLEMENTED
 - A Python wrapper for the library: usefull to plot the motor responses and tune the controllers. Note that this requires to build the library as SHARED which is less efficient.

## General requirements
In order to use this library, you need to have set up ROS2. The controllers implemented here work on hardware that have direct I/O to microcontrollers such as a RaspberryPi.

## Library
### Communication
The lowest level of the library is the interface to the chips that are able to produce binary and or PWM signal. There are currently two chips implemented:
 - PCA9685: uses i2c to communicate with the host (where the program runs)
 - BCM2835: is the chips present on RPis to controls the GPIOs. The i2c communication used by the chip above, actually uses this too. Note that one should not use directly the GPIOs of the BCM2835 to power any device but connect it to some other chip like a H-Bridge.

#### PCA9685

The datasheet can be found @ https://www.nxp.com/docs/en/data-sheet/PCA9685.pdf

This controller is found on ready to use RaspberryPi hat to controller servo motors. This code is tested on RaspberryPi 4 with the "Adafruit Servo/PWM Pi HAT!".

```
sudo apt install i2c-tools libi2c-dev
```

#### BCM2835
The following dependency is required:
```
sudo apt install libcap2 libcap-dev
```
If you already have installed the [BCM2835 library](https://www.airspayce.com/mikem/bcm2835/index.html) on your PC, run the build with the option BUILD_3RDPARTY to OFF. Else CMake will download and install on the colcon install space the library.

BCM2835 edge detection feature requires to changes the BIOS configuration. See https://github.com/raspberrypi/linux/issues/2550.

 - On a RaspberryPi OS `/boot/config.txt`
 - On a Ubuntu distribution (in my case Ubuntu20 on a RPi4) `/boot/firmware/usercfg.txt`

Add the line 
```
dtoverlay=gpio-no-irq
```

### Encoders

### Controllers

Not implemented


## Nodes

Not implemented

## Examples
The library comes with a serie of example programs that can you can use to build your own program.

## Python wrapper
The library is wrapped in python. To build it, install SWIG.
```
sudo apt install swig4.0
```
Because Python works with shared libraries, the project needs to be build with the BUILD_SHARED_LIBS flag. Use
```
colcon build --packages-select motor_controllers --cmake-args -DBUILD_PYTHON_WRAPPER=ON -DBUILD_SHARED_LIBS=ON
```
to build the project as SHARED libraries with the Python wrapper. *Note that all the examples and nodes are availables but might be slighlty slower.*

### Discussions
The wrapper could be implemented directly with pybind11 as the unique_ptr reaquired to write manual conversion files. However with it too, one cannot use wrap a function taking a unique pointer as input. There a bit of trickery has to be used and a better solution would be to provide a helper class for python that instanciate an Encoder directly from the pin numbers.