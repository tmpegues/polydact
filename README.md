# Polydact: a flexible extra digit
[Polydact on portfolio](tmpegues.github.io/project/polydact)
## Overview
Polydact a wearable robotic sixth finger that gives its user greater dexterity than they have with their natural hand. The device is controlled by a glove with embedded flex sensors, as shown in the video below.

[![Polydact in use](/media/video.png)](https://youtu.be/Mq05AG6Wxf4)
<img src=media/index_finger.gif width="33%">
<img src=media/middle_finger.gif width="33%">
<img src=media/ring_finger.gif width="33%">


## Hardware Required
There are four main parts of the device: the tentacle itself, a control device, actuators, and electronics. *Italicized* parts are off the shelf parts that can be purchased. **Bolded** parts are custom designed and must be fabricated.

### 1. Tentacle
Four parts are required:
1. [**Tentacle**](/design/3d_files/multi_material_tentacle/)
    * Import all STL files to slicer as a single object with multiple parts
2. [**Socket**](/design/3d_files/hand_bracket/adjustable_mount_socket.stl)
3. [**Wrist mount**](/design/3d_files/hand_bracket/adjustable_mount_hand_bracket.stl)
4. A strap is also required to attach the wrist mount to the user.

### 2. Control Device
A glove with three flex sensors attached is the recommended control device. Below is an image and circuit diagram of the glove created for this project. It uses 3 *[Spectra Symbol flex sensors](https://www.adafruit.com/product/1070)*.

![Control glove and circuit diagram](/design/circuit/glove_circuit2.svg)

### 3. Actuation
Six parts are required:
1. *Motors*
    * *Robotis Dynamixel XM430-W210-T* were used for this project.
2. [**3x Motor Frames**](/design/3d_files/motor_parts/motor_frame.stl)
3. [**Motor Strap**](/design/3d_files/motor_parts/motor_strap/)
    * Import all STL files to slicer as a single object with multiple parts
4. [**3x Spools**](/design/3d_files/motor_parts/spool.stl)
    * [**Spool Tops**](/design/3d_files/motor_parts/spool_top.stl) are optional, but help retain tendons on spools
5. *A flexiblie, inextensible wire or thread to be used as a cable or tendon.* Braided fishing line is used for this project.
6. *Guide tubes* for routing the tendon from the motors to the tentacle.

### 4. Electronics
Two functions are required: the ability to interface with the control device and to interface with the motors. Two options are suggested:
1. Use a *Raspberry Pi* with external analog to digital converter (ADC). An eight-channel *MCP3008 ADC* is used for this project, but only three channels are required.
    * The Raspberry Pi will interface with the control device using the ADC and control the motors by connecting to the U2D2 using USB.
2. Use a laptop or other full featured computer connected by USB serial to a microcontroller with built-in ADC that interfaces with the control device.
    * [Example code](/pico_serial_sensor/) is provided to use a *Raspberry Pi Pico 2* to inferface with the control device.

Power: A 12 V battery can be used connected to the U2D2 Power Hub directly, and the Raspberry Pi through a buck converter to allow use of the device not tethered to an external power supply.

## Assembly
Using a battery powered Raspberry Pi for motor control with ADC for control device interface:
1. Thread tendons through the holes in the tentacle, socket, and guide tubes. Tie all three tendons in a knot at the free end of the tentacle.
    * It can be helpful to place two sections of heat-shrink tubing over on each guide tube to fix them to the socket and motor frames.
2. Place two M3 screw through holes on opposite faces of the socket and wrist mount. Adjust the angle of the socket relative to the wrist mount and tighten using nuts on the outside of the wrist mount.
3. If the tentacle is loose in the socket, hex nuts can be placed in the counterbores on the inside of the socket and screws can then be used to hold the tentacle in place.
4. Individually tie or otherwise fix the free ends of the the tendons to their own spools.
5. Screw spools onto motors and attach frames
    * If using heat shrink on guide tubes, this is a good time to shrink the tubes onto both the socket and motor frames
6. Attach switch box, Raspberry Pi, U2D2 and Power Hub, and motor strap to chest strap and tie diagonally across the user's chest.
7. Attach wrist mount to strap and tie strap to user's wrist.
8. Slide motors into motor strap.
9. Admire your handiwork


## ROS Setup
Packages `polydact` and `polydact_interfaces` are written for ROS 2 Kilted and can be downloaded and built with
```
git clone https://github.com/tmpegues/polydact.git
cd polydact
rosdep install --from-paths src --ignore-src
colcon build --paths src/*
source install/setup.bash
```
After installing dependencies, building workspace, and sourcing, you will be ready to launch.

## Launching
The main launchfile can be used with
```
ros2 launch polydact polydact.launch.xml sensor_source:=pi motors:=true control_mode:=0 plotter:=false
```
This will launch both the `pi_adc_sensor_node` and `motor_coordinator` nodes.
* `sensor_source` can be changed to `serial` to use a serial device over USB instead.
* `motors:=true` launches the `motor_coordinator` node.
    * If developing or troubleshooting a control device, `motors:=false` will not launch the `motor_coordinator_node`, allowing for focus on teh sensor node.
* `control_mode:=0` will initialize the motors with torque off.
    * After sensor calibration, motor torque can be activated with `ros2 service call /set_mode polydact_interfaces/srv/Mode '{mode: 1}'`
* `plotter:=false` will not open the PlotJuggler visualization of sensor positions.
    * When controlling the device from a Raspberry Pi, it is recommended to run PlotJuggler on another device that can receive the ROS topics so the Polydact user can see that their finger movements are being read properly.

