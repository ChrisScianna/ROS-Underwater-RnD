## `Vectornav ROS Driver`
====================

A ROS node for VectorNav INS & GPS devices.

This package provides a sensor_msg interface for the VN100, 200, & 300 
devices. Simply configure your launch files to point to the serial port
of the deice and you can use rostopic to quickly get running.   

The MIT License (MIT)
----------------------

Copyright (c) 2018 Dereck Wonnacott <dereck@gmail.com>

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.



### QuickStart Guide

This assumes that you have a VectorNav device connected to your computer 
via a USB cable and that you have already created a [catkin workspace][2]

Build:

1. cd ~/catkin_ws/src
2. git clone https://github.com/dawonn/vectornav.git
3. cd ..
4. catkin_make


Run:

5. (Terminal 1) roscore
6. (Terminal 2) roslaunch vectornav vectornav.launch
7. (Terminal 3) rostopic list
8. (Terminal 3) rostopic echo /vectornav/IMU
9. (Terminal #) ctrl+c to quit



### Overview

#### vnpub node

This node provides a ROS interface for a vectornav device. It can be configured
via ROS parameters and publishes sensor data via ROS topics.


#### vectornav.launch

This launch file contains the default parameters for connecting a device to ROS.
You will problaby want to copy it into your own project and modify as required. 

### Published Topics


*/vectornav/IMU* ([sensor_msgs/Imu](http://docs.ros.org/en/api/sensor_msgs/html/msg/Imu.html)): IMU data.

*vectornav/Mag* ([sensor_msgs/MagneticField](https://docs.ros.org/en/api/sensor_msgs/html/msg/MagneticField.html)): Magnetic field data.

*vectornav/GPS* ([sensor_msgs/NavSatFix](https://docs.ros.org/en/api/sensor_msgs/html/msg/NavSatFix.html)): GPS data.

*vectornav/Odom* ([nav_msgs/Odometry](https://docs.ros.org/en/api/nav_msgs/html/msg/Odometry.html)): Odometry data.

*vectornav/Temp* ([sensor_msgs::Temperature](https://docs.ros.org/en/api/sensor_msgs/html/msg/Temperature.html)): Temperature data.

*vectornav/Pres* ([sensor_msgs/FluidPressure](http://docs.ros.org/en/api/sensor_msgs/html/msg/FluidPressure.html)): Fluid pressure data.

*/diagnostics* ([diagnostic_msgs/DiagnosticArray](http://docs.ros.org/en/api/diagnostic_msgs/html/msg/DiagnosticArray.html)): `vectornav` node diagnostics
 - Check if the IMU data has stalled. (Warning)
 - Check if the IMU data readings have stagnated. (Error)

### References

- [VectorNav](http://www.vectornav.com/)
- [ROS Workspace Tutorial](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment)
