C3 INS ROS node README
----------------------

Overview
--------
The C3 INS ROS node receives ("consumes") UDP packets from the ixBlue C3 INS unit.
The UDP packet is decoded, and if the contents are sucessfully validated, the 
information is published ("produced") for the other ROS nodes in the system.

The C3 INS UDP packets are assumed to be in the proprietary ixBlue "NAVIGATION LONG"
format. See numbered page 370 of the ixBlue "INS Interface Libary" document.

Simulating the INS Unit
-----------------------
To simulate the INS unit, a scapy script has been written to generate and send
UDP packets to the C3 INS ROS node. In the source code tree see:
     ./scapy/c3_ins.py.
This script needs to be run on a host that can "talk" to the C3 INS ROS node over IP.
This host needs to have python and scapy installed. The script contains
instructions on how to install scapy and launch it.

Unit Testing the C3 INS ROS Node
--------------------------------
* Open several xterm/console/putty sessions to the target.

* Cross compile the ixblue_c3_ins node and move it to the target. scp works well
for this. To "whole-sale" copy the entire tree for testing:
    cd home/twile/pod-devel
    scp -r ixblue_c3_ins root@192.168.214.200:../ros/pod/ixblue_c3_ins

* cd into ixblue_c3_ins directory and launch the node:
    roslaunch ixblue_c3_ins ixblue_c3_ins.launch

* On the remote host to simulate the INS node sending UCP traffic, launch the scapy
script:
    sudo python ./c3_ins.py

* In another terminal on the target, echo the ROS topic messages to the terminal:
    rostopic  echo ixblue_c3_ins/C3Ins

* When the ROS node receives a UDP packet and sucessfully parses it, the
ROS echo command will display the published data, example below:

header: 
  seq: 188
  stamp: 
    secs: 1306311723
    nsecs: 965270978
  frame_id: ixblue_c3_ins
nl_header: 9386
user_status: 0
algo_status_0: 0
algo_status_1: 0
heading: 3.1400001049
roll: 0.0
pitch: 0.0
north_speed: 2.03125
east_speed: 0.0
vertical_speed: 0.0
latitude: 849964988
longitude: 856562008
attitude: -100.0
timestamp: 0
heading_err_sd: 0.0
roll_err_sd: 0.0
pitch_err_sd: 0.0
north_speed_err_sd: 0.0
east_speed_err_sd: 0.0
vertical_speed_err_sd: 0.0
latitude_err_sd: 0.0
longitude_err_sd: 0.0
altitude_err_sd: 0.0
