C3 INS ROS node
---------------

Overview
--------
The C3 INS ROS node receives ("consumes") UDP packets from the ixBlue C3 INS unit.
The UDP packet is decoded, and if the contents are sucessfully validated, the 
information is published ("produced") for the other ROS nodes in the system.

The C3 INS UDP packets are assumed to be in the proprietary ixBlue "NAVIGATION LONG"
format. See numbered page 370 of the ixBlue "INS Interface Libary" document.
