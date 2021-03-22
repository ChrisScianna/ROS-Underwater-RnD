#!/usr/bin/env python
#license removed for brevity
import rospy
import std_msgs.msg
from valeport_svs.msg import SVS
import time
import serial
import numpy as np
import sys
import time
NODE_VERSION = 1.03
rospy.set_param('/version_numbers/valeport_svs', NODE_VERSION)
versionInfo="Starting SVS node Version: %s" % NODE_VERSION
rospy.loginfo(versionInfo)

sensorPort = rospy.get_param("/svs/port",'/dev/ttyUSB1')
sensorBaudrate = int(rospy.get_param("/svs/baudRate",19200))
logFlag = bool(rospy.get_param("/svs/logging",True))
topicName = rospy.get_param("/svs/topic",'/svsData')
rate = float(rospy.get_param("/svs/rate",'1'))

def talker():
	global rate, topicName, sensorBaudrate, sensorPort	
	pause=0.1	
	pub = rospy.Publisher(topicName, SVS, queue_size=1)	
	rospy.init_node('svsValues', anonymous=True)	
	rate = rospy.Rate(rate) #hz	
	# configure the serial connections (the parameters differs on the device you are connecting to)	
	ser = serial.Serial(
		port=sensorPort,
		baudrate=sensorBaudrate,
		parity=serial.PARITY_NONE,
		stopbits=serial.STOPBITS_ONE,
		bytesize=serial.EIGHTBITS
	)
	ser.isOpen()	
	svsValue=0	
	dataline=''	
	while not rospy.is_shutdown():		
		data="1"		
		while ser.inWaiting() < 1:
			time.sleep(0.1)#stops high cpu usage (sacrafices control C functionality)
			if rospy.is_shutdown():				
				print('shutdown')				
				sys.exit()		
		while ser.inWaiting() > 0:			
			data=ser.read(1).decode('utf-8')			
			if data == ' ':				
				if len(dataline) > 7:					
					svsValue=float(dataline.rstrip())					
					if logFlag:						
						print(svsValue)					
					h = std_msgs.msg.Header()					
					h.stamp = rospy.Time.now()					
					h.frame_id="svs"					
					values=SVS(header=h, svs=svsValue)					
					pub.publish(values)					
					rate.sleep()				
				dataline=''			
			else:				
				dataline=dataline+data		

if __name__ == '__main__':	
	try:		
		talker()	
	except rospy.ROSInterruptException:		
		pass
