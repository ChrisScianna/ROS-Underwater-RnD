#!/usr/bin/env python
import rospy
from keller_9lx_pressure.msg import Pressure
from valeport_svs.msg import SVS
import socket
import time
import sys
NODE_VERSION = 1.03
rospy.set_param('/version_numbers/inputs_to_ins', NODE_VERSION)
versionInfo="Starting inputs to INS node Version: %s" % NODE_VERSION
rospy.loginfo(versionInfo)
pressureTopicName = rospy.get_param("/pressure/topic",'/pressureData')
svsTopicName = rospy.get_param("/svs/topic",'/svsData')
ipAddressINS = rospy.get_param("/inputs_to_ins/ipAddress",'192.168.36.185')
portINS = int(rospy.get_param("/inputs_to_ins/port",8117))
rate = float(rospy.get_param("/svs/rate",'1'))
logFlag = bool(rospy.get_param("/inputs_to_ins/logging",True))

while not rospy.is_shutdown():
	try:
		s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		host =ipAddressINS
		port = portINS
		s.connect((host,port))
		print("Input to INS Connected to INS")
		break
	except:
		if logFlag:
			print("Input to INS failed to connect to INS. Will try again.")
		time.sleep(1)

svsData=None
pressureData=None

def sendMessage():
	message="$SVP70,000000,"+str(svsData)+",00.0,"+str(pressureData)+",000,0,0,1\r\n"
	if logFlag:
		print(message)
	s.send(message.encode())

def callbackPressure(data):
	global svsData, pressureData
	pressureData= '%05.1f'%((data.pressure-1.01325)*10)
	if svsData is not None:
		sendMessage()

def callbackSVS(data):
	global pressureData, svsData
	svsData='%08.3f'%data.svs
	if pressureData is not None:
		sendMessage()
		
def listener():
	rospy.init_node('inputsINS', anonymous=True)
	rospy.Subscriber(pressureTopicName, Pressure, callbackPressure)
	rospy.Subscriber(svsTopicName, SVS, callbackSVS)
	rospy.spin()

if __name__ == '__main__':		
	listener()

#Code to send pressure and sound velocity data to the INS
#Message type: SVP 70 (page 313 of INS-Interface Library)
#$SVP70,000000,SSSS.SSS,00.0,PPP.P,000,0,0,1<CR><LF>
#where SSSS.SSS is Speed of sound in m/s
#and   PPP.P is Pressure in tenth of bar (zeroed at 1 atm)
