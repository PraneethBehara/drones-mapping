#!/usr/bin/env python

import struct
import rospy
import NetworkManager
from std_msgs.msg import String
from manual_measurements.msg import AccessPointInfo, WifiInfo
import numpy as np

class AccessPointsReader:

	def __init__(self):
		rospy.init_node('AccessPointsReader', anonymous=True)
		self.rate = rospy.Rate(10)
		self.pub = rospy.Publisher('APInfo', WifiInfo, queue_size=10)

	def run(self):
		while not rospy.is_shutdown():
			message = WifiInfo()
			message.header.stamp = rospy.Time.now()
			message.accesspoint = self.getAccessPoints()
			#print message
			self.pub.publish(message)
			self.rate.sleep()

	def getAccessPoints(self):
		aplist = []
		try:
			for dev in NetworkManager.NetworkManager.GetDevices():
					if dev.DeviceType != NetworkManager.NM_DEVICE_TYPE_WIFI:
						continue
					for ap in dev.GetAccessPoints():
						param = AccessPointInfo()
						param.name = str(ap.Ssid)
						param.addr = str(ap.HwAddress)
						param.strength = ap.Strength
						param.frequency = 5.0 if ap.Frequency > 3000 else 2.5
						aplist.append(param)	
		except Exception:
			pass
		return aplist[:-1]

def main():
	AccessPointsReader().run()

if __name__ == "__main__":
	main()
