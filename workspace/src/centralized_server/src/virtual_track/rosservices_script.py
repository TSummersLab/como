#!/usr/bin/env python

'''
This program is designed to provide rosservices to COMOs running the virtual track program so that the COMOs can stop moving, change speeds, or change lanes.
-------------------------------
File Name:     rosservices_scripts
Author :       Aditya Sinha
Last updated : July 2nd, 2024
'''

import rospy

from centralized_server.srv import StartStop, StartStopResponse, LaneSwitch, LaneSwitchResponse, ChangeSpeed, ChangeSpeedResponse

# Subscribes to /start_stop rosservice used to start or stop a COMO from moving
class StartStopSrv:
	def __init__(self):
		self.stop_flag = False
		self.service = rospy.Service('/start_stop', StartStop, self.set_start_stop)

	def set_start_stop(self, req):
		self.stop_flag = req.stop
		return StartStopResponse(success=True)

	def get_stop_flag(self):
		return self.stop_flag

# Subscribes to /lane_switch rosservice used to change lanes in given track
class LaneSwitchSrv:
	def __init__(self):
		self.lane = 0
		self.service = rospy.Service('/lane_switch', LaneSwitch, self.set_lane)

	def set_lane(self, req):
		self.lane = req.lane
		return LaneSwitchResponse(success=True)

	def get_lane(self):
		return self.lane

# Subscribes to /change_speed rosservice used change throttle speed of the COMO
class ChangeSpeedSrv:
	def __init__(self):
		self.speed = 7.25
		self.service = rospy.Service('/change_speed', ChangeSpeed, self.set_speed)

	def set_speed(self, req):
		self.speed = req.speed
		return ChangeSpeedResponse(success=True)

	def get_speed(self):
		return self.speed
