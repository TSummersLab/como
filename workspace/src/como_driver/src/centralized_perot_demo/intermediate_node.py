#!/usr/bin/env python

import roslib
import rospy
from barc.msg import ECU
from como_driver.srv import Choice,ChoiceResponse
from como_driver.msg import ChoicePass

NAMESPACE = rospy.get_namespace()
NAMESPACE = NAMESPACE[:-1] # removes slash at the end

class ECUPublish:
	def __init__(self):
		self.ecu = ECU(0.,0.)
		self.ecu_pub = rospy.Publisher(NAMESPACE + '/ecu', ECU, queue_size = 1)
		
	def set_ecu(self, motor, servo):
		self.ecu = ECU(float(motor), float(servo))
	
	def publish_ecu(self):
		self.ecu_pub.publish(self.ecu)

class ECUSubscribe:
	def __init__(self):
		self.ecu = ECU(0.,0.)
		self.str_sub = rospy.Subscriber(NAMESPACE + "/temp_ecu", ECU, self.callback, queue_size =1)	
	def callback(self, data):
		self.ecu = ECU(data.motor, data.servo)
		
	def get_ecu(self):
		return self.ecu

class ChoiceSub:
    def __init__(self):
        self.sent_state = "start"
        self.sent_namespace = ""
        self.str_sub = rospy.Subscriber("/choice", ChoicePass, self.callback, queue_size=10)

    def callback(self, data):
        self.sent_state = data.state
        self.sent_namespace = data.namespace

    def get_sent_state(self):
        return self.sent_state

    def get_sent_namespace(self):
        return self.sent_namespace

def main():
	rospy.init_node("intermediate_node", anonymous = True)
	ecuPublish = ECUPublish()
	ecuSubscribe = ECUSubscribe()
	rate = rospy.Rate(30)
	choiceSub = ChoiceSub()

	while not rospy.is_shutdown():
		ecu_transfer = ecuSubscribe.get_ecu()

                sent_state = choiceSub.get_sent_state()
                sent_namespace = "/" + choiceSub.get_sent_namespace()
		#print(NAMESPACE, sent_namespace)
                ecuPublish.set_ecu(ecu_transfer.motor, ecu_transfer.servo)
                if NAMESPACE == sent_namespace:
                    if sent_state == "stop":
                        ecuPublish.set_ecu(0, ecu_transfer.servo)
                    elif sent_state == "start":
                        ecuPublish.set_ecu(ecu_transfer.motor, ecu_transfer.servo)

		ecuPublish.publish_ecu()

		rate.sleep()


if __name__ == "__main__":
	try:
		main()
	except rospy.ROSInterruptException:
		rospy.logfatal("ROS Interrupt. Shutting down line_follower_ctrl node")
		pass
