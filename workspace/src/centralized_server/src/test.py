#!/usr/bin/env python

import roslib
import rospy
from rospy import init_node, Subscriber, Publisher, get_param
from rospy import Rate, is_shutdown, ROSInterruptException, spin, on_shutdown
from barc.msg import ECU, Encoder
from tf.transformations import euler_from_quaternion
import numpy as np
from math import atan2
from geometry_msgs.msg import Point, Twist, PoseStamped
from std_msgs.msg import Float64
from numpy import pi


def main():
    rospy.init_node("speed_controller")
    nh = Publisher('ecu_pwm', ECU, queue_size=10)
    rateHz = 50
    dt = 1.0/rateHz
    rate = Rate(rateHz)
    while not rospy.is_shutdown():
        msg = rospy.wait_for_message("/vrpn_client_node/COMO4/pose", PoseStamped)
        msg2 = rospy.wait_for_message("/vrpn_client_node/target_drone/pose", PoseStamped)
        
        x = msg.pose.position.x
        y = msg.pose.position.y
        goalX = msg2.pose.position.x
        goalY = msg2.pose.position.y
        rot_q = msg.pose.orientation
        (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q,w])
        inc_x = goalX - x
        inc_y = goalY - y
        print(theta)
        theta = -theta
        angle_to_goal = atan2(inc_x, inc_y)
        distance_to_goal = np.sqrt(np.square(inc_x) + np.square(inc_y))
        servo_to_goal = (angle_to_goal - theta)
        throttle = 1558 + (distance_to_goal -0.2) * 18
        if throttle > 1584:
            throttle = 1584
        steering = 1492 + 335 * (angle_to_goal - theta)
        if steering > 1984:
            steering = 1984
        elif steering < 984:
            steering = 984
        if steering >= 1984 and distance_to_goal > 0.5:
            throttle = 1594
        if steering <= 984 and distance_to-_goal > 0.5:
            throttle = 1594
        ecu_cmd = ECU(throttle, steering)
        nh.publish(ecu_cmd)
        rate.sleep()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.logfatal("ROS Interrupt. Shutting down speed_controller node")
        pass
