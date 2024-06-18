#!/usr/bin/env python

import roslib
import rospy
import rospkg
from rospy import init_node, Subscriber, Publisher, get_param
from rospy import Rate, is_shutdown, ROSInterruptException, spin, on_shutdown
from barc.msg import ECU, mod_ECU, Encoder
from tf.transformations import euler_from_quaternion
import numpy as np
from math import atan2
from geometry_msgs.msg import Point, Twist, PoseStamped
from std_msgs.msg import Float64, String
from numpy import pi
from datetime import datetime
import csv

from como_tracks.oval_track_sim import * 

def bound_servo_angle(servo_angle):
    if servo_angle > 0:
        servo_angle -= np.pi * 2
    else:
        servo_angle += np.pi * 2
    return servo_angle

x_track, y_track, heading_track, waypoint_x_track, waypoint_y_track = [], [], [], [], []

rospack = rospkg.RosPack()
package_path = rospack.get_path('centralized_server')

def log_track_data(x_track, y_track):
    timestamp = datetime.now()
    formatted_timestamp = timestamp.strftime("%Y-%m-%d_%H:%M:%S")
    fileName = "oval_track_single_lane_pos" + formatted_timestamp + ".txt"

    path = os.path.join(package_path, 'data', 'logs', fileName)
    f = open(path, 'w')
    f.write("x,y,heading,waypoint_x,waypoint_y\n")
    for i in range(len(x_track)):
        row = str(x_track[i]) + ',' + str(y_track[i]) + ',' + str(heading_track[i]) + ',' + str(waypoint_x_track[i]) + ',' + str(waypoint_y_track[i]) + '\n'
        f.write(row)
    f.close()

def save_track();
    headers = ['x', 'y', 'heading', 'waypoint_x', 'waypoint_y']
    data = []
    for i in range(len(x_track)):
        row = []
        row.append(x_track[i])
        row.append(y_track[i])
        row.append(heading_track[i])
        row.append(waypoint_x_track[i])
        row.append(waypoint_y_track[i])
        data.append(row)

    timestamp = datetime.now()
    formatted_timestamp = timestamp.strftime("%Y-%m-%d_%H:%M:%S")
    fileName = "oval_track_single_lane_pos" + formatted_timestamp + ".csv"
    path = os.path.join(package_path, 'data', 'logs', fileName)
    writer = csv.writer(open(path, 'w'), delimiter=',', lineterminator='\n')
    writer.writerow(headers)
    writer.writerows(data)

def main():
    rospy.init_node("speed_controller")
    #nh = Publisher('ecu_pwm', ECU, queue_size=10)
    nh = Publisher('ecu', mod_ECU, queue_size=10)
    rateHz = 100 # 120 Hz is preferred, but 100 Hz is configured in launch file (vrpn_client_ros)
    dt = 1.0/rateHz
    rate = Rate(rateHz)

    file = os.path.join(package_path, 'src', 'virtual_track', 'como_tracks', 'tracks', 'oval_track_single_centerline4.txt')
    print(file)
    x, y = load_tack_txt(file)
    x0, y0 = x[0], y[0]
    x1, y1 = x[1], y[1]
    dyn = 'singleTrack'

    v = 1
    kp = 1
    theta0 = np.arctan2(y1 - y0, x1 - x0)
    lookahead = 0.25
    dt = 0.05
    steer_min, steer_max = -np.pi / 4, np.pi / 4

    robot = SingleTrack(x=x0, y=y0, heading=theta0, v=v, lr=0.1, lf=0.1, dt=dt)

    while not rospy.is_shutdown():
        msg = rospy.wait_for_message("/vrpn_client_node/COMO4/pose", PoseStamped)
        robot_pos = np.array([msg.pose.position.x, msg.pose.position.y])
        waypoints_ahead_x, waypoints_ahead_y, dist = get_waypoints_ahead_looped(x, y, robot_pos, lookahead)
        wp = np.array([waypoints_ahead_x[-1], waypoints_ahead_y[-1]])
        
        x_track.append(robot_pos[0])
        y_track.append(robot_pos[1])

        goal_x = wp[0]
        goal_y = wp[1]
        waypoint_x_track.append(goal_x)
        waypoint_y_track.append(goal_y)
        
        print("Pos", robot_pos[0], robot_pos[1])
        rot_q = msg.pose.orientation
        (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
        inc_x = goal_x - robot_pos[0]
        inc_y = goal_y - robot_pos[1]
        print("Way", goal_x, goal_y)
        print("Dis", inc_x, inc_y)
        heading_track.append(theta)

        angle_to_goal = atan2(inc_y, inc_x) - np.pi/2
        print("Ang", theta, angle_to_goal)
        distance_to_goal = np.sqrt(np.square(inc_x) + np.square(inc_y))
        servo_to_goal = (theta - angle_to_goal)
        if abs(servo_to_goal) > np.pi:
            servo_to_goal = bound_servo_angle(servo_to_goal)
        print(servo_to_goal)
        
        #throttle = 1630
        motor = 6.5 
        
        '''
        steering = 1524 + 250 * servo_to_goal
        if steering > 1824:
            steering = 1824
        elif steering < 1224:
            steering = 1224
        print(throttle, steering)
        '''
        servo_to_goal = np.clip(servo_to_goal, steer_min, steer_max)
        servo_to_goal += np.pi/2

        motor_gain = 1.0
        steering_gain = 1.0

        ecu_cmd = mod_ECU(motor, servo_to_goal, motor_gain, steering_gain)
        #ecu_cmd = ECU(throttle, steering)
        nh.publish(ecu_cmd)
        rate.sleep()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        #log_track_data(x_track, y_track)
        save_track()
        rospy.logfatal("ROS Interrupt. Shutting down speed_controller node")
        pass
