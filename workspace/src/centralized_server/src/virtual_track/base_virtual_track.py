#!/usr/bin/env python

'''
This program is designed to provide the essential structure for the virtual track program (currently used by the
COMO vehicles platform) in a modular architecture. Composed of mapping and localization classes,
motion planners using the known track (of the como_tracks package) and a pure-pursuit PID controller,
the COMO vehicles (and any other platforms) can be driven autonomously on the virtual track. Additional modules are
also included for data collection, state estimation, and visualization.
-------------------------------
File Name:     base_virtual_track
Author :       Sai Bommisetty
Last updated : June 24th, 2024
'''

# Standard Library Imports
import roslib
import rospy
from rospy import init_node, Subscriber, Publisher, get_param
from rospy import Rate, is_shutdown, ROSInterruptException, spin, on_shutdown
import numpy as np
from math import atan2, sqrt
from Queue import Queue
import threading

# External Package Imports
from tf.transformations import euler_from_quaternion

# Message Imports
from barc.msg import ECU, mod_ECU, Encoder
from geometry_msgs.msg import Point, Twist, PoseStamped
from std_msgs.msg import Float64, String

# Service Imports
#from centralized_server.srv import StartStop, StartStopResponse, LaneSwitch, LaneSwitchResponse, ChangeSpeed, ChangeSpeedResponse

# Local ROS Package Imports
from rosservices_script import *
from como_tracks.oval_track_sim import *
from como_tracks.load_track import *
from post_processing import *
from file_handling import *
from real_time_operations import *
from topics import *


def bound_servo_angle(servo_angle):
    """Limit the servo angle to the bounds of [0, 2 * pi]"""
    if servo_angle > 0:
        servo_angle -= np.pi * 2
    else:
        servo_angle += np.pi * 2
    return servo_angle

# Initialize data collection entries
# todo: Set up data collection procedures using rosbag
x_track, y_track, timestamp_track = [], [], []
motive_data, waypoints, velocity_track, control_inputs = [], [], [], []

# Retrieve absolute path to current ROS package
package_path = find_package_path('centralized_server')

# Start threading event to ROS Shutdown procedure to ensure completion of data collection
post_processing_done = threading.Event()

def main():
    # Initialize ROS Nodes and Publishers
    rospy.init_node("virtual_track_controller")
    nh = Publisher('ecu', mod_ECU, queue_size=10) # Publishes ECU commands to the low_level controller
    est_vel = Publisher('est_vel', Float64, queue_size=10) # Publishes estimated velocities to the /est_vel topic

    # Establish the desired rate of operation for the program
    rateHz = 100 # 100 Hz is a suitable and tested frequency near the Optitrack Motive operational frequency (120 Hz)
    dt = 1.0/rateHz # Calculate operational frequency, for use with related models
    rate = Rate(rateHz)

    # Initialize the rosservices
    stop_srv = StartStopSrv()
    lane_switch = LaneSwitchSrv()
    speed_srv = ChangeSpeedSrv()

    # Mapping : Initialize environment with the desired virtual track data
    global x, y
    file = os.path.join(package_path, 'src', 'virtual_track', 'como_tracks', 'tracks', 'figure8_two_centerline.csv')
    lane = lane_switch.get_lane()
    currentlane = lane
    x, y = load_figure8_two_centerline_track(file, lane)

    # Initialize lookahead and steering limits for planner and controller operations
    lookahead = 0.5
    steer_min, steer_max = -np.pi / 4, np.pi / 4

    # Initialize classes for autonomy modules (Motive data, estimated velocity, data communication, etc.)
    motive_sub = MotiveSub("COMO4")
    est_vel_sub = EstVelSub()
    # todo: Include updated Virtual Track classes and VehicleSub infrastructure

    # Initialize variables for velocity estimation and localization
    global pos_queue
    pos_queue = Queue(maxsize=12)
    c_idx = 0 # Represents last known index of robot position on the virtual track. Starts at a default of 0.

    motor_gain = 1.0 # Initialize initial motor gain. Note that motor_gain is initialized outside of the program loop
    # as current controller does not actively use motor gain.

    while not rospy.is_shutdown():
        stop_flag = stop_srv.get_stop_flag() # Boolean that defines if motor speed is set to 0
	speed_int = speed_srv.get_speed() # Gets float that is used to set motor speed
        lane = lane_switch.get_lane() # Gest integer that is used to choose a lane
        # Loads new lane from track if lane is changed
        if currentlane != lane:
            x, y = load_figure8_two_centerline_track(file, lane)
            currentlane = lane

        # Localization: Retrieve the current pose, orientation, and corresponding timestamp
        robot_pos, stamp = motive_sub.get_cur_pose()
        timestamp_track.append(stamp)
        rot_q = motive_sub.get_orientation()
        (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

        # Velocity Estimation: Update position queue for velocity estimation module
        pos_queue.put([robot_pos, stamp])
        cur_estimate = est_vel_sub.avg_velocity_est()
        if pos_queue.full():
            shift_queue(pos_queue, est_vel, 2, 3, 0, cur_estimate)

        # Mapping: Retrieve waypoints from the virtual track from current position up to the predefined lookahead
        waypoints_ahead_x, waypoints_ahead_y, dist, close_idx = get_waypoints_heuristic(x, y, robot_pos, lookahead,
                                                                                        c_idx)
        c_idx = close_idx # Track most recent index of current position on the virtual track for fast searching

        # Motion Planning : Retrieve target waypoint for the pure pursuit controller
        wp = np.array([waypoints_ahead_x[-1], waypoints_ahead_y[-1]])
        goal_x = wp[0]
        goal_y = wp[1]

        # Update data collection arrays (x, y, heading, waypoints)
        x_track.append(robot_pos[0])
        y_track.append(robot_pos[1])
        motive_data.append([robot_pos[0], robot_pos[1], theta])
        waypoints.append([wp[0], wp[1]])

        # Calculate the required servo angle for the vehicle
        inc_x = goal_x - robot_pos[0]
        inc_y = goal_y - robot_pos[1]
        angle_to_goal = atan2(inc_y, inc_x) - np.pi/2
        # distance_to_goal = np.sqrt(np.square(inc_x) + np.square(inc_y))
        servo_to_goal = (theta - angle_to_goal)

        # Initialize components for controller commands
        motor = speed_int
        servo_gain = 1

        # PID controller for steering (servo)
        if abs(servo_to_goal) > np.pi:
            servo_to_goal = bound_servo_angle(servo_to_goal)

        if abs(servo_to_goal) > 0.05:
            servo_gain = (servo_to_goal/5.25 + 1)

        # Preliminary Velocity cruise control
        desired_speed = 1.8
        cur_speed = est_vel_sub.get_velocity()
        vel_diff = desired_speed - cur_speed

        # Bound and adjust servo angle for hardware
        servo_to_goal = np.clip(servo_to_goal, steer_min, steer_max)
        servo_to_goal += np.pi/2

        # If stop_flag is true set throttle to 0 to stop the COMO, else continue with defined motor speed
        if stop_flag:
            control_inputs.append([0, servo_to_goal, 0, servo_gain])
            ecu_cmd = mod_ECU(0, servo_to_goal, 0, servo_gain)
        else:
            control_inputs.append([motor, servo_to_goal, motor_gain, servo_gain])
            ecu_cmd = mod_ECU(motor, servo_to_goal, motor_gain, servo_gain)

        #control_inputs.append([motor, servo_to_goal, motor_gain, servo_gain])

        # Publish mod_ECU command to the low_level controller
        #ecu_cmd = mod_ECU(motor, servo_to_goal, motor_gain, servo_gain)
        nh.publish(ecu_cmd)

        # Pause the program loop to match the desired rate.
        rate.sleep()

def shutdown_handler():
    """Handle all shutdown events following termination of the program (data processing, cleanup processes, etc.)"""

    # Generate figures and datasets from collected data
    timestamp = generate_timestamp()
    param_log = rospy.get_param('/oval_track_follower/log')
    param_plot = rospy.get_param('/oval_track_follower/plot')
    if param_log:
        save_track(package_path, timestamp, motive_data, waypoints, timestamp_track, velocity_track, control_inputs)
    if param_plot:
        plot_track(x, y, x_track, y_track, timestamp, package_path)
    # plot_data(velocity_track, timestamp, package_path, 'velocity', 1000, 'velocity', '')

    post_processing_done.set()

if __name__ == "__main__":
    try:
        rospy.on_shutdown(shutdown_handler)
        main()
        post_processing_done.wait()
    except rospy.ROSInterruptException:
        rospy.logfatal("ROS Interrupt. Shutting down speed_controller node")
        pass
