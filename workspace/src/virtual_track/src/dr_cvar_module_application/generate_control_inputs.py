import numpy as np
from math import atan2, sqrt
from como_tracks.sample_trajectory_generation import *

def bound_servo_angle(servo_angle):
    if servo_angle > 0:
        servo_angle -= np.pi * 2
    else:
        servo_angle += np.pi * 2
    return servo_angle

def apply_obstacle_controls(x, y, robot_pos, yaw, est_vel):
    print("Obstacle controls") # Standard Virtual Track controls
    
    # Define virtual track trajectory settings
    lookahead = 0.5
    c_idx = 0

    # Select waypoints
    waypoints_x, waypoints_y, dist, c_idx = get_waypoints_heuristic(x, y, robot_pos, lookahead, c_idx)
    wp = np.array([waypoints_x[-1], waypoints_y[-1]])

    # Data about selected waypoint
    delta_x = wp[0] - robot_pos[0]
    delta_y = wp[1] - robot_pos[1]

    # Compute control inputs
    angle_to_goal = atan2(delta_y, delta_x) - np.pi/2
    distance_to_goal = np.sqrt(np.square(delta_x) + np.square(delta_y))
    servo_to_goal = (yaw - angle_to_goal)
    
    if abs(servo_to_goal) > np.pi:
        servo_angle = bound_servo_angle(servo_to_goal)

    servo_gain = 1.0
    if abs(servo_to_goal) > 0.05:
        servo_gain = (servo_to_goal/5.25 + 1)

    max_steer = np.pi/4
    min_steer = -np.pi/4
    servo_to_goal = np.clip(servo_to_goal, min_steer, max_steer)
    servo_to_goal += np.pi/2

    motor = 7.75
    motor_gain = 1
    desired_speed = 1.8

    return [motor, motor_gain, servo_to_goal, servo_gain]

def apply_ego_controls(ego_traj, robot_pos, yaw, est_vel):
    # Retrieve x and y components of the ego trajectory
    x = ego_traj.x
    y = ego_traj.y

    # Select a suitable target point
    wp = [x[-1], y[-1]]
   
    # Data about selected waypoint
    delta_x = wp[0] - robot_pos[0]
    delta_y = wp[1] - robot_pos[1]

    # Compute control inputs
    angle_to_goal = atan2(delta_y, delta_x) - np.pi/2
    distance_to_goal = np.sqrt(np.square(delta_x) + np.square(delta_y))
    servo_to_goal = (yaw - angle_to_goal)

    if abs(servo_to_goal) > np.pi:
        servo_angle = bound_servo_angle(servo_to_goal)

    servo_gain = 1.0
    if abs(servo_to_goal) > 0.05:
        servo_gain = (servo_to_goal/5.25 + 1)

    max_steer = np.pi/4
    min_steer = -np.pi/4
    servo_to_goal = np.clip(servo_to_goal, min_steer, max_steer)
    servo_to_goal += np.pi/2

    motor = 7.75
    motor_gain = 1
    desired_speed = 1.8

    return [motor, motor_gain, servo_to_goal, servo_gain]
