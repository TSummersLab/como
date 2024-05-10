#!/usr/bin/env python3

import roslib
import rospy
from rospy import init_node, Subscriber, Publisher, get_param
from rospy import Rate, is_shutdown, ROSInterruptException, spin, on_shutdown
from como_tracks.load_track import *
from como_tracks.sample_trajectory_generation import *
from tools.post_processing import *
from tools.file_handling import *
from vehicle_data import VehicleSub
from ego_traj_sub import EgoTrajSub
from generate_control_inputs import apply_obstacle_controls, apply_ego_controls
from virtual_track.msg import ECU, mod_ECU
from double_integrator_dynamics_model.como_simulations import reach_avoid
import threading

package_path = find_package_path('virtual_track')
post_processing_done = threading.Event()

# Determine active vehicle
NAMESPACE = rospy.get_namespace()
NAMESPACE = NAMESPACE[:-1] # removes slash at the end of the namespace

def main():
    # TODO: Collect namespace/id for corresponding vehicle
    NAMESPACE = "COMO4"
    rospy.init_node("dr_cvar_module_" + NAMESPACE + "_controller")
    nh = Publisher('ecu', mod_ECU, queue_size=10)

    rateHz = 100 # Assuming standard operating rate for controller module, to match that of the Virtual Track controller
    dt = 1.0/rateHz
    rate = Rate(rateHz)

    file = os.path.join(package_path, 'src', 'dr_cvar_module_application', 'como_tracks', 'tracks', 'otsl_track.csv')
    x, y = load_single_lane_oval_track(file)

    print(NAMESPACE + "controller node is active")

    veh_sub = VehicleSub(NAMESPACE)
    # TODO: Clarify flag for ego/obstacle vehicles; Specify rosparam?
    ego_flag = NAMESPACE == "COMO4"

    while not rospy.is_shutdown():
        # Get current pose, update velocity estimations
        robot_pos, stamp = veh_sub.motive_sub.get_cur_pose()
        yaw = veh_sub.motive_sub.get_yaw()
        est_vel = veh_sub.est_vel_sub.get_velocity()
        veh_sub.est_vel_pub.estimate_velocity(robot_pos, stamp, est_vel)

        # Handle track data and generate waypoints?
        
        # TODO: Get ego_trajectory and corresponding control inputs
        if ego_flag:
            ego_traj_sub = EgoTrajSub()
            motor, motor_gain, servo_to_goal, servo_gain = apply_ego_controls(ego_traj_sub.get_ego_traj(), robot_pos, yaw, est_vel)
        else:
            motor, motor_gain, servo_to_goal, servo_gain = apply_obstacle_controls(x, y, robot_pos, yaw, est_vel)

        ecu_cmd = mod_ECU(motor, motor_gain, servo_to_goal, servo_gain)
        nh.publish(ecu_cmd)

        rate.sleep()

def shutdown_handler():
    print(NAMESPACE + "controller node is shutting down")

if __name__ == "__main__":
    try:
        rospy.on_shutdown(shutdown_handler)
        main()
        post_processing_done.wait()
    except rospy.ROSInterruptException:
        rospy.logfatal("ROS Interrupt. Shutting down dr-cvar-planner node")
        pass
