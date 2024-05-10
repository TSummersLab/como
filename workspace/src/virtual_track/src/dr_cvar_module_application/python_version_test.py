#!/usr/bin/env python3

import rospy
from tools.file_handling import *
from vehicle_data import VehicleSub
from vehicle_data.topics_data import MotiveSub, EstVelSub, EstVelPub
from rospy import init_node, Subscriber, Publisher, get_param, Rate, is_shutdown, ROSInterruptException, spin, on_shutdown
from statistics.stat_basics import get_true_mean
from dr_cvar_safety_filtering_ros.drone_simulations import *
# from como_tracks.oval_track import *

def main():
    rospy.init_node("test_python_version")
    print("Test python version for ROS 1 Noetic")
    rate = Rate(2)
    reach_avoid(seed=2023, exp_type='ego_and_3_vehicles', metric='drcvar', filter_slack=False, samp_dist='norm', realize_dist='lap', show_traj=True, show_col_dist=True, show_cvxpy_data=True, plot_traj=True, plot_col_dist=True, plot_cvxpy_data=True, xlim=(-5, 5), ylim=(-3, 4), figsize=None)
    while not rospy.is_shutdown():
        print("Test")
        rate.sleep()

def shutdown_handler():
    print("Active shutdown handler")

if __name__ == "__main__":
    try:
        rospy.on_shutdown(shutdown_handler)
        main()
    except rospy.ROSInterruptException:
        rospy.logfatal("ROS Interrupt. Shutting down ROS node")
        pass
