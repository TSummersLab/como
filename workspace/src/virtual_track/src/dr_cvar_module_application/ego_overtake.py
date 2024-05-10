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
from double_integrator_dynamics_model.como_simulations import reach_avoid
import threading

package_path = find_package_path('virtual_track')
post_processing_done = threading.Event()

def main():
    rospy.init_node("dr_cvar_module_planner")
    rateHz = 10 # Assuming small rate for planning module, to be 1/10 of real-time controller
    dt = 1.0/rateHz
    rate = Rate(rateHz)

    #TODO: Regenerate/Modify selected track
    file = os.path.join(package_path, 'src', 'dr_cvar_module_application', 'como_tracks', 'tracks', 'otsl_track.csv')

    # TODO: Specify Ego vehicle
    ego_id = "COMO4" #TODO: Modify to be based on rosparam?
    ego_vehicle = VehicleSub(ego_id)

    # TODO: Repeat for each obstacle
    obstacle_ids = ["COMO2"]
    obstacle_vehicles = {}
    for obstacle_id in obstacle_ids:
        obstacle_vehicles[obstacle_id] = VehicleSub(obstacle_id)


    while not rospy.is_shutdown():
        #TODO: Add function calls for DR-CVaR module
        print("Planner node is running")
        reach_avoid(seed=2023, exp_type='ego_overtaking', metric='drcvar', filter_slack=False, samp_dist='norm', realize_dist='lap', show_traj=True, show_col_dist=True, show_cvxpy_data=True, plot_traj=True, plot_col_dist=True, plot_cvxpy_data=True, xlim=(-3, 3), ylim=(-5, 4), figsize=None, track=file)

def shutdown_handler():
    print("Planner node is shutting down")

if __name__ == "__main__":
    try:
        rospy.on_shutdown(shutdown_handler)
        main()
        post_processing_done.wait()
    except rospy.ROSInterruptException:
        rospy.logfatal("ROS Interrupt. Shutting down dr-cvar-planner node")
        pass
