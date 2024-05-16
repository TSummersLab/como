import numpy as np
from como_tracks.load_track import load_single_lane_oval_track
from como_tracks.sample_trajectory_generation import get_waypoints_heuristic
from backend.dynamic_vehicles import CircularSingleIntegrator, CircularDoubleIntegrator
from backend.safe_halfspaces import DRCVaRHalfspace, \
        CVaRHalfspace, MeanHalfspace
from backend.safety_filters import MPCFilter, MPCFilterWithSlack
from backend.ref_traj_generation import MPCReferenceTrajectory
from vehicle_data import VehicleSub

def como_exp_setup(exp_type, metric, filter_slack, track):
    """
    COMO experiment setup
    :param exp_type: ['ego_overtaking']
    :param metric: ['mean', 'cvar', 'drcvar']
    :param filter_slack: True/False
    :return:
    """

    ENV_LIM = 5 # Switch to use lab_x, lab_y from roslaunch params
    LAB_X = 6.096
    LAB_Y = 9.144

    # Define track representation
    x, y = load_single_lane_oval_track(track)

    # optimization problem solver
    SOLVER = 'ECOS'

    # Experiment settings
    t0 = 0 # start time
    dt = 0.1 # discrete time step
    
    # Vehicle Settings
    radius = 0.75

    # MPC filter settings
    horizon = 10 # MPC time horizon
    mpc_Q, mpc_QT, mpc_R = 2., 5., 1
    filter_solver = SOLVER

    # Reference trajectory setting
    ego_ref_Q, ego_ref_QT, ego_ref_R = 1., 3., 1.
    traj_gen_solver = SOLVER

    # safe halfspace params
    num_samp = 20 # number of samples
    delta = 0.1 # loss bound
    alpha = 0.2 # alpha-worst cases (when applicable)
    eps = 0.05 # wasserstein ball (when applicable)

    # General optimization problem settings
    # max_steer = np.pi/4
    accel_lim_x = 100
    accel_lim_y = 100
    # TODO: Add vehicle specific dynamic/mechanical constraints

    measurement_Ax_leq_b = {'A': np.array([np.array([1, 0]), np.array([-1, 0]), np.array([0, 1]), np.array([0, -1])]),
                            'b': np.array([ENV_LIM, ENV_LIM, ENV_LIM, ENV_LIM])}
    control_Ax_leq_b = {'A': np.array([np.array([1, 0]), np.array([-1, 0]), np.array([0, 1]), np.array([0, -1])]),
                        'b': np.array([accel_lim_x, accel_lim_x, accel_lim_y, accel_lim_y])}

    noise_std_dev = np.array([0, 0])  # noise std dev to sample data from [0.1, 0.1] previously used

    # Save data
    fig_name = metric + '_' + exp_type
    if filter_slack:
        fig_name += "_with_slack"
    EGO_COLOR, EGO_ALPHA = 'tab:blue', 0.2
    OBST_COLORS, OBST_ALPHA = ['tab:red', 'tab:orange', 'tab:pink'], 0.2


    # Define experiment configuration and data
    if exp_type == 'ego_overtaking':
        total_time = 1.5 # total time in seconds
        sim_steps = int(np.ceil(total_time / dt))

        # Collect sensor and external data for the specified ego vehicle
        ego_veh_id = "COMO2" #TODO: Modify to be selected via rosparam
        ego_veh_sub = VehicleSub(ego_veh_id, pose=[2.0, -1.5, 0], orientation=[0, 0, -np.pi/2], velocity=1.0)

        # Ego vehicle setup
        ego_radius = radius
        ego_init_pos = ego_veh_sub.motive_sub.get_pose()
        ego_veh_vel = ego_veh_sub.est_vel_sub.get_velocity()
        ego_init_vel = [0, -1 * ego_veh_vel]
        ego_init_state = np.hstack([ego_init_pos, ego_init_vel])
        ego_veh = CircularDoubleIntegrator(ego_init_state, dt, t0, ego_radius)
        
        # Determine goal_state (waypoint) [for current timestep]
        c_idx = 0
        waypoints_ahead_x, waypoints_ahead_y, dist, c_idx = get_waypoints_heuristic(x, y, ego_init_pos, ego_veh_vel * horizon * dt, c_idx)
        wp = np.array([waypoints_ahead_x[-1], waypoints_ahead_y[-1]])
        print(ego_init_pos)
        print(wp)
        ego_goal_state = np.array([wp[0], wp[1], 0, 0])

        # Define obstacle(s) data for the application
        obst_veh_ids = ["COMO4"]
        obst_init_positions = []
        obst_ref_vels = []
        obst_radii = []
        for id in obst_veh_ids:
            obst_radius = radius
            obst_radii.append(obst_radius)
            obst_veh_sub = VehicleSub(id, pose=[-2.0, 2.0, 0], orientation=[0, 0, np.pi/2], velocity=1.0)
            obst_init_pos = obst_veh_sub.motive_sub.get_pose()
            obst_init_positions.append(obst_init_pos)
            obst_init_vel = [0, 1 * obst_veh_sub.est_vel_sub.get_velocity()]
            obst_ref_vels.append(obst_init_vel)
        obst_init_states = obst_init_positions

    else:
        raise NotImplementedError('Experiment type not supported')

    #TODO: Generate obstacle(s) from data
    obst_vehs = [CircularSingleIntegrator(obst_init_state, dt, t0, obst_radius) for obst_init_state, obst_radius in zip(obst_init_states, obst_radii)]
    num_obst = len(obst_vehs)

    # Safe halfspace
    if metric == 'drcvar':
        safe_hs = DRCVaRHalfspace(alpha, eps, delta, num_samp, solver=SOLVER)
    elif metric == 'cvar':
        safe_hs = CVaRHalfspace(alpha, delta, num_samp, loss_type='continuous', solver=SOLVER)
    elif metric == 'mean':
        safe_hs = MeanHalfspace()
    else:
        raise NotImplementedError('Invalid risk metric')
    
    # solve for the first time to eliminate CVXPY time overhead
    safe_hs.set_opt_pb_params(np.zeros([2, ]), np.zeros([2, num_samp]), [0])
    safe_hs.solve_opt_pb()

    # Reference Trajectory Generator
    ego_traj_gen = MPCReferenceTrajectory(horizon, ego_veh.A, ego_veh.B, Q=ego_ref_Q, QT=ego_ref_QT, R=ego_ref_R, control_Ax_leq_b=control_Ax_leq_b)

    # MPC Filter
    Q = np.eye(ego_veh.n) * mpc_Q
    QT = np.eye(ego_veh.n) * mpc_QT
    R = np.eye(ego_veh.m) * mpc_R
    if filter_slack:
        mpc_filter = MPCFilterWithSlack(num_obst, ego_veh, horizon, Q, QT, R, measurement_Ax_leq_b=measurement_Ax_leq_b,
                                        control_Ax_leq_b=control_Ax_leq_b)
    else:
        mpc_filter = MPCFilter(num_obst, ego_veh, horizon, Q, QT, R, measurement_Ax_leq_b=measurement_Ax_leq_b,
                               control_Ax_leq_b=control_Ax_leq_b)
    
    return(ENV_LIM, 
            x, y,
            dt, horizon,
            num_samp, sim_steps,
            num_obst,
            ego_veh, EGO_COLOR, EGO_ALPHA,
            ego_traj_gen, ego_goal_state,
            obst_vehs, OBST_COLORS, OBST_ALPHA,
            obst_init_positions, obst_ref_vels,
            traj_gen_solver, filter_solver,
            noise_std_dev,
            safe_hs, mpc_filter,
            fig_name)
