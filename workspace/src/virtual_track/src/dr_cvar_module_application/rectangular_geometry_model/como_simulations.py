import time
import numpy as np
from como_tracks.sample_trajectory_generation import get_full_state_ego_trajectory, generate_sample_trajectories
from backend.printing_fxns import print_colored
from backend.simulation_functions import *
from backend.collision_check import distance_to_collision
from statistics.random_samples_fxns import generate_noise_samples
from experiment_setup import como_exp_setup
import pickle

def forward_sim_veh(horizon, veh_dyn, ref_ctrl):
    ref_traj = np.zeros([veh_dyn.n, horizon + 1])
    ref_traj[:, 0] = veh_dyn.get_state
    for t in range(horizon):
        ref_traj[:, t + 1] = veh_dyn.sim(ref_traj[:, t], ref_ctrl)
    return ref_traj

def reach_avoid(seed, exp_type, metric, filter_slack, samp_dist='norm', realize_dist='lap',
                show_traj=False, show_col_dist=False, show_cvxpy_data=False,
                plot_traj=False, plot_col_dist=False, plot_cvxpy_data=False,
                xlim=None, ylim=None, figsize=None, track=None):
    """
    Simple reach avoid problem with MPC reference trajectories that move the vehicles towards the goal
    Allows for multiple obstacle vehicles
    """
    np.random.seed(seed)

    ENV_LIM, LAB_X, LAB_Y, \
        x, y, \
        dt, horizon, \
        num_samp, sim_steps, \
        num_obst, \
        ego_veh, EGO_COLOR, EGO_ALPHA, \
        obst_vehs, OBST_COLORS, OBST_ALPHA, \
        obst_init_positions, obst_ref_vels, \
        traj_gen_solver, filter_solver, \
        noise_std_dev, \
        safe_hs, mpc_filter, \
        fig_name = como_exp_setup(exp_type, metric, filter_slack, track)

    print("COMO Exp Setup")

    # variables to save data
    safe_hs_time_info = []
    mpc_filter_time_info = []
    mpc_filter_status = []
    ego_pos_all, ego_polys_all = [ego_veh.get_pos], [ego_veh.get_patch(EGO_COLOR, EGO_ALPHA)]
    obst_pos_all, obst_polys_all = [], []
    obst_pos_now, obst_polys_now = get_obst_pos_and_patches(obst_vehs, OBST_COLORS, OBST_ALPHA) #TODO: Ensure it works for new models
    obst_pos_all.append(obst_pos_now)
    obst_polys_all.append(obst_polys_now)
    #TODO: Modify method of calculating distance to collision using new geometries
    ego_obst_dist_to_col = [distance_to_collision(ego_veh, obst_veh) for obst_veh in obst_vehs]

    safe_polys_all = []
    t_itr_times = []
    all_samples = []

    for sim_step in range(sim_steps):
        print_colored('Iteration: ' + str(sim_step), 'm')
        print_colored('--------------', 'm')

        t_itr_start = time.time()

        # Get ego reference trajectory
        #TODO: Generate ego_trajectory using como_tracks operations
        ego_idx = 0 # Assume starting index, unless specified otherwise
        ego_ref_traj = get_full_state_ego_trajectory(x, y, ego_veh.get_state, ego_idx, horizon)
        print(ego_ref_traj)
        # Double check performance of ref_trajectory, given that only 2D position (x, y) is considered

        # Get obstacle predicted trajectory
        obst_ref_trajs = []
        itr_samples = []
        for obst_veh, obst_ref_vel in zip(obst_vehs, obst_ref_vels):
            print("Determining obstacle predicted trajectories")
            ref_trajs = generate_sample_trajectories(x, y, obst_veh.get_state, num_samp, horizon, dt)
            obst_ref_trajs.extend(ref_trajs)
            #TODO: Utilize trajectory-generation operations in como_tracks
            # Look into use of forward_sim_veh as in drone_simulations?

        # Get safe halfspaces
        safe_hs_time_info.append(np.zeros([horizon, num_obst]))
        hs_A, hs_b = [np.zeros((num_obst, 2)) for _ in range(horizon)], np.zeros([num_obst, horizon])
        for t in range(horizon):
            ego_ref_state_t = ego_ref_traj[:, t + 1]
            if t == 0:
                safe_polys_t0 = []
            for ob_num, (obst_ref_traj, obst_veh) in enumerate(zip(obst_ref_trajs, obst_vehs)):
                print("Sim step:", sim_step, "Ob_num:", ob_num)
                obst_ref_state_t = obst_ref_traj[:, t + 1]
                xi = np.ones((2, num_samp))
                if noise_std_dev[0] == 0:
                    xi[0, :] *= obst_ref_state_t[0]
                else:
                    xi[0, :] = generate_noise_samples(num_samp, obst_ref_state_t[0], np.sqrt(noise_std_dev[0]), dist=samp_dist)

                if noise_std_dev[1] == 0:
                    xi[1, :] *= obst_ref_state_t[1]
                else:
                    xi[1, :] = generate_noise_samples(num_samp, obst_ref_state_t[1], np.sqrt(noise_std_dev[1]), dist=samp_dist)

                print(xi)
                ego_ref_pos_t = ego_veh.C @ ego_ref_state_t
                obst_ref_pos_t = obst_veh.C @ obst_ref_state_t
                h = (obst_ref_pos_t - ego_ref_pos_t) / np.linalg.norm(obst_ref_pos_t - ego_ref_pos_t)
                print(ego_veh.get_pos)
                print(obst_veh.get_pos)
                print(safe_hs)
                safe_hs.set_opt_pb_params(h, xi, [ego_veh.get_radius + obst_veh.get_radius])
                #TODO: Modify opt_pb_params above to use rectangular model
                solved, info = safe_hs.solve_opt_pb()
                print(safe_hs, solved, info)
                if not solved:
                    raise NotImplementedError('Safe Halfspace error: Error finding a safe halfspace')
                safe_hs_time_info[-1][t, ob_num] = info['solve_call_time'] * 1000
                A, b = safe_hs.get_Ax_leq_b_A, safe_hs.get_Ax_leq_b_b
                hs_A[t][ob_num, :] = A
                hs_b[ob_num, t] = b
                if t == 0:
                    safe_polys_t0.append(safe_hs.get_poly)
                    itr_samples.append(xi)
        
        safe_poly_intersection = None
        for i, sp in enumerate(safe_polys_t0):
            if i == 0:
                safe_poly_intersection = sp
            else:
                safe_poly_intersection = safe_poly_intersection.intersect(sp)
        safe_polys_all.append(safe_poly_intersection)
        all_samples.append(itr_samples)

        # Employ the safety filter
        mpc_filter.set_opt_pb_params(hs_A, hs_b, ego_ref_traj)
        solved, info = mpc_filter.solve_opt_pb(solver=filter_solver)
        if sim_step == 0:  # resolve only the first time to avoid CVXPY time overhead
            solved, info = mpc_filter.solve_opt_pb(solver=filter_solver)
        if not solved and (mpc_filter.x_mpc is None or mpc_filter.u_mpc is None):
            raise NotImplementedError('MPC Filter error: Error finding a safe trajectory')
        elif not solved:
            print_colored('MPC Filter FAILED. TRAJECTORY SHIFTED.', 'r')
            mpc_filter_status.append(False)
        else:
            mpc_filter_status.append(True)
        mpc_filter_time_info.append(info['solve_call_time'] * 1000)

        # Apply controls
        apply_ego_control(mpc_filter, ego_ref_traj, ego_veh, verbose=True)
        
        for obst_init_pos, obst_ref_vel, obst_veh in zip(obst_init_positions, obst_ref_vels, obst_vehs):
            obst_ctrl = np.array([0, obst_init_pos[1] - obst_veh.get_pos[1]]) / dt
            obst_ctrl[0] = obst_ref_vel[0]
            print("Apply controls")
            print(obst_veh.get_state)
            obst_veh.step(obst_ctrl)
            print(obst_veh.get_state)
            if realize_dist == 'norm':
                print(obst_veh.get_state)
                print(np.squeeze(np.array([np.random.normal(0, noise_std_dev[0], 1),
                                                              np.random.normal(0, noise_std_dev[1], 1), 0, 0, 0])))
                obst_veh.overwrite_state(obst_veh.get_state +
                                         np.squeeze(np.array([np.random.normal(0, noise_std_dev[0], 1),
                                                              np.random.normal(0, noise_std_dev[1], 1), 0, 0, 0])))
            elif realize_dist == 'lap':
                print("lap")
                print(obst_veh.get_state)
                obst_veh.overwrite_state(obst_veh.get_state +
                                         np.squeeze(np.array([np.squeeze(np.random.laplace(0, noise_std_dev[0], 1)),
                                                              np.squeeze(np.random.laplace(0, noise_std_dev[1], 1)), 0, 0, 0])))
                print(obst_veh.get_state)
            else:
                raise NotImplementedError('Realization distribution not supported')

        ego_pos_all.append(ego_veh.get_pos)
        ego_polys_all.append(ego_veh.get_patch(EGO_COLOR, EGO_ALPHA))
        obst_pos_now, obst_polys_now = get_obst_pos_and_patches(obst_vehs, OBST_COLORS, OBST_ALPHA)
        obst_pos_all.append(obst_pos_now)
        obst_polys_all.append(obst_polys_now)
        ego_obst_dist_to_col.append([distance_to_collision(ego_veh, obst_veh) for obst_veh in obst_vehs])

        t_itr_end = time.time()
        t_itr_times.append(t_itr_end - t_itr_start)

    print_colored('----------------------------------------------', 'y')
    print_colored('Total Iteration times: {}'.format(t_itr_times), 'y')
    print_colored('min, avrg, max: {}, {}, {}'.format(min(t_itr_times), np.mean(t_itr_times), max(t_itr_times)), 'y')

    data_dict = {"num_obst": num_obst, "ENV_LIM": ENV_LIM, "safe_polys_all": safe_polys_all,
                 "EGO_COLOR": EGO_COLOR, "ego_pos_all": ego_pos_all, "ego_polys_all": ego_polys_all,
                 "obst_pos_all": obst_pos_all, "obst_polys_all": obst_polys_all,
                 "mpc_filter_time_info": mpc_filter_time_info, "mpc_filter_status": mpc_filter_status,
                 "safe_hs_time_info": safe_hs_time_info,
                 "ego_obst_dist_to_col": ego_obst_dist_to_col,
                 "OBST_COLORS": OBST_COLORS, "seed": seed,
                 "samples": all_samples}

    path = os.path.join("exp_data", exp_type, metric, "slack={}".format(filter_slack))
    path_exist = os.path.exists(path)
    if not path_exist:
        os.makedirs(path)
    pickle.dump(data_dict,
                open(os.path.join(path, 'seed={}.pkl'.format(seed)), 'wb'))

    fig_name = os.path.join(path, 'traj_seed={}'.format(seed))
    plot_sim_trajs(num_obst, ENV_LIM, safe_polys_all,
                   EGO_COLOR, ego_pos_all, ego_polys_all,
                   OBST_COLORS, obst_pos_all, obst_polys_all,
                   show=show_traj, save=plot_traj, save_file_name=fig_name+'.png',
                   xlim=xlim, ylim=ylim, figsize=figsize)

    plot_sim_cvx_pb_data(mpc_filter_time_info, mpc_filter_status, safe_hs_time_info, OBST_COLORS,
                         show=show_cvxpy_data, save=plot_cvxpy_data, save_file_name=fig_name + '_cvx_data.png')

    plot_ego_obst_dist_to_collision(ego_obst_dist_to_col, OBST_COLORS,
            show=show_col_dist, save=plot_col_dist, save_file_name=fig_name + '_col_dist.png')


def main():
    reach_avoid(seed=2023, exp_type='ego_overtaking', metric='drcvar', filter_slack=False,
                samp_dist='norm', realize_dist='lap',
                show_traj=True, show_col_dist=True, show_cvxpy_data=True,
                plot_traj=True, plot_col_dist=True, plot_cvxpy_data=True,
                xlim=(-5, 5), ylim=(-3, 4), figsize=None)


if __name__ == "__main__":
    main()
