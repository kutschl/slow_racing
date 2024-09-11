import time
import os
import yaml
import numpy as np
from load_track import load_track
from get_vehicle_model import get_one_track_model
from get_OCP import get_OCP
from plot_functions import plot_track_one_track, plot_track_two_track, plot_waypoints_and_track
import prep_track
import amk
'''
init start 
'''

# Parameter
T = 3
N = 40
MODEL = 'ONE_TRACK'  # ONE_TRACK, TWO_TRACK
MPC_OBJECTIVE = 'EXPLORING'  # EXPLORING, FOLLOWING

# Load Trackdata
if MPC_OBJECTIVE == 'EXPLORING':
    track_data = load_track("tracks/waypoints.csv")
    fill1 = np.full((track_data.shape[0], 1), 2.5)
    fill2 = np.full((track_data.shape[0], 1), 2.5)
    track_data = np.hstack((track_data, fill1, fill2))
elif MPC_OBJECTIVE == 'FOLLOWING':
    track_data = load_track("tracks/waypoints.csv")
    fill1 = np.full((track_data.shape[0], 1), 2.5)
    fill2 = np.full((track_data.shape[0], 1), 2.5)
    track_data = np.hstack((track_data, fill1, fill2))

# Stepsize for Linearization and Optimization
stepsize_opts = {"stepsize_prep": 0.1,
                 "stepsize_reg": 0.4}

# Splinify Track
racetrack, spline_lengths_raceline = prep_track.prep_track(reftrack_imp=track_data,
                                                           stepsize_opts=stepsize_opts)

# Load Vehicle and Optimization Parameter
pathpath = os.path.join(os.getcwd(), 'src', 'controller', 'controller', 'racing_MPC', 'parameter.yaml')
with open(pathpath) as stream:
    pars = yaml.safe_load(stream)
    

# Get Vehicle Model
model = get_one_track_model(racetrack, pars, MPC_OBJECTIVE)
x0 = np.array([1.31854376, 0.07005226, 0.07426934, 1.91234433, 0, 0])
qp_iter = 1

# Get OCP Structure
ocp = get_OCP(model, N, T, x0, MODEL)

max_n_sim = 1
end_n = max_n_sim
t_sum = 0
t_max = 0

nx = model.x.size()[0]
nu = model.u.size()[0]

s_cur, w_cur1 = amk.path_matching_global(path_cl=racetrack[:,0:3], ego_position=np.array([60, 10]))
s_cur, w_cur2 = amk.path_matching_global(path_cl=racetrack[:,0:3], ego_position=np.array([60, 20]))


print(f"w_cur1: {w_cur1}")
print(f"w_cur2: {w_cur2}")

#plot
x_hist = np.ndarray((nx, N, max_n_sim))
u_hist = np.ndarray((nu, N, max_n_sim))

'''
init end
'''

for i in range(max_n_sim):
    # Solve OCP
    
    
    t = time.time()
    
    # Current Position along racetrack - sehr innefizient, aber macht erstmal seinen job
    s_cur, w_cur = amk.path_matching_global(path_cl=racetrack[:,0:3], ego_position=np.array([30, 0])) #y, x
    mu_ref_idx = np.argmin(np.abs(racetrack[:,0] - s_cur))
    mu_ref = racetrack[mu_ref_idx, 3]
    mu_cur = 0.1 - mu_ref # heading
    mu_cur = (mu_cur + np.pi) % (2 * np.pi) - np.pi

    # x0 = np.array([s_cur, w_cur, mu_cur, v_x, v_y, rotation um z, Gas/Bremssignal [-1;1], Lenkwinkel in rad])

    # update initial condition
    ocp.set(0, "lbx", x0)
    ocp.set(0, "ubx", x0)
    for j in range(qp_iter):
        ocp.solve()
    t_elapsed = time.time() - t
    

    # Calculate Time Sum
    t_sum += t_elapsed
    if t_elapsed > t_max:
        t_max = t_elapsed

    '''here its going steps '''
    # Save Data in Struct
    for j in range(N):
        x0 = ocp.get(j, "x")
        u0 = ocp.get(j, "u")
        for k in range(nx):
            x_hist[k, j, i] = x0[k]
        for k in range(nu):
            u_hist[k, j, i] = u0[k]

    # Set State for next iteration
    x0 = ocp.get(1, "x")
    
    """ 1. Initialize MPC, see init
    Looop over
        2. ocp.set("aktueller stand" = x_vector)
        3. ocp.solve
        4. ocp.get(1, "x") lade die prädiktion
    Loop end
    """
    

    if x0[0] > racetrack[-1, 0]:
        end_n = i
        x_hist = x_hist[:, :, :end_n]
        u_hist = u_hist[:, :, :end_n]
        break

total_track_time = end_n * T / N
print("Total track time: {:.3f} s".format(total_track_time))
print("Total computation time: {:.3f} s".format(t_sum))
print("Average computation time: {:.3f} ms".format(t_sum / end_n * 1000))
print("Maximum computation time: {:.3f} ms".format(t_max * 1000))

# Plot
keep = plot_track_one_track(x_hist, racetrack)

