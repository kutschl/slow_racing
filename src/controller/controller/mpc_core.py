from .racing_MPC.load_track import load_track
from .racing_MPC.get_vehicle_model import get_one_track_model, get_two_track_model
from .racing_MPC.get_OCP import get_OCP
from .racing_MPC.plot_functions import plot_track_one_track, plot_track_two_track
from .racing_MPC import prep_track
from .racing_MPC import amk
import yaml
import numpy as np
import matplotlib.pyplot as plt

class mpc_core():
    
    def __init__(self, racecar_angle, racecar_twist):
        # Parameter
        self.T = 3
        self.N = 40
        self.MODEL = 'ONE_TRACK'  # ONE_TRACK, TWO_TRACK
        self.MPC_OBJECTIVE = 'EXPLORING'  # EXPLORING, FOLLOWING

        # Load Trackdata
        if self.MPC_OBJECTIVE == 'EXPLORING':
            track_data = load_track("tracks/HRL_centerline.csv")
            self.get_logger().info(f"track_data: {track_data.shape}")
            track_data = track_data[::10]    # Apply scaling to x and y columns
            self.get_logger().info(f"track_data: {track_data.shape}")
            track_data = track_data / 20.0
            print(f"track_data: {track_data[:5]}")
            
            fill1 = np.full((track_data.shape[0], 1), 2.5)
            fill2 = np.full((track_data.shape[0], 1), 2.5)
            track_data = np.hstack((track_data, fill1, fill2))
        elif self.MPC_OBJECTIVE == 'FOLLOWING':
            track_data = load_track("tracks/waypoints.csv")
            #track_data = track_data[::10]  # Apply scaling to x and y columns
            #track_data[:, 0:2] *= 0.1
            fill1 = np.full((track_data.shape[0], 1), 2.5)
            fill2 = np.full((track_data.shape[0], 1), 2.5)
            track_data = np.hstack((track_data, fill1, fill2))

        # Stepsize for Linearization and Optimization
        stepsize_opts = {"stepsize_prep": 0.1,
                        "stepsize_reg": 0.4}

        # Splinify Track
        self.racetrack, self.spline_lengths_raceline = prep_track.prep_track(reftrack_imp=track_data,   stepsize_opts=stepsize_opts)
        # Plot waypoints and generated racetrack
        # plot_waypoints_and_track(track_data, self.racetrack)
        
        # Load Vehicle and Optimization Parameter
        #pathpath = os.path.join( 'parameter.yaml')
        #with open(pathpath) as stream:
        #    pars = yaml.safe_load(stream)
            
        # Load Vehicle and Optimization Parameter
        #pathpath = os.path.join(os.getcwd(), 'controller', , 'parameter.yaml')
        pathpath = "/sim_ws/install/controller/lib/python3.10/site-packages/controller/parameter.yaml"
        with open(pathpath) as stream:
            pars = yaml.safe_load(stream)
            #/sim_ws/src/controller/controller/parameter.yaml
            #/sim_ws/install/controller/lib/python3.10/site-packages/controller/parameter.yaml
    
        # Get Vehicle Model
        self.model = get_one_track_model(self.racetrack, pars, self.MPC_OBJECTIVE)
        
        # Current Position along racetrack - sehr innefizient, aber macht erstmal seinen job
        s_cur, w_cur = amk.path_matching_global(path_cl=self.racetrack[:,0:3], 
                                                ego_position=np.array([72.65,
                                                                    9.9 ]) ) #y, x
        mu_ref_idx = np.argmin(np.abs(self.racetrack[:,0] - s_cur))
        mu_ref = self.racetrack[mu_ref_idx, 3]
        mu_cur = racecar_angle - mu_ref # heading
        mu_cur = (mu_cur + np.pi) % (2 * np.pi) - np.pi
        
        #self.get_logger().info(f"s_cur: {s_cur}, w_cur: {w_cur}, mu_cur: {mu_cur}")
        #x0 = np.array([s_cur, w_cur, mu_cur, v_x, v_y, rotation um z, Gas/Bremssignal [-1;1], Lenkwinkel in rad])
        self.x0 = np.array([ s_cur, w_cur, mu_cur, racecar_twist[0], racecar_twist[1],racecar_twist[2],
                            1, racecar_twist[2] ])
            
            
        #self.x0 = np.array([1, 0, 0, 2.5, 0, 0, 0, 0])
        #self.x0 = np.array([1, 72.65, 9.9, 2.5, 0, 0, 0, 0])
        #self.x0 = np.array([ 1, 0, 0, self.racecar_twist[0], self.racecar_twist[1], self.racecar_angle, 0.8, self.racecar_twist[2] ])

        self.qp_iter = 1

        # Get OCP Structure
        # self.ocp = get_OCP(self.model, self.N, self.T, self.x0, self.MODEL)
        self.ocp = get_OCP(self.model, self.N, self.T, self.x0, self.MODEL)
        self.once = 1
        
        self.max_n_sim = 500
        self.end_n = self.max_n_sim
        #self.t_sum = 0
        #self.t_max = 0

        self.nx = self.model.x.size()[0]
        self.nu = self.model.u.size()[0]

        self.i = 0
        #plot
        self.x_hist = np.ndarray((self.nx, self.N, self.max_n_sim))
        self.u_hist = np.ndarray((self.nu, self.N, self.max_n_sim))
        self.car_positions = np.empty((self.max_n_sim, 2))
        

        
    def mpc_solver(self, racecar_angle, racecar_twist, racecar_position):
        '''MPC'''
        # Current Position along racetrack - sehr innefizient, aber macht erstmal seinen job
        s_cur, w_cur = amk.path_matching_global(path_cl=self.racetrack[:,0:3], 
                                                ego_position=np.array([racecar_position[0], 
                                                                    racecar_position[1] ]) ) #y, x
        mu_ref_idx = np.argmin(np.abs(self.racetrack[:,0] - s_cur))
        mu_ref = self.racetrack[mu_ref_idx, 3]
        mu_cur = racecar_angle - mu_ref # heading
        mu_cur = (mu_cur + np.pi) % (2 * np.pi) - np.pi
        
        #self.get_logger().info(f"self.x0 before calculation: {self.x0}")

        #x0 = np.array([s_cur, w_cur, mu_cur, v_x, v_y, rotation um z, Gas/Bremssignal [-1;1], Lenkwinkel in rad])
        self.x0 = np.array([ s_cur, w_cur, mu_cur, racecar_twist[0], racecar_twist[1], racecar_twist[2],
                            1, racecar_twist[2]])

        #self.get_logger().info(f"self.x0 after : {self.x0}")

        # set initial condition
        self.ocp.set(0, "lbx", self.x0)
        self.ocp.set(0, "ubx", self.x0)
    
        # Solve OCP           
        success = self.ocp.solve()
        if success:
            self.get_logger().info(f"OCP solved successfully.")
        else:
            self.get_logger().error(f"Failed to solve OCP.")

        '''here its going steps '''
        # Save Data in Struct
        # for j in range(self.N):
        #     self.x0 = self.ocp.get(j, "x")
        #     self.u0 = self.ocp.get(j, "u")
        #     self.u0 = self.ocp.get(0, "u")
        #     self.get_logger().info(f"Updated u0: {self.u0}")

        # Set State for next iteration
        for j in range(self.N):
            self.x0 = self.ocp.get(j, "x")
            self.u0 = self.ocp.get(j, "u")
            for k in range(self.nx):
                self.x_hist[k, j, self.i] = self.x0[k]
            for k in range(self.nu):
                self.u_hist[k, j, self.i] = self.u0[k]
        
        self.x0 = self.ocp.get(0, "x")
        self.u0 = self.ocp.get(0, "u")
        
        self.get_logger().info(f"self.x0 after solve : {self.u0}")
        
        if self.x0[0] > self.racetrack[-1, 0]:
            self.end_n = self.i
            self.x_hist = self.x_hist[:, :, :self.end_n]
            self.u_hist = self.u_hist[:, :, :self.end_n]
            
        # Track car's X and Y position over time
        self.car_positions[self.i , 0] = self.racecar_position[0] # X position
        self.car_positions[self.i , 1] = self.racecar_position[1]  # Y position              
        
        self.i += 1
        
        if(self.i >= self.max_n_sim):
            total_track_time = self.end_n * self.T / self.N
            self.get_logger().info("Total track time: {:.3f} s".format(total_track_time))
            # Plot the car path
            plot_track_ros(self.x_hist, self.racetrack, self.car_positions)
            if self.MODEL == 'ONE_TRACK':
                keep = plot_track_one_track(self.x_hist, self.racetrack)
                #self.i = 0
                
        
def plot_track_ros(x_hist, racetrack, car_positions,  save_path="car_trajectory.png"):
    plt.figure(figsize=(10, 6))
    
    # Plot the racetrack
    plt.plot(racetrack[:, 1], racetrack[:, 2], label="Track Centerline", color='black', linewidth=2)

    plt.scatter(car_positions[:, 0], car_positions[:, 1], label="Car Path", color='green', linewidth=2, zorder=3)
    
    plt.scatter(car_positions[0, 0], car_positions[0, 1], label="Car Path", color='red', linewidth=2, zorder=5)
    
    # Highlight positions where X and Y are both under 5 (blue)
    under_5_indices = np.where((car_positions[:, 0] < 5) & (car_positions[:, 1] < 5))
    plt.scatter(car_positions[under_5_indices, 0], car_positions[under_5_indices, 1], 
                label=f"Under (5,5) {len(under_5_indices)}", color='blue', linewidth=2, zorder=6)
    