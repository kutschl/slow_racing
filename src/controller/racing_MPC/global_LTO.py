import time
import import_track
import prep_track
import yaml
import os
from casadi import *
import numpy as np
from plot_functions_lto import plot_track_one_track, plot_track_two_track, para2global

# Model type
MODEL = 'TWO_TRACK'  # ONE_TRACK, TWO_TRACK

# Specify Trackdata
file_name_input = "fsg18_waypoints.csv"
file_name_output = "fsg18_optimized.csv"

# Performance Counter
tic = time.perf_counter()
print("Performance Timer Started")

# Import Track
track_data_import = import_track.import_track(file_name_input)

# Stepsize for Linearization and Optimization
stepsize_opts = {"stepsize_prep": 0.5,
                 "stepsize_reg": 0.2}

# Splinify Track
reftrack, spline_lengths_refline = prep_track.prep_track(reftrack_imp=track_data_import,
                                                         stepsize_opts=stepsize_opts)

# Load Vehicle and Optimization Parameter
with open("parameter.yaml") as stream:
    pars = yaml.safe_load(stream)

# Direct Orthogonal Gauss-Legendre Collocation
# Degree of Interpolating Polynomial
d = 3

# Get Collocation Points
tau_root = np.append(0, collocation_points(d, 'legendre'))

# Coefficients of the Collocation Equation
C = np.zeros((d + 1, d + 1))

# Coefficients of the Continuity Equation
D = np.zeros(d + 1)

# Coefficients of the Quadrature Function
B = np.zeros(d + 1)

# Construct Polynomial Basis
for j in range(d + 1):
    # Construct Lagrange Polynomials to get the Polynomial Basis at the Collocation Point
    p = np.poly1d([1])
    for r in range(d + 1):
        if r != j:
            p *= np.poly1d([1, -tau_root[r]]) / (tau_root[j] - tau_root[r])

    # Evaluate the Polynomial at the final time to get the Coefficients of the Continuity Equation
    D[j] = p(1.0)

    # Evaluate the time Derivative of the Polynomial at all Collocation Points to get the Coefficients of the
    # Continuity Equation
    pder = np.polyder(p)
    for r in range(d + 1):
        C[j, r] = pder(tau_root[r])

    # Evaluate the Integral of the Polynomial to get the Coefficients of the Quadrature Function
    pint = np.polyint(p)
    B[j] = pint(1.0)

# Close Track
discr_points = np.arange(reftrack.shape[0])
no_points_orig = reftrack.shape[0]

kappa_refline_cl = np.append(reftrack[:, 4], reftrack[0, 4])
discr_points_cl = np.append(discr_points, no_points_orig)
w_boarder_left_cl = np.append(reftrack[:, 5], reftrack[0, 5])
w_boarder_right_cl = np.append(reftrack[:, 6], reftrack[0, 6])

# Step Size for Optimization
h = stepsize_opts["stepsize_reg"]

# Optimization Steps
steps = [i for i in range(discr_points_cl.size)]

# Control Intervals
N = steps[-1]

# Interpolate Kappa -> Steps
kappa_interp = interpolant('kappa_interp', 'linear', [steps], kappa_refline_cl)

# Interpolate Track Border -> Steps
w_boarder_left = interpolant('w_boarder_left', 'linear', [steps], w_boarder_left_cl)
w_boarder_right = interpolant('w_boarder_right', 'linear', [steps], w_boarder_right_cl)

# Load Parameter
m = pars['m']
lf = pars['lf']
lr = pars['lr']
lb = pars['lb']
r_wheel = pars['r_wheel']
Cm1 = pars['Cm1']
Cm2 = pars['Cm2']
Cr2 = pars['Cr2']
Iz = pars['Iz']
Iw = pars['Iw']
g = pars['g']
B_lat = pars['B_lat']
C_lat = pars['C_lat']
D1_lat = pars['D1_lat']
D2_lat = pars['D2_lat']
B_long = pars['B_long']
C_long = pars['C_long']
D1_long = pars['D1_long']
D2_long = pars['D2_long']
road_mu = pars['road_mu']
eps_long = pars['eps_long']
eps_pot = pars['eps_pot']
w_safety = pars['w_safety']

# Load State Constraints
mu_min = -pars['mu_max']
mu_max = pars['mu_max']
v_x_min = pars['v_x_min']
v_x_max = pars['v_x_max']
v_y_min = -pars['v_y_max']
v_y_max = pars['v_y_max']
r_min = -pars['r_max']
r_max = pars['r_max']
T_min = pars['T_min']
T_max = pars['T_max']
delta_min = -pars['delta_max']
delta_max = pars['delta_max']

# Load Controls Constraints
d_D_min = -pars['d_D_max']
d_D_max = pars['d_D_max']
d_delta_min = -pars['d_delta_max']
d_delta_max = pars['d_delta_max']

if MODEL == 'ONE_TRACK':
    # State Variables
    w = MX.sym("w")
    mu = MX.sym("mu")
    v_x = MX.sym("v_x")
    v_y = MX.sym("v_y")
    r = MX.sym("r")
    Torque = MX.sym("Torque")
    delta = MX.sym("delta")

    x = vertcat(w, mu, v_x, v_y, r, Torque, delta)

    # Control Variables
    d_D = MX.sym("d_D")
    d_delta = MX.sym("d_delta")

    u = vertcat(d_D, d_delta)

    # Curvature Kappa
    kappa = MX.sym('kappa')

    # Dynamics
    # Drivetrain
    F_x = (Cm1 - Cm2 * v_x) * Torque - Cr2 * v_x * v_x

    # Slip Angle
    alpha_f = - atan2((lf * r + v_y), fabs(v_x)) + delta
    alpha_r = atan2((lr * r - v_y), fabs(v_x))

    # Normal Force
    F_nf = - lr / (lf + lr) * m * g * 1 / 2
    F_nr = - lf / (lf + lr) * m * g * 1 / 2

    # Pajecka Parameter
    D_f = (D1_lat + D2_lat / 1000 * F_nf) * F_nf
    D_r = (D1_lat + D2_lat / 1000 * F_nr) * F_nr

    # Lateral Force
    F_yf = - 2 * road_mu * D_f * sin(C_lat * atan(B_lat * alpha_f * 180 / pi))
    F_yr = - 2 * road_mu * D_r * sin(C_lat * atan(B_lat * alpha_r * 180 / pi))

    s_dot_inv = 1 / ((v_x * cos(mu) - v_y * sin(mu)) / (1 - w * kappa))

    # Dynamic System Equation
    xdot = vertcat((v_x * sin(mu) + v_y * cos(mu)) * s_dot_inv,
                   (r * s_dot_inv - kappa),
                   (1 / m * (F_x - F_yf * sin(delta) + m * v_y * r)) * s_dot_inv,
                   (1 / m * (F_yr + F_yf * cos(delta) - m * v_x * r)) * s_dot_inv,
                   (1 / Iz * (F_yf * lf * cos(delta) - F_yr * lr)) * s_dot_inv,
                   d_D * s_dot_inv,
                   d_delta * s_dot_inv)

    # Objective term
    Q = np.diag([0, 0, 0, 0, 0, 0, 0])
    R = np.diag([1e-4, 1e-2])
    x_ref = np.array([0, 0, 0, 0, 0, 0, 0]).T

    L = s_dot_inv * 1e1 + (x - x_ref).T @ Q @ (x - x_ref) + u.T @ R @ u

    # State Constraints
    x_init_lb = ([- w_boarder_right(0) + lb + w_safety, mu_min, v_x_min, v_y_min, r_min, T_min, delta_min])
    x_init_ub = ([w_boarder_left(0) - lb - w_safety, mu_max, v_x_max, v_y_max, r_max, T_max, delta_max])
    x_init_guess = ([0, 0, 7, 0, 0, 0, 0])

    x_const_lb = ([- 10, mu_min, v_x_min, v_y_min, r_min, T_min, delta_min])
    x_const_ub = ([10, mu_max, v_x_max, v_y_max, r_max, T_max, delta_max])
    x_const_guess = ([0, 0, 7, 0, 0, 0, 0])

    # Controls Constraints
    u_const_lb = ([d_D_min, d_delta_min])
    u_const_ub = ([d_D_max, d_delta_max])
    u_guess = ([0, 0])

    # Continuous time dynamics
    f = Function('f', [x, u, kappa], [xdot, L], ['x', 'u', 'kappa'], ['xdot', 'L'])
    F_yf_func = Function('F_yf_func', [x], [F_yf], ['x'], ['F_yf'])
    F_yr_func = Function('F_yr_func', [x], [F_yr], ['x'], ['F_yr'])
    F_x_func = Function('F_x_func', [x], [F_x], ['x'], ['F_x'])

elif MODEL == 'TWO_TRACK':
    # State Variables
    w = MX.sym("w")
    mu = MX.sym("mu")
    v_x = MX.sym("v_x")
    v_y = MX.sym("v_y")
    r = MX.sym("r")
    omega_fl = MX.sym("omega_fl")
    omega_fr = MX.sym("omega_fr")
    omega_rl = MX.sym("omega_rl")
    omega_rr = MX.sym("omega_rr")
    delta = MX.sym("delta")

    x = vertcat(w, mu, v_x, v_y, r, omega_fl, omega_fr, omega_rl, omega_rr, delta)

    # Control Variables
    D_fl = MX.sym("D_fl")
    D_fr = MX.sym("D_fr")
    D_rl = MX.sym("D_rl")
    D_rr = MX.sym("D_rr")
    d_delta = MX.sym("d_delta")

    u = vertcat(D_fl, D_fr, D_rl, D_rr, d_delta)

    # Curvature Kappa
    kappa = MX.sym('kappa')

    # Dynamics
    # Wheel Velocity
    v_y_fl = v_y + lf * r
    v_y_fr = v_y + lf * r
    v_y_rl = v_y - lr * r
    v_y_rr = v_y - lr * r

    v_x_fl = v_x - lb * r
    v_x_fr = v_x + lb * r
    v_x_rl = v_x - lb * r
    v_x_rr = v_x + lb * r

    v_c_fl = v_y_fl * cos(delta) - v_x_fl * sin(delta)
    v_c_fr = v_y_fr * cos(delta) - v_x_fr * sin(delta)
    v_c_rl = v_y_rl
    v_c_rr = v_y_rr

    v_l_fl = v_y_fl * sin(delta) + v_x_fl * cos(delta)
    v_l_fr = v_y_fr * sin(delta) + v_x_fr * cos(delta)
    v_l_rl = v_x_rl
    v_l_rr = v_x_rr

    # Side Slip Angle
    alpha_fl = atan2(v_c_fl, v_l_fl)
    alpha_fr = atan2(v_c_fr, v_l_fr)
    alpha_rl = atan2(v_c_rl, v_l_rl)
    alpha_rr = atan2(v_c_rr, v_l_rr)

    # Longitudinal Slip
    slip_fl = (r_wheel * omega_fl - v_l_fl) / v_l_fl
    slip_fr = (r_wheel * omega_fr - v_l_fr) / v_l_fr
    slip_rl = (r_wheel * omega_rl - v_l_rl) / v_l_rl
    slip_rr = (r_wheel * omega_rr - v_l_rr) / v_l_rr

    # Normal Force
    F_nf = - lr / (lf + lr) * m * g * 1 / 2
    F_nr = - lf / (lf + lr) * m * g * 1 / 2

    # Pajecka Parameter
    D_long_f = (D1_long + D2_long / 1000 * F_nf) * F_nf
    D_long_r = (D1_long + D2_long / 1000 * F_nr) * F_nr
    D_lat_f = (D1_lat + D2_lat / 1000 * F_nf) * F_nf
    D_lat_r = (D1_lat + D2_lat / 1000 * F_nr) * F_nr

    # Tire Longitudinal Force
    F_l_fl = road_mu * D_long_f * sin(C_long * atan(B_long * slip_fl))
    F_l_fr = road_mu * D_long_f * sin(C_long * atan(B_long * slip_fr))
    F_l_rl = road_mu * D_long_r * sin(C_long * atan(B_long * slip_rl))
    F_l_rr = road_mu * D_long_r * sin(C_long * atan(B_long * slip_rr))

    # Tire Lateral Force
    F_c_fl = road_mu * D_lat_f * sin(C_lat * atan(B_lat * alpha_fl * 180 / pi))
    F_c_fr = road_mu * D_lat_f * sin(C_lat * atan(B_lat * alpha_fr * 180 / pi))
    F_c_rl = road_mu * D_lat_r * sin(C_lat * atan(B_lat * alpha_rl * 180 / pi))
    F_c_rr = road_mu * D_lat_r * sin(C_lat * atan(B_lat * alpha_rr * 180 / pi))

    # Vehicle Longitudinal Forces
    F_x_fl = F_l_fl * cos(delta) - F_c_fl * sin(delta)
    F_x_fr = F_l_fr * cos(delta) - F_c_fr * sin(delta)
    F_x_rl = F_l_rl
    F_x_rr = F_l_rr

    # Vehicle Lateral Forces
    F_y_fl = F_l_fl * sin(delta) + F_c_fl * cos(delta)
    F_y_fr = F_l_fr * sin(delta) + F_c_fr * cos(delta)
    F_y_rl = F_c_rl
    F_y_rr = F_c_rr

    s_dot_inv = 1 / ((v_x * cos(mu) - v_y * sin(mu)) / (1 - w * kappa))

    # Dynamic System Equation
    xdot = vertcat((v_x * sin(mu) + v_y * cos(mu)) * s_dot_inv,
                   (r * s_dot_inv - kappa),
                   (1 / m * (F_x_fl + F_x_fr + F_x_rl + F_x_rr + m * v_y * r - Cr2 * v_x * v_x)) * s_dot_inv,
                   (1 / m * (F_y_fl + F_y_fr + F_y_rl + F_y_rr - m * v_x * r)) * s_dot_inv,
                   (1 / Iz * (lf * (F_y_fl + F_y_fr) - lr * (F_y_rl + F_y_rr) + lb * (
                               -F_x_fl + F_x_fr - F_x_rl + F_x_rr))) * s_dot_inv,
                   (1 / Iw * (D_fl * 13.6 - r_wheel * F_l_fl)) * s_dot_inv,
                   (1 / Iw * (D_fr * 13.6 - r_wheel * F_l_fr)) * s_dot_inv,
                   (1 / Iw * (D_rl * 13.6 - r_wheel * F_l_rl)) * s_dot_inv,
                   (1 / Iw * (D_rr * 13.6 - r_wheel * F_l_rr)) * s_dot_inv,
                   (d_delta * s_dot_inv))

    # Objective term
    Q = np.diag([0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
    R = np.diag([1e-4, 1e-4, 1e-4, 1e-4, 1e-2])
    x_ref = np.array([0, 0, 0, 0, 0, 0, 0, 0, 0, 0]).T

    L = s_dot_inv * 1e1 + (x - x_ref).T @ Q @ (x - x_ref) + u.T @ R @ u

    # State Constraints
    x_init_lb = ([- w_boarder_right(0) + lb + w_safety, mu_min, v_x_min, v_y_min, r_min, 5, 5, 5, 5, delta_min])
    x_init_ub = ([w_boarder_left(0) - lb - w_safety, mu_max, v_x_max, v_y_max, r_max, 120, 120, 120, 120, delta_max])
    x_init_guess = ([0, 0, 17, 0, 0, 70, 70, 70, 70, 0])

    x_const_lb = ([- 10, mu_min, v_x_min, v_y_min, r_min, 5, 5, 5, 5, delta_min])
    x_const_ub = ([10, mu_max, v_x_max, v_y_max, r_max, 120, 120, 120, 120, delta_max])
    x_const_guess = ([0, 0, 17, 0, 0, 70, 70, 70, 70, 0])

    # Controls Constraints
    u_const_lb = ([-12, -12, -12, -12, d_delta_min])
    u_const_ub = ([12, 12, 12, 12, d_delta_max])
    u_guess = ([0, 0, 0, 0, 0])

    # Continuous time dynamics
    f = Function('f', [x, u, kappa], [xdot, L], ['x', 'u', 'kappa'], ['xdot', 'L'])
    F_c_fl_func = Function('F_c_fl_func', [x], [F_c_fl], ['x'], ['F_c_fl'])
    F_c_fr_func = Function('F_c_fr_func', [x], [F_c_fr], ['x'], ['F_c_fr'])
    F_c_rl_func = Function('F_c_rl_func', [x], [F_c_rl], ['x'], ['F_c_rl'])
    F_c_rr_func = Function('F_c_rr_func', [x], [F_c_rr], ['x'], ['F_c_rr'])

    F_l_fl_func = Function('F_l_fl_func', [x], [F_l_fl], ['x'], ['F_l_fl'])
    F_l_fr_func = Function('F_l_fr_func', [x], [F_l_fr], ['x'], ['F_l_fr'])
    F_l_rl_func = Function('F_l_rl_func', [x], [F_l_rl], ['x'], ['F_l_rl'])
    F_l_rr_func = Function('F_l_rr_func', [x], [F_l_rr], ['x'], ['F_l_rr'])
else:
    raise ValueError('Selected Model is not valid!')

nx = x.shape[0]
nu = u.shape[0]

# Start with an empty NLP
w = []
w0 = []
lbw = []
ubw = []
J = 0
g = []
lbg = []
ubg = []

# For plotting x and u given w
x_plot = []
u_plot = []

# "Lift" initial conditions
Xk = MX.sym('X0', nx)
w.append(Xk)
lbw.append(x_init_lb)
ubw.append(x_init_ub)
w0.append(x_init_guess)
x_plot.append(Xk)

# Formulate the NLP
print("Formulate Problem...")
for k in range(N):
    # New NLP variable for the control
    Uk = MX.sym('U_' + str(k), nu)
    w.append(Uk)
    lbw.append(u_const_lb)
    ubw.append(u_const_ub)
    w0.append(u_guess)
    u_plot.append(Uk)

    # State at collocation points
    Xc = []
    for j in range(d):
        Xkj = MX.sym('X_' + str(k) + '_' + str(j), nx)
        Xc.append(Xkj)
        w.append(Xkj)
        lbw.append(x_const_lb)
        ubw.append(x_const_ub)
        w0.append(x_const_guess)

    # Loop over collocation points
    Xk_end = D[0] * Xk
    for j in range(1, d + 1):
        # Expression for the state derivative at the collocation point
        xp = C[0, j] * Xk
        for r in range(d):
            xp = xp + C[r + 1, j] * Xc[r]

        kappa_col = kappa_interp(k + tau_root[j])

        # Append collocation equations
        fj, qj = f(Xc[j - 1], Uk, kappa_col)
        g.append(h * fj - xp)
        lbg.append([0] * nx)
        ubg.append([0] * nx)

        # Add contribution to the end state
        Xk_end = Xk_end + D[j] * Xc[j - 1]

        # Add contribution to quadrature function
        J = J + B[j] * qj * h

    # New NLP variable for state at end of interval
    Xk = MX.sym('X_' + str(k + 1), nx)
    w.append(Xk)

    # Track Width Constrtaints
    w_left_constaint = w_boarder_left(k + 1)
    w_right_constaint = w_boarder_right(k + 1)

    lbw.append([- w_right_constaint + lb + w_safety])
    lbw.append(x_const_lb[1:])
    ubw.append([w_left_constaint - lb - w_safety])
    ubw.append(x_const_ub[1:])
    w0.append(x_const_guess)
    x_plot.append(Xk)

    if MODEL == 'ONE_TRACK':
        # Front Tire Friction Ellipse
        g.append((eps_long * F_x_func(Xk) / 2) ** 2 + F_yf_func(Xk) ** 2)
        lbg.append([-np.inf])
        ubg.append([(eps_pot * 2 * road_mu * D_f) ** 2])

        # Back Tire Friction Ellipse
        g.append((eps_long * F_x_func(Xk) / 2) ** 2 + F_yr_func(Xk) ** 2)
        lbg.append([-np.inf])
        ubg.append([(eps_pot * 2 * road_mu * D_r) ** 2])
    elif MODEL == 'TWO_TRACK':
        # Front Left Tire Friction Ellipse
        g.append((F_l_fl_func(Xk) / (D_long_f * road_mu)) ** 2 + (F_c_fl_func(Xk) / (D_lat_f * road_mu)) ** 2)
        lbg.append([0])
        ubg.append([eps_pot ** 2])

        # Front Right Tire Friction Ellipse
        g.append((F_l_fr_func(Xk) / (D_long_f * road_mu)) ** 2 + (F_c_fr_func(Xk) / (D_lat_f * road_mu)) ** 2)
        lbg.append([0])
        ubg.append([eps_pot ** 2])

        # Back Left Tire Friction Ellipse
        g.append((F_l_rl_func(Xk) / (D_long_r * road_mu)) ** 2 + (F_c_rl_func(Xk) / (D_lat_r * road_mu)) ** 2)
        lbg.append([0])
        ubg.append([eps_pot ** 2])

        # Back Right Tire Friction Ellipse
        g.append((F_l_rr_func(Xk) / (D_long_r * road_mu)) ** 2 + (F_c_rr_func(Xk) / (D_lat_r * road_mu)) ** 2)
        lbg.append([0])
        ubg.append([eps_pot ** 2])


    # Add equality constraint
    g.append(Xk_end - Xk)
    lbg.append([0] * nx)
    ubg.append([0] * nx)

g.append(w[0] - Xk)
lbg.append([0] * nx)
ubg.append([0] * nx)

# Concatenate vectors
w = vertcat(*w)
g = vertcat(*g)
x_plot = horzcat(*x_plot)
u_plot = horzcat(*u_plot)
w0 = np.concatenate(w0)
lbw = np.concatenate(lbw)
ubw = np.concatenate(ubw)
lbg = np.concatenate(lbg)
ubg = np.concatenate(ubg)

# Create an NLP solver
opts = {"expand": True,
        "ipopt.max_iter": 2000,
        "ipopt.tol": 1e-8}
prob = {'f': J, 'x': w, 'g': g}
solver = nlpsol('solver', 'ipopt', prob, opts)

# Function to get x and u trajectories from w
trajectories = Function('trajectories', [w], [x_plot, u_plot], ['w'], ['x', 'u'])

# Solve the NLP
print("Solving Problem...")
sol = solver(x0=w0, lbx=lbw, ubx=ubw, lbg=lbg, ubg=ubg)
x_opt, u_opt = trajectories(sol['x'])
x_opt = x_opt.full()  # to numpy array
u_opt = u_opt.full()  # to numpy array

s_opt = np.asarray(discr_points_cl) * h
x_opt = np.vstack((s_opt, x_opt))

# Convert to Cartesian Coordinates
racetrack_x = np.ndarray([x_opt.shape[1]])
racetrack_y = np.ndarray([x_opt.shape[1]])
racetrack_w_left = np.ndarray([x_opt.shape[1]])
racetrack_w_rigth = np.ndarray([x_opt.shape[1]])

for j in range(x_opt.shape[1]):
    racetrack_x[j], racetrack_y[j], _ = para2global(x_opt[0, j], x_opt[1, j], x_opt[2, j], reftrack)
    racetrack_w_left[j] = w_boarder_left(j) - x_opt[1, j]
    racetrack_w_rigth[j] = w_boarder_right(j) + x_opt[1, j]


racetrack_cartesian = np.vstack((racetrack_x, racetrack_y, racetrack_w_left, racetrack_w_rigth)).T
np.savetxt("fsg18_opt_one.csv", racetrack_cartesian, delimiter=",")

#racetrack, spline_lengths_raceline = prep_track.prep_track(reftrack_imp=racetrack_cartesian,
#                                                           stepsize_opts=stepsize_opts)

#np.savetxt("track_data3.txt", racetrack, delimiter=",")

# Output
print("Total computation time: {:.3f} s".format(time.perf_counter() - tic))
# Plot the result
if MODEL == 'ONE_TRACK':
    plot_track_one_track(x_opt, reftrack)
elif MODEL == 'TWO_TRACK':
    plot_track_two_track(x_opt, u_opt, reftrack)