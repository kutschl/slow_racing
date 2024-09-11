import numpy as np
from casadi import *


def get_one_track_model(track_data, pars, MPC_OBJECTIVE):
      # Save Data
    model = types.SimpleNamespace()

    # Parameter
    m = pars['m']
    lf = pars['lf']
    lr = pars['lr']
    Cm1 = pars['Cm1']
    Cm2 = pars['Cm2']
    Cr2 = pars['Cr2']

    # Interpolate Track Data
    s_ref = track_data[:, 0]
    kappa_ref = -track_data[:, 4]
    kappa = interpolant("kappa", "bspline", [s_ref], kappa_ref)

    # State Variables
    s = MX.sym("s")
    w = MX.sym("w")
    mu = MX.sym("mu")
    v = MX.sym("v")
    D = MX.sym("D")
    delta = MX.sym("delta")

    x = vertcat(s, w, mu, v, D, delta)

    # State Variables Dot
    s_dot = MX.sym("s_dot")
    w_dot = MX.sym("w_dot")
    mu_dot = MX.sym("mu_dot")
    v_dot = MX.sym("v_dot")
    D_dot = MX.sym("D_dot")
    delta_dot = MX.sym("delta_dot")

    x_dot = vertcat(s_dot, w_dot, mu_dot, v_dot, D_dot, delta_dot)

    # Control Variables
    d_D = MX.sym("d_D")
    d_delta = MX.sym("d_delta")

    u = vertcat(d_D, d_delta)

    # Dynamics
    # Drivetrain
    F_x = (Cm1 - Cm2 * v) * D - Cr2 * v * v

    beta = atan(lr / (lr+lf) * tan(delta))
    r = v / lr * sin(beta)
    a_lat = v * r

    # Dynamic System Equation
    f_expl = vertcat(
        (v * cos(mu + beta)) / (1 - w * kappa(s)),
        v * sin(mu + beta),
        r - kappa(s) * (v * cos(mu + beta)) / (1 - w * kappa(s)),
        1 / m * F_x,
        d_D,
        d_delta,
    )

    # Cost Expression
    if MPC_OBJECTIVE == 'EXPLORING':
        q_s = 1e-1
        q_w = 1e1
        q_w_e = 1e-1
        q_mu = 1e-1
        q_mu_e = 0
        q_D = 1e-3
        q_delta = 1e-3
        cost_y_expr = 1 / ((v * cos(mu)) / (1 - w * kappa(s))) * q_s + \
                      w ** 2 * q_w + \
                      mu ** 2 * q_mu + \
                      d_D ** 2 * q_D + \
                      d_delta ** 2 * q_delta

        cost_y_expr_e = w ** 2 * q_w_e + mu ** 2 * q_mu_e

    # State Constraints
    model.constraints_lbx = np.array([-10, -pars['mu_max'], pars['v_x_min'], pars['T_min'], -pars['delta_max']])
    model.constraints_ubx = np.array([10, pars['mu_max'], pars['v_x_max'], pars['T_max'], pars['delta_max']])
    model.constraints_idxbx = np.array([1, 2, 3, 4, 5])

    # Control Constraints
    model.constraints_lbu = np.array([-pars['d_D_max'], -pars['d_delta_max']])
    model.constraints_ubu = np.array([pars['d_D_max'], pars['d_delta_max']])
    model.constraints_idxbu = np.array([0, 1])
    
    # Nonlinear Constraints
    max_gg = 1
    model.constraints_expr = a_lat
    model.constraints_uh = np.array([max_gg])
    model.constraints_lh = np.array([-max_gg])

    # Save Data
    model.x = x
    model.x_dot = x_dot
    model.u = u
    model.cost_y_expr = cost_y_expr
    model.cost_y_expr_e = cost_y_expr_e
    model.f_expl = f_expl
    model.f_impl = f_expl - x_dot

    return model