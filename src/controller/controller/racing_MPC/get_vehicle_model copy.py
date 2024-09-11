import numpy as np
from casadi import *


def get_one_track_model(track_data, pars, MPC_OBJECTIVE):
    # Save Data
    model = types.SimpleNamespace()

    # Parameter
    m = pars['m']
    lf = pars['lf']
    lr = pars['lr']
    lb = pars['lb']
    w_safety = pars['w_safety']
    Cm1 = pars['Cm1']
    Cm2 = pars['Cm2']
    Cr2 = pars['Cr2']
    Iz = pars['Iz']
    g = pars['g']
    B = pars['B_lat']
    C = pars['C_lat']
    D1 = pars['D1_lat']
    D2 = pars['D2_lat']
    road_mu = pars['road_mu']
    eps_long = pars['eps_long']
    eps_pot = pars['eps_pot']

    # Interpolate Track Data
    s_ref = track_data[:, 0]
    kappa_ref = track_data[:, 4]
    w_boarder_left_ref = track_data[:, 5]
    w_boarder_right_ref = track_data[:, 6]

    kappa = interpolant("kappa", "bspline", [s_ref], kappa_ref)
    w_boarder_left = interpolant("w_boarder_left", "bspline", [s_ref], w_boarder_left_ref)
    w_boarder_right = interpolant("w_boarder_right", "bspline", [s_ref], w_boarder_right_ref)

    # State Variables
    s = MX.sym("s")
    w = MX.sym("w")
    mu = MX.sym("mu")
    v_x = MX.sym("v_x")
    v_y = MX.sym("v_y")
    r = MX.sym("r")
    D = MX.sym("D")
    delta = MX.sym("delta")

    x = vertcat(s, w, mu, v_x, v_y, r, D, delta)

    # State Variables Dot
    s_dot = MX.sym("s_dot")
    w_dot = MX.sym("w_dot")
    mu_dot = MX.sym("mu_dot")
    v_x_dot = MX.sym("v_x_dot")
    v_y_dot = MX.sym("v_y_dot")
    r_dot = MX.sym("r_dot")
    D_dot = MX.sym("D_dot")
    delta_dot = MX.sym("delta_dot")

    x_dot = vertcat(s_dot, w_dot, mu_dot, v_x_dot, v_y_dot, r_dot, D_dot, delta_dot)

    # Control Variables
    d_D = MX.sym("d_D")
    d_delta = MX.sym("d_delta")

    u = vertcat(d_D, d_delta)

    # Dynamics
    # Drivetrain
    #todo change to bachelorarbeit
    F_x = (Cm1 - Cm2 * v_x) * D - Cr2 * v_x * v_x

    # Slip Angle
    alpha_f = (lf * r + v_y)/ v_x + delta
    alpha_r = (lr * r - v_y)/ v_x

    # Lateral Force
    F_yf = - 200 * alpha_f
    F_yr = - 200 * alpha_r

    # Dynamic System Equation
    f_expl = vertcat(
        (v_x * cos(mu) - v_y * sin(mu)) / (1 - w * kappa(s)),
        v_x * sin(mu) + v_y * cos(mu),
        r - kappa(s) * (v_x * cos(mu) - v_y * sin(mu)) / (1 - w * kappa(s)),
        1 / m * (F_x - F_yf * sin(delta) + m * v_y * r),
        1 / m * (F_yr + F_yf * cos(delta) - m * v_x * r),
        1 / Iz * (F_yf * lf * cos(delta) - F_yr * lr),
        d_D,
        d_delta,
    )

    # Cost Expression
    if MPC_OBJECTIVE == 'EXPLORING':
        q_s = 1e-1
        q_w = 1e-1
        q_w_e = 1e-1
        q_mu = 0
        q_mu_e = 1
        q_D = 1e-6
        q_delta = 1e-3
        cost_y_expr = 1 / ((v_x * cos(mu) - v_y * sin(mu)) / (1 - w * kappa(s))) * q_s + \
                      w ** 2 * q_w + \
                      mu ** 2 * q_mu + \
                      d_D ** 2 * q_D + \
                      d_delta ** 2 * q_delta

        cost_y_expr_e = w ** 2 * q_w_e + mu ** 2 * q_mu_e
    elif MPC_OBJECTIVE == 'FOLLOWING':
        q_s = 1e-3
        q_w = 1e-6
        q_w_e = 1e-4
        q_mu = 1e-6
        q_mu_e = 0
        q_D = 1e-6
        q_delta = 1e-3
        cost_y_expr = 1 / ((v_x * cos(mu) - v_y * sin(mu)) / (1 - w * kappa(s))) * q_s + \
                      w ** 2 * q_w + \
                      mu ** 2 * q_mu + \
                      d_D ** 2 * q_D + \
                      d_delta ** 2 * q_delta

        cost_y_expr_e = w ** 2 * q_w_e + mu ** 2 * q_mu_e

    # State Constraints
    model.constraints_lbx = np.array([-10, -pars['mu_max'], pars['v_x_min'], -pars['v_y_max'], -pars['r_max'], pars['T_min'], -pars['delta_max']])
    model.constraints_ubx = np.array([10, pars['mu_max'], pars['v_x_max'], pars['v_y_max'], pars['r_max'], pars['T_max'], pars['delta_max']])
    model.constraints_idxbx = np.array([1, 2, 3, 4, 5, 6, 7])

    # Control Constraints
    model.constraints_lbu = np.array([-pars['d_D_max'], -pars['d_delta_max']])
    model.constraints_ubu = np.array([pars['d_D_max'], pars['d_delta_max']])
    model.constraints_idxbu = np.array([0, 1])

    # Tire Constraints
    max_alpha = 3
    model.constraints_expr = vertcat(alpha_f, alpha_r)
    model.constraints_uh = np.array([max_alpha, max_alpha])
    model.constraints_lh = np.array([-max_alpha, -max_alpha])

    # Save Data
    model.x = x
    model.x_dot = x_dot
    model.u = u
    model.cost_y_expr = cost_y_expr
    model.cost_y_expr_e = cost_y_expr_e
    model.f_expl = f_expl
    model.f_impl = f_expl - x_dot

    return model


def get_two_track_model(track_data, pars, MPC_OBJECTIVE):
    # Save Data
    model = types.SimpleNamespace()
    # todo cost NLSQARE
    # todo change phi to x instead of y axis

    # Parameter
    m = pars['m']
    lf = pars['lf']
    lr = pars['lr']
    lb = pars['lb']
    w_safety = pars['w_safety']
    Iz = pars['Iz']
    Iw = pars['Iw']
    g = pars['g']
    r_wheel = pars['r_wheel']
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
    Cr2 = pars['Cr2']

    # Interpolate Track Data
    s_ref = track_data[:, 0]
    kappa_ref = track_data[:, 4]
    w_boarder_left_ref = track_data[:, 5]
    w_boarder_right_ref = track_data[:, 6]

    kappa = interpolant("kappa", "bspline", [s_ref], kappa_ref)
    w_boarder_left = interpolant("w_boarder_left", "bspline", [s_ref], w_boarder_left_ref)
    w_boarder_right = interpolant("w_boarder_right", "bspline", [s_ref], w_boarder_right_ref)

    # State Variables
    s = MX.sym("s")
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

    x = vertcat(s, w, mu, v_x, v_y, r, omega_fl, omega_fr, omega_rl, omega_rr, delta)

    # State Variables Dot
    s_dot = MX.sym("s_dot")
    w_dot = MX.sym("w_dot")
    mu_dot = MX.sym("mu_dot")
    v_x_dot = MX.sym("v_x_dot")
    v_y_dot = MX.sym("v_y_dot")
    r_dot = MX.sym("r_dot")
    omega_fl_dot = MX.sym("omega_fl_dot")
    omega_fr_dot = MX.sym("omega_fr_dot")
    omega_rl_dot = MX.sym("omega_rl_dot")
    omega_rr_dot = MX.sym("omega_rr_dot")
    delta_dot = MX.sym("delta_dot")

    x_dot = vertcat(s_dot, w_dot, mu_dot, v_x_dot, v_y_dot, r_dot, omega_fl_dot, omega_fr_dot, omega_rl_dot,
                    omega_rr_dot, delta_dot)

    # Control Variables
    D_fl = MX.sym("D_fl")
    D_fr = MX.sym("D_fr")
    D_rl = MX.sym("D_rl")
    D_rr = MX.sym("D_rr")
    d_delta = MX.sym("d_delta")

    u = vertcat(D_fl, D_fr, D_rl, D_rr, d_delta)

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

    # Dynamic System Equation
    f_expl = vertcat(
        (v_x * cos(mu) - v_y * sin(mu)) / (1 - w * kappa(s)),
        v_x * sin(mu) + v_y * cos(mu),
        r - kappa(s) * (v_x * cos(mu) - v_y * sin(mu)) / (1 - w * kappa(s)),
        1 / m * (F_x_fl + F_x_fr + F_x_rl + F_x_rr + m * v_y * r - Cr2 * v_x * v_x),
        1 / m * (F_y_fl + F_y_fr + F_y_rl + F_y_rr - m * v_x * r),
        1 / Iz * (lf * (F_y_fl + F_y_fr) - lr * (F_y_rl + F_y_rr) + lb * (-F_x_fl + F_x_fr - F_x_rl + F_x_rr)),
        1 / Iw * (D_fl * 13.6 - r_wheel * F_l_fl),
        1 / Iw * (D_fr * 13.6 - r_wheel * F_l_fr),
        1 / Iw * (D_rl * 13.6 - r_wheel * F_l_rl),
        1 / Iw * (D_rr * 13.6 - r_wheel * F_l_rr),
        d_delta,
    )

    # Cost Expression
    if MPC_OBJECTIVE == 'EXPLORING':
        q_s = 1e-3
        q_w = 0
        q_mu = 1e-4
        q_w_e = 1e-4
        q_mu_e = 0
        q_D_fl = 1e-7
        q_D_fr = 1e-7
        q_D_rl = 1e-7
        q_D_rr = 1e-7
        q_delta = 1e-6
        cost_y_expr = 1 / ((v_x * cos(mu) - v_y * sin(mu)) / (1 - w * kappa(s))) * q_s + \
                      w ** 2 * q_w + \
                      mu ** 2 * q_mu + \
                      D_fl ** 2 * q_D_fl + \
                      D_fr ** 2 * q_D_fr + \
                      D_rl ** 2 * q_D_rl + \
                      D_rr ** 2 * q_D_rr + \
                      d_delta ** 2 * q_delta

        cost_y_expr_e = w ** 2 * q_w_e + mu ** 2 * q_mu_e
    elif MPC_OBJECTIVE == 'FOLLOWING':
        q_s = 2e-3
        q_w = 1e-6
        q_mu = 1e-6
        q_w_e = 1e-5
        q_mu_e = 0
        q_D_fl = 1e-7
        q_D_fr = 1e-7
        q_D_rl = 1e-7
        q_D_rr = 1e-7
        q_delta = 1e-6
        cost_y_expr = 1 / ((v_x * cos(mu) - v_y * sin(mu)) / (1 - w * kappa(s))) * q_s + \
                      w ** 2 * q_w + \
                      mu ** 2 * q_mu + \
                      D_fl ** 2 * q_D_fl + \
                      D_fr ** 2 * q_D_fr + \
                      D_rl ** 2 * q_D_rl + \
                      D_rr ** 2 * q_D_rr + \
                      d_delta ** 2 * q_delta

        cost_y_expr_e = w ** 2 * q_w_e + mu ** 2 * q_mu_e

    # State Constraints
    model.constraints_lbx = np.array(
        [-10, -pars['mu_max'], pars['v_x_min'], -pars['v_y_max'], -pars['r_max'], -pars['delta_max']])
    model.constraints_ubx = np.array(
        [10, pars['mu_max'], pars['v_x_max'], pars['v_y_max'], pars['r_max'], pars['delta_max']])

    model.constraints_idxbx = np.array([1, 2, 3, 4, 5, 10])

    # Controls Constraints
    model.constraints_lbu = np.array([-12, -12, -12, -12, -10])
    model.constraints_ubu = np.array([12, 12, 12, 12, 10])
    model.constraints_idxbu = np.array([0, 1, 2, 3, 4])

    # Tire Constraints
    F_tire_comb_fl = (F_l_fl / (D_long_f * road_mu)) ** 2 + (F_c_fl / (D_lat_f * road_mu)) ** 2
    F_tire_comb_fr = (F_l_fr / (D_long_f * road_mu)) ** 2 + (F_c_fr / (D_lat_f * road_mu)) ** 2
    F_tire_comb_rl = (F_l_rl / (D_long_r * road_mu)) ** 2 + (F_c_rl / (D_lat_r * road_mu)) ** 2
    F_tire_comb_rr = (F_l_rr / (D_long_r * road_mu)) ** 2 + (F_c_rr / (D_lat_r * road_mu)) ** 2
    model.constraints_expr = vertcat(F_tire_comb_fl,
                                     F_tire_comb_fr,
                                     F_tire_comb_rl,
                                     F_tire_comb_rr,
                                     w_boarder_left(s) - lb - w_safety - w,
                                     - w_boarder_right(s) + lb + w_safety - w)
    model.constraints_uh = np.array([(eps_pot) ** 2,
                                     (eps_pot) ** 2,
                                     (eps_pot) ** 2,
                                     (eps_pot) ** 2,
                                     10, 0])
    model.constraints_lh = np.array([- (eps_pot) ** 2,
                                     - (eps_pot) ** 2,
                                     - (eps_pot) ** 2,
                                     - (eps_pot) ** 2,
                                     0, -10])

    #Helper Functions
    model.F_l_fl = Function("F_l_fl", [x, u], [F_l_fl])
    model.F_c_fl = Function("F_c_fl", [x, u], [F_c_fl])

    # Save Data
    model.x = x
    model.x_dot = x_dot
    model.u = u
    model.cost_y_expr = cost_y_expr
    model.cost_y_expr_e = cost_y_expr_e
    model.f_expl = f_expl
    model.f_impl = f_expl - x_dot

    return model
