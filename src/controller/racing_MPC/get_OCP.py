import numpy as np
import scipy.linalg
from acados_template import AcadosModel, AcadosOcp, AcadosOcpSolver
from casadi import vertcat

def get_OCP(model, N, T, x0, MODEL):
    # Parameter
    nx = model.x.size()[0]
    nu = model.u.size()[0]
    ny = nx + nu
    ny_e = nx
    ns = 2

    # Preload Structure
    ocp = AcadosOcp()
    ocp.dims.N = N

    # Define Acados Model
    acados_model = AcadosModel()
    acados_model.f_impl_expr = model.f_impl
    acados_model.f_expl_expr = model.f_expl
    acados_model.x = model.x
    acados_model.xdot = model.x_dot
    acados_model.u = model.u
    acados_model.name = "Spatial_Model"
    ocp.model = acados_model

    # Setup Cost
    ocp.cost.cost_type = "EXTERNAL"
    ocp.cost.cost_type_e = "EXTERNAL"

    ocp.model.cost_expr_ext_cost = model.cost_y_expr
    ocp.model.cost_expr_ext_cost_e = model.cost_y_expr_e

    # Slack
    ocp.cost.zl = 100 * np.ones((ns,))
    ocp.cost.zu = 100 * np.ones((ns,))
    ocp.cost.Zl = 1 * np.ones((ns,))
    ocp.cost.Zu = 1 * np.ones((ns,))

    # Contstraints
    ocp.constraints.lbx = model.constraints_lbx
    ocp.constraints.ubx = model.constraints_ubx
    ocp.constraints.idxbx = model.constraints_idxbx

    ocp.constraints.lbu = model.constraints_lbu
    ocp.constraints.ubu = model.constraints_ubu
    ocp.constraints.idxbu = model.constraints_idxbu

    ocp.model.con_h_expr = model.constraints_expr
    ocp.constraints.lh = model.constraints_lh
    ocp.constraints.uh = model.constraints_uh

    ocp.constraints.lsh = np.zeros(ns)
    ocp.constraints.ush = np.zeros(ns)
    if MODEL == 'ONE_TRACK':
        ocp.constraints.idxsh = np.array([2, 3])
    elif MODEL == 'TWO_TRACK':
        ocp.constraints.idxsh = np.array([4, 5])

    # Setup Initial Condition
    ocp.constraints.x0 = x0

    # Setup Solver
    ocp.solver_options.tf = T
    ocp.solver_options.qp_solver = "FULL_CONDENSING_HPIPM"
    ocp.solver_options.nlp_solver_type = "SQP_RTI"
    ocp.solver_options.hessian_approx = "GAUSS_NEWTON"
    ocp.solver_options.integrator_type = "ERK"
    ocp.solver_options.sim_method_num_stages = 4
    ocp.solver_options.sim_method_num_steps = 3
    ocp.solver_options.ext_cost_num_hess = 0

    # Create Solver
    ocp = AcadosOcpSolver(ocp, json_file="acados_ocp.json")

    return ocp
