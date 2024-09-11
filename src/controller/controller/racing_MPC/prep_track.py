import numpy as np
from . import amk
import trajectory_planning_helpers as tph


def prep_track(reftrack_imp, stepsize_opts):

    # Smoothing and Interpolating Reference Track
    reftrack_interp = amk.spline_approximation(track=reftrack_imp,
                             stepsize_prep=stepsize_opts["stepsize_prep"],
                             stepsize_reg=stepsize_opts["stepsize_reg"],
                             debug=False)

    # Calculate Splines
    refpath_interp_cl = np.vstack((reftrack_interp[:, :2], reftrack_interp[0, :2]))

    coeffs_x, coeffs_y, a_interp, normvec_normalized_interp = tph.calc_splines.\
        calc_splines(path=refpath_interp_cl)

    spline_lengths_refline = tph.calc_spline_lengths.calc_spline_lengths(coeffs_x=coeffs_x,
                                                                         coeffs_y=coeffs_y)

    psi_refline, kappa_refline = tph.calc_head_curv_an. \
        calc_head_curv_an(coeffs_x=coeffs_x,
                           coeffs_y=coeffs_y,
                           ind_spls=np.arange(0, spline_lengths_refline.size),
                           t_spls=spline_lengths_refline)

    s_refline = np.cumsum(spline_lengths_refline)
    reftrack = np.column_stack((s_refline, reftrack_interp[:, :2], psi_refline, kappa_refline, reftrack_interp[:, 2:4]))

    return reftrack, spline_lengths_refline
