import numpy as np
import amk
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

    # calculate heading and curvature (numerically)
    psi_refline, kappa_refline = tph.calc_head_curv_num. \
        calc_head_curv_num(path=reftrack_interp[:, :2],
                           el_lengths=spline_lengths_refline,
                           is_closed=True,
                           stepsize_curv_preview=0.5,
                           stepsize_curv_review=0.5,
                           stepsize_psi_preview=0.5,
                           stepsize_psi_review=0.5)

    s_refline = np.cumsum(spline_lengths_refline)
    reftrack = np.column_stack((s_refline, reftrack_interp[:, :2], psi_refline, kappa_refline, reftrack_interp[:, 2:4]))

    return reftrack, spline_lengths_refline
