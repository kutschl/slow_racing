from scipy import interpolate
from scipy import optimize
from scipy import spatial
import trajectory_planning_helpers.get_rel_path_part
from typing import Union
import numpy as np
import math
import trajectory_planning_helpers as tph


def spline_approximation(track: np.ndarray,
                         k_reg: int = 3,
                         s_reg: int = 12,
                         stepsize_prep: float = 1.0,
                         stepsize_reg: float = 3.0,
                         debug: bool = False) -> np.ndarray:
    """
    author:
    Fabian Christ

    modified by:
    Alexander Heilmeier

    .. description::
    Smooth spline approximation for a track (e.g. centerline, reference line).

    .. inputs::
    :param track:           [x, y, w_tr_right, w_tr_left, (banking)] (always unclosed).
    :type track:            np.ndarray
    :param k_reg:           order of B splines.
    :type k_reg:            int
    :param s_reg:           smoothing factor (usually between 5 and 100).
    :type s_reg:            int
    :param stepsize_prep:   stepsize used for linear track interpolation before spline approximation.
    :type stepsize_prep:    float
    :param stepsize_reg:    stepsize after smoothing.
    :type stepsize_reg:     float
    :param debug:           flag for printing debug messages
    :type debug:            bool

    .. outputs::
    :return track_reg:      [x, y, w_tr_right, w_tr_left, (banking)] (always unclosed).
    :rtype track_reg:       np.ndarray

    .. notes::
    The function can only be used for closable tracks, i.e. track is closed at the beginning!
    The banking angle is optional and must not be provided!
    """

    # ------------------------------------------------------------------------------------------------------------------
    # LINEAR INTERPOLATION BEFORE SMOOTHING ----------------------------------------------------------------------------
    # ------------------------------------------------------------------------------------------------------------------

    track_interp = tph.interp_track.interp_track(track=track,
                                                 stepsize=stepsize_prep)
    track_interp_cl = np.vstack((track_interp, track_interp[0]))
    track_interp_cl = np.vstack((track, track[0]))

    # ------------------------------------------------------------------------------------------------------------------
    # SPLINE APPROXIMATION / PATH SMOOTHING ----------------------------------------------------------------------------
    # ------------------------------------------------------------------------------------------------------------------

    # create closed track (original track)
    track_cl = np.vstack((track, track[0]))
    no_points_track_cl = track_cl.shape[0]
    el_lengths_cl = np.sqrt(np.sum(np.power(np.diff(track_cl[:, :2], axis=0), 2), axis=1))
    dists_cum_cl = np.cumsum(el_lengths_cl)
    dists_cum_cl = np.insert(dists_cum_cl, 0, 0.0)

    # find B spline representation of the inserted path and smooth it in this process
    # (tck_cl: tuple (vector of knots, the B-spline coefficients, and the degree of the spline))
    tck_cl, t_glob_cl = interpolate.splprep([track_interp_cl[:, 0], track_interp_cl[:, 1]],
                                            k=k_reg,
                                            s=s_reg,
                                            per=1)[:2]

    # calculate total length of smooth approximating spline based on euclidian distance with points at every 0.25m
    no_points_lencalc_cl = math.ceil(dists_cum_cl[-1]) * 4
    path_smoothed_tmp = np.array(interpolate.splev(np.linspace(0.0, 1.0, no_points_lencalc_cl), tck_cl)).T
    len_path_smoothed_tmp = np.sum(np.sqrt(np.sum(np.power(np.diff(path_smoothed_tmp, axis=0), 2), axis=1)))

    # get smoothed path
    no_points_reg_cl = math.ceil(len_path_smoothed_tmp / stepsize_reg) + 1
    path_smoothed = np.array(interpolate.splev(np.linspace(0.0, 1.0, no_points_reg_cl), tck_cl)).T[:-1]

    # ------------------------------------------------------------------------------------------------------------------
    # PROCESS TRACK WIDTHS (AND BANKING ANGLE IF GIVEN) ----------------------------------------------------------------
    # ------------------------------------------------------------------------------------------------------------------

    # find the closest points on the B spline to input points
    dists_cl = np.zeros(no_points_track_cl)                 # contains (min) distances between input points and spline
    closest_point_cl = np.zeros((no_points_track_cl, 2))    # contains the closest points on the spline
    closest_t_glob_cl = np.zeros(no_points_track_cl)        # containts the t_glob values for closest points
    t_glob_guess_cl = dists_cum_cl / dists_cum_cl[-1]       # start guess for the minimization

    for i in range(no_points_track_cl):
        # get t_glob value for the point on the B spline with a minimum distance to the input points
        closest_t_glob_cl[i] = optimize.fmin(dist_to_p,
                                             x0=t_glob_guess_cl[i],
                                             args=(tck_cl, track_cl[i, :2]),
                                             disp=False)

        # evaluate B spline on the basis of t_glob to obtain the closest point
        closest_point_cl[i] = interpolate.splev(closest_t_glob_cl[i], tck_cl)

        # save distance from closest point to input point
        dists_cl[i] = math.sqrt(math.pow(closest_point_cl[i, 0] - track_cl[i, 0], 2)
                                + math.pow(closest_point_cl[i, 1] - track_cl[i, 1], 2))

    if debug:
        print("Spline approximation: mean deviation %.2fm, maximum deviation %.2fm"
              % (float(np.mean(dists_cl)), float(np.amax(np.abs(dists_cl)))))

    # get side of smoothed track compared to the inserted track
    sides = np.zeros(no_points_track_cl - 1)

    for i in range(no_points_track_cl - 1):
        sides[i] = tph.side_of_line.side_of_line(a=track_cl[i, :2],
                                                 b=track_cl[i+1, :2],
                                                 z=closest_point_cl[i])

    sides_cl = np.hstack((sides, sides[0]))

    # calculate new track widths on the basis of the new reference line, but not interpolated to new stepsize yet
    w_tr_right_new_cl = track_cl[:, 2] + sides_cl * dists_cl
    w_tr_left_new_cl = track_cl[:, 3] - sides_cl * dists_cl

    # interpolate track widths after smoothing (linear)
    w_tr_right_smoothed_cl = np.interp(np.linspace(0.0, 1.0, no_points_reg_cl), closest_t_glob_cl, w_tr_right_new_cl)
    w_tr_left_smoothed_cl = np.interp(np.linspace(0.0, 1.0, no_points_reg_cl), closest_t_glob_cl, w_tr_left_new_cl)

    track_reg = np.column_stack((path_smoothed, w_tr_right_smoothed_cl[:-1], w_tr_left_smoothed_cl[:-1]))

    # interpolate banking if given (linear)
    if track_cl.shape[1] == 5:
        banking_smoothed_cl = np.interp(np.linspace(0.0, 1.0, no_points_reg_cl), closest_t_glob_cl, track_cl[:, 4])
        track_reg = np.column_stack((track_reg, banking_smoothed_cl[:-1]))

    return track_reg


# ----------------------------------------------------------------------------------------------------------------------
# DISTANCE CALCULATION FOR OPTIMIZATION --------------------------------------------------------------------------------
# ----------------------------------------------------------------------------------------------------------------------

# return distance from point p to a point on the spline at spline parameter t_glob
def dist_to_p(t_glob: np.ndarray, path: list, p: np.ndarray):
    s = interpolate.splev(t_glob, path)
    return spatial.distance.euclidean(p, np.concat(s))


def path_matching_local(path: np.ndarray,
                        ego_position: np.ndarray,
                        consider_as_closed: bool = False,
                        s_tot: Union[float, None] = None,
                        no_interp_values: int = 11) -> tuple:
    """
    author:
    Alexander Heilmeier

    .. description::
    Get the corresponding s coordinate and the displacement of the own vehicle in relation to a local path.

    .. inputs::
    :param path:                Unclosed path used to match ego position ([s, x, y]).
    :type path:                 np.ndarray
    :param ego_position:        Ego position of the vehicle ([x, y]).
    :type ego_position:         np.ndarray
    :param consider_as_closed:  If the path is closed in reality we can interpolate between last and first point. This
                                can be enforced by setting consider_as_closed = True.
    :type consider_as_closed:   bool
    :param s_tot:               Total length of path in m.
    :type s_tot:                Union[float, None]
    :param no_interp_values:    Number of interpolation points that are created between the two closest points on the
                                path to obtain a more accurate result.
    :type no_interp_values:     int

    .. outputs::
    :return s_interp:           Interpolated s position of the vehicle in m.
    :rtype s_interp:            np.ndarray
    :return d_displ:            Estimated displacement from the trajectory in m.
    :rtype d_displ:             np.ndarray
    """

    # ------------------------------------------------------------------------------------------------------------------
    # CHECK INPUT ------------------------------------------------------------------------------------------------------
    # ------------------------------------------------------------------------------------------------------------------

    if path.shape[1] != 3:
        raise RuntimeError("Inserted path must have 3 columns [s, x, y]!")

    if consider_as_closed and s_tot is None:
        print("WARNING: s_tot is not handed into path_matching_local function! Estimating s_tot on the basis of equal"
              "stepsizes")
        s_tot = path[-1, 0] + path[1, 0] - path[0, 0]  # assume equal stepsize

    # ------------------------------------------------------------------------------------------------------------------
    # SELF LOCALIZATION ON RACELINE ------------------------------------------------------------------------------------
    # ------------------------------------------------------------------------------------------------------------------

    # get the nearest path point to ego position
    dists_to_cg = np.hypot(path[:, 1] - ego_position[0], path[:, 2] - ego_position[1])
    ind_min = np.argpartition(dists_to_cg, 1)[0]

    # get previous and following point on path
    if consider_as_closed:
        if ind_min == 0:
            ind_prev = dists_to_cg.shape[0] - 1
            ind_follow = 1

        elif ind_min == dists_to_cg.shape[0] - 1:
            ind_prev = ind_min - 1
            ind_follow = 0

        else:
            ind_prev = ind_min - 1
            ind_follow = ind_min + 1

    else:
        ind_prev = max(ind_min - 1, 0)
        ind_follow = min(ind_min + 1, dists_to_cg.shape[0] - 1)

    # get angle between selected point and neighbours: ang1 to previous point, ang2 to following point on path
    ang_prev = np.abs(trajectory_planning_helpers.angle3pt.angle3pt(path[ind_min, 1:3],
                                                                    ego_position,
                                                                    path[ind_prev, 1:3]))

    ang_follow = np.abs(trajectory_planning_helpers.angle3pt.angle3pt(path[ind_min, 1:3],
                                                                      ego_position,
                                                                      path[ind_follow, 1:3]))

    # extract neighboring points -> closest point and the point resulting in the larger angle
    if ang_prev > ang_follow:
        a_pos = path[ind_prev, 1:3]
        b_pos = path[ind_min, 1:3]
        s_curs = np.append(path[ind_prev, 0], path[ind_min, 0])
    else:
        a_pos = path[ind_min, 1:3]
        b_pos = path[ind_follow, 1:3]
        s_curs = np.append(path[ind_min, 0], path[ind_follow, 0])

    # adjust s if closed path shell be considered and we have the case of interpolation between last and first point
    # Todo: Check right handling of mu in thi
    if consider_as_closed:
        if ind_min == 0 and ang_prev > ang_follow:
            s_curs[1] = s_tot
        elif ind_min == dists_to_cg.shape[0] - 1 and ang_prev <= ang_follow:
            s_curs[1] = s_tot

    # interpolate between those points (linear) for better positioning
    t_lin = np.linspace(0.0, 1.0, no_interp_values)  # set relative lengths that are evaluated for interpolation
    x_cg_interp = np.linspace(a_pos[0], b_pos[0], no_interp_values)
    y_cg_interp = np.linspace(a_pos[1], b_pos[1], no_interp_values)

    # get nearest of those interpolated points relative to ego position
    dists_to_cg = np.hypot(x_cg_interp - ego_position[0], y_cg_interp - ego_position[1])
    ind_min_interp = np.argpartition(dists_to_cg, 1)[0]
    t_lin_used = t_lin[ind_min_interp]

    # ------------------------------------------------------------------------------------------------------------------
    # CALCULATE REQUIRED INFORMATION -----------------------------------------------------------------------------------
    # ------------------------------------------------------------------------------------------------------------------

    # calculate current path length
    s_interp = np.interp(t_lin_used, (0.0, 1.0), s_curs)

    # get displacement between ego position and path (needed for lookahead distance)
    d_displ = dists_to_cg[ind_min_interp]

    return s_interp, d_displ

def path_matching_global(path_cl: np.ndarray,
                         ego_position: np.ndarray,
                         s_expected: Union[float, None] = None,
                         s_range: float = 20.0,
                         no_interp_values: int = 11) -> tuple:
    """
    author:
    Alexander Heilmeier

    .. description::
    Get the corresponding s coordinate and the displacement of the own vehicle in relation to the global path.

    .. inputs::
    :param path_cl:         Closed path used to match ego position ([s, x, y]).
    :type path_cl:          np.ndarray
    :param ego_position:    Ego position of the vehicle ([x, y]).
    :type ego_position:     np.ndarray
    :param s_expected:      Expected s position of the vehicle in m.
    :type s_expected:       Union[float, None]
    :param s_range:         Range around expected s position of the vehicle to search for the match in m.
    :type s_range:          float
    :param no_interp_values:    Number of interpolation points that are created between the two closest points on the
                                path to obtain a more accurate result.
    :type no_interp_values:     int

    .. outputs::
    :return s_interp:       Interpolated s position of the vehicle in m. The following holds: s_interp in range
                            [0.0,s_tot[.
    :rtype s_interp:        float
    :return d_displ:        Estimated displacement from the trajectory in m.
    :rtype d_displ:         float
    """

    # ------------------------------------------------------------------------------------------------------------------
    # CHECK INPUT ------------------------------------------------------------------------------------------------------
    # ------------------------------------------------------------------------------------------------------------------

    if path_cl.shape[1] != 3:
        raise RuntimeError("Inserted path must have 3 columns [s, x, y]!")

    # ------------------------------------------------------------------------------------------------------------------
    # GET RELEVANT PART OF PATH FOR EXPECTED S -------------------------------------------------------------------------
    # ------------------------------------------------------------------------------------------------------------------

    # get s_tot into a variable
    s_tot = path_cl[-1, 0]

    if s_expected is not None:
        path_rel = trajectory_planning_helpers.get_rel_path_part.get_rel_path_part(path_cl=path_cl,
                                                                                   s_pos=s_expected,
                                                                                   s_dist_back=s_range,
                                                                                   s_dist_forw=s_range)[0]

        # path must not be considered closed specifically as it is continuous and unclosed by construction
        consider_as_closed = False

    else:
        path_rel = path_cl[:-1]

        # path is unclosed to keep every point unique but must be considered closed to get proper matching between
        # last and first point
        consider_as_closed = True

    # ------------------------------------------------------------------------------------------------------------------
    # USE PATH MATCHING FUNCTION ON RELEVANT PART ----------------------------------------------------------------------
    # ------------------------------------------------------------------------------------------------------------------

    # get s_interp and d_displ
    s_interp, d_displ = path_matching_local(path=path_rel,
                            ego_position=ego_position,
                            consider_as_closed=consider_as_closed,
                            s_tot=s_tot,
                            no_interp_values=no_interp_values)

    # cut length if bigger than s_tot
    if s_interp >= s_tot:
        s_interp -= s_tot

    # now the following holds: s_interp -> [0.0; s_tot[

    return s_interp, d_displ
