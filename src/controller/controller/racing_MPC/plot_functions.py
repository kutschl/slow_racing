from matplotlib import cm
from matplotlib import pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Slider
from matplotlib.patches import Ellipse
import numpy as np


def plot_track_one_track(x_hist, track_data):
    s = x_hist[0, :, :].T
    w = x_hist[1, :, :].T
    mu = x_hist[2, :, :].T
    v_x = x_hist[3, :, :].T
    v_y = x_hist[4, :, :].T
    r = x_hist[5, :, :].T
    D = x_hist[6, :, :].T
    delta = np.rad2deg(x_hist[7, :, :].T)

    # Calculate Cartesian Position
    x = np.ndarray([x_hist.shape[2], x_hist.shape[1]])
    y = np.ndarray([x_hist.shape[2], x_hist.shape[1]])

    for i in range(x_hist.shape[2]):
        for j in range(x_hist.shape[1]):
            x[i, j], y[i, j], _ = para2global(s[i, j], w[i, j], mu[i, j], track_data)

    # Create Plot
    fig, ax = plt.subplots()
    plt.subplots_adjust(bottom=0.25)
    plt.ylabel('y[m]')
    plt.xlabel('x[m]')

    # Plot Position Heatmap with velocity
    position = ax.scatter(x[:, 0], y[:, 0], c=v_x[:, 0], cmap=cm.rainbow, edgecolor='none', marker='o')
    color_bar = plt.colorbar(position, fraction=0.1)
    color_bar.set_label("Velocity in m/s")

    # Plot Track Referenceline
    plt.plot(track_data[:, 1], track_data[:, 2], ':', color='k')

    # Plot Boarder
    boarder_left_x = track_data[:, 1] - track_data[:, 5] * np.sin((track_data[:, 3] + np.pi / 2))
    boarder_left_y = track_data[:, 2] + track_data[:, 5] * np.cos((track_data[:, 3] + np.pi / 2))
    boarder_right_x = track_data[:, 1] + track_data[:, 6] * np.sin((track_data[:, 3] + np.pi / 2))
    boarder_right_y = track_data[:, 2] - track_data[:, 6] * np.cos((track_data[:, 3] + np.pi / 2))

    plt.plot(boarder_left_x, boarder_left_y, color='k', linewidth=1)
    plt.plot(boarder_right_x, boarder_right_y, color='k', linewidth=1)

    # Plot Vehicle Prediction
    position_prediction, = plt.plot(x[-1, :], y[-1, :], '-', color='r')

    # Shape Box
    ax.axis('equal')

    # Create Iteration Slider
    iter_axis = plt.axes([0.25, 0.1, 0.65, 0.03], facecolor='lightgoldenrodyellow')
    iter_slider = Slider(iter_axis, 'Iteration', 1, s.shape[0] - 1, valinit=s.shape[0])

    # Create Second Figure
    fig2, ax2 = plt.subplots(8, 1, figsize=(17, 13))

    ax2[0].grid(True)
    s_plot, = ax2[0].plot(s[-1, :])
    ax2[0].set_title('Progress s', fontsize=10)

    ax2[1].grid(True)
    w_plot, = ax2[1].plot(w[-1, :])
    ax2[1].set_title('Deviation w', fontsize=10)

    ax2[2].grid(True)
    mu_plot, = ax2[2].plot(mu[-1, :])
    ax2[2].set_title('Reference Angle Deviaten mu', fontsize=10)

    ax2[3].grid(True)
    v_x_plot, = ax2[3].plot(v_x[-1, :])
    ax2[3].set_title('Velocity v_x', fontsize=10)

    ax2[4].grid(True)
    v_y_plot, = ax2[4].plot(v_y[-1, :])
    ax2[4].set_title('Velocity v_y', fontsize=10)

    ax2[5].grid(True)
    r_plot, = ax2[5].plot(r[-1, :])
    ax2[5].set_title('Yaw Rate r', fontsize=10)

    ax2[6].grid(True)
    D_plot, = ax2[6].plot(D[-1, :])
    ax2[6].set_title('Acceleration Command D', fontsize=10)

    ax2[7].grid(True)
    delta_plot, = ax2[7].plot(delta[-1, :])
    ax2[7].set_title('Steering Angle delta', fontsize=10)

    for i in range(7):
        plt.setp(ax2[i].get_xticklabels(), visible=False)

    plt.tight_layout()

    def update(val):
        iter_value = int(iter_slider.val)
        iter_position = np.vstack((x[0:iter_value, 0], y[0:iter_value, 0]))
        position.set_offsets(iter_position.T)

        iter_prediction = np.vstack((x[iter_value, :], y[iter_value, :]))
        position_prediction.set_data(iter_prediction)

        s_plot.set_ydata(s[iter_value, :])
        w_plot.set_ydata(w[iter_value, :])
        mu_plot.set_ydata(mu[iter_value, :])
        v_x_plot.set_ydata(v_x[iter_value, :])
        v_y_plot.set_ydata(v_y[iter_value, :])
        r_plot.set_ydata(r[iter_value, :])
        D_plot.set_ydata(D[iter_value, :])
        delta_plot.set_ydata(delta[iter_value, :])

        for i in range(8):
            ax2[i].relim()
            ax2[i].autoscale_view()

        # ax2[1].set_ylim([-1, 1])
        # ax2[2].set_ylim([-0.1, 0.1])
        # ax2[3].set_ylim([7, 17])
        # ax2[4].set_ylim([-1, 1])
        # ax2[5].set_ylim([-2, 2])
        # ax2[6].set_ylim([-1, 1])
        # ax2[7].set_ylim([-15, 15])
        fig.canvas.draw_idle()
        fig2.canvas.draw_idle()

    iter_slider.on_changed(update)

    plt.interactive(False)
    plt.show()
    return iter_slider


def plot_track_two_track(x_hist, u_hist, track_data, model):
    s = x_hist[0, :, :].T
    w = x_hist[1, :, :].T
    mu = x_hist[2, :, :].T
    v_x = x_hist[3, :, :].T
    v_y = x_hist[4, :, :].T
    r = x_hist[5, :, :].T
    omega_fl = x_hist[6, :, :].T
    omega_fr = x_hist[7, :, :].T
    omega_rl = x_hist[8, :, :].T
    omega_rr = x_hist[9, :, :].T
    delta = np.rad2deg(x_hist[10, :, :].T)
    D_fl = u_hist[0, :, :].T
    D_fr = u_hist[1, :, :].T
    D_rl = u_hist[2, :, :].T
    D_rr = u_hist[3, :, :].T


    # Calculate Cartesian Position
    x = np.ndarray([x_hist.shape[2], x_hist.shape[1]])
    y = np.ndarray([x_hist.shape[2], x_hist.shape[1]])

    for i in range(x_hist.shape[2]):
        for j in range(x_hist.shape[1]):
            x[i, j], y[i, j], _ = para2global(s[i, j], w[i, j], mu[i, j], track_data)

    # Create Plot
    fig, ax = plt.subplots()
    plt.subplots_adjust(bottom=0.25)
    plt.ylabel('y[m]')
    plt.xlabel('x[m]')

    # Plot Position Heatmap with velocity
    position = ax.scatter(x[:, 0], y[:, 0], c=v_x[:, 0], cmap=cm.rainbow, edgecolor='none', marker='o')
    color_bar = plt.colorbar(position, fraction=0.1)
    color_bar.set_label("Velocity in m/s")

    # Plot Track Referenceline
    plt.plot(track_data[:, 1], track_data[:, 2], ':', color='k')

    # Plot Boarder
    boarder_left_x = track_data[:, 1] - track_data[:, 5] * np.sin((track_data[:, 3] + np.pi / 2))
    boarder_left_y = track_data[:, 2] + track_data[:, 5] * np.cos((track_data[:, 3] + np.pi / 2))
    boarder_right_x = track_data[:, 1] + track_data[:, 6] * np.sin((track_data[:, 3] + np.pi / 2))
    boarder_right_y = track_data[:, 2] - track_data[:, 6] * np.cos((track_data[:, 3] + np.pi / 2))

    plt.plot(boarder_left_x, boarder_left_y, color='k', linewidth=1)
    plt.plot(boarder_right_x, boarder_right_y, color='k', linewidth=1)

    # Plot Vehicle Prediction
    position_prediction, = plt.plot(x[-1, :], y[-1, :], '-', color='r')

    # Shape Box
    ax.axis('equal')

    # Create Iteration Slider
    fig4, ax4 = plt.subplots(1, 1, figsize=(5, 5))
    iter_axis = plt.axes([0.25, 0.1, 0.65, 0.03], facecolor='lightgoldenrodyellow')
    iter_slider = Slider(iter_axis, 'Iteration', 1, s.shape[0] - 1, valinit=s.shape[0])

    # Create Second Figure
    fig2, ax2 = plt.subplots(8, 1, figsize=(17, 13))

    ax2[0].grid(True)
    s_plot, = ax2[0].plot(s[-1, :])
    ax2[0].set_title('Progress s', fontsize=10)
    ax2[0].set_ylabel('m')

    ax2[1].grid(True)
    w_plot, = ax2[1].plot(w[-1, :])
    ax2[1].set_title('Deviation w', fontsize=10)
    ax2[1].set_ylabel('m')

    ax2[2].grid(True)
    mu_plot, = ax2[2].plot(mu[-1, :])
    ax2[2].set_title('Reference Angle Deviaten mu', fontsize=10)
    ax2[2].set_ylabel('rad')

    ax2[3].grid(True)
    v_x_plot, = ax2[3].plot(v_x[-1, :])
    ax2[3].set_title('Velocity v_x', fontsize=10)
    ax2[3].set_ylabel('m/s')

    ax2[4].grid(True)
    v_y_plot, = ax2[4].plot(v_y[-1, :])
    ax2[4].set_title('Velocity v_y', fontsize=10)
    ax2[4].set_ylabel('m/s')

    ax2[5].grid(True)
    r_plot, = ax2[5].plot(r[-1, :])
    ax2[5].set_title('Yaw Rate r', fontsize=10)
    ax2[5].set_ylabel('rad/s')

    ax2[6].grid(True)
    omega_fl_plot, = ax2[6].plot(omega_fl[-1, :])
    omega_fr_plot, = ax2[6].plot(omega_fr[-1, :])
    omega_rl_plot, = ax2[6].plot(omega_rl[-1, :])
    omega_rr_plot, = ax2[6].plot(omega_rr[-1, :])
    ax2[6].set_title('Wheelspeed omega', fontsize=10)
    ax2[6].set_ylabel('1/s')

    ax2[7].grid(True)
    delta_plot, = ax2[7].plot(delta[-1, :])
    ax2[7].set_title('Steering Angle delta', fontsize=10)
    ax2[7].set_ylabel('deg')

    for i in range(7):
        plt.setp(ax2[i].get_xticklabels(), visible=False)

    plt.tight_layout()

    # Create Third Figure
    fig3, ax3 = plt.subplots(4, 1, figsize=(17, 3))

    r_plot2, = ax3[0].plot(r[-1, :])
    ax3[0].set_title('Yaw Rate', fontsize=10)
    ax3[0].set_ylabel('rad/s')

    delta_plot2, = ax3[1].plot(delta[-1, :])
    ax3[1].set_title('Steering Angle', fontsize=10)
    ax3[1].set_ylabel('deg')

    omega_fl_plot2, = ax3[2].plot(omega_fl[-1, :])
    omega_fr_plot2, = ax3[2].plot(omega_fr[-1, :])
    omega_rl_plot2, = ax3[2].plot(omega_rl[-1, :])
    omega_rr_plot2, = ax3[2].plot(omega_rr[-1, :])
    ax3[2].set_title('Wheel Speed', fontsize=10)
    ax3[2].set_ylabel('1/s')
    ax3[2].legend(['Front Left', 'Front Right', 'Rear Left', 'Rear Right'], loc=1)

    D_fl_plot, = ax3[3].plot(D_fl[-1, :])
    D_fr_plot, = ax3[3].plot(D_fr[-1, :])
    D_rl_plot, = ax3[3].plot(D_rl[-1, :])
    D_rr_plot, = ax3[3].plot(D_rr[-1, :])
    ax3[3].set_title('Torque Request', fontsize=10)
    ax3[3].set_ylabel('Nm')
    ax3[3].legend(['Front Left', 'Front Right', 'Rear Left', 'Rear Right'], loc=1)

    ax3[0].grid(True)
    ax3[1].grid(True)
    ax3[2].grid(True)
    ax3[3].grid(True)
    plt.xlabel('timesteps')


    def update(val):
        iter_value = int(iter_slider.val)
        iter_position = np.vstack((x[0:iter_value, 0], y[0:iter_value, 0]))
        position.set_offsets(iter_position.T)

        iter_prediction = np.vstack((x[iter_value, :], y[iter_value, :]))
        position_prediction.set_data(iter_prediction)

        s_plot.set_ydata(s[iter_value, :])
        w_plot.set_ydata(w[iter_value, :])
        mu_plot.set_ydata(mu[iter_value, :])
        v_x_plot.set_ydata(v_x[iter_value, :])
        v_y_plot.set_ydata(v_y[iter_value, :])
        r_plot.set_ydata(r[iter_value, :])
        omega_fl_plot.set_ydata(omega_fl[iter_value, :])
        omega_fr_plot.set_ydata(omega_fr[iter_value, :])
        omega_rl_plot.set_ydata(omega_rl[iter_value, :])
        omega_rr_plot.set_ydata(omega_rr[iter_value, :])
        delta_plot.set_ydata(delta[iter_value, :])
        D_fl_plot.set_ydata(D_fl[iter_value, :])
        D_fr_plot.set_ydata(D_fr[iter_value, :])
        D_rl_plot.set_ydata(D_rl[iter_value, :])
        D_rr_plot.set_ydata(D_rr[iter_value, :])
        omega_fl_plot2.set_ydata(omega_fl[iter_value, :])
        omega_fr_plot2.set_ydata(omega_fr[iter_value, :])
        omega_rl_plot2.set_ydata(omega_rl[iter_value, :])
        omega_rr_plot2.set_ydata(omega_rr[iter_value, :])
        r_plot2.set_ydata(r[iter_value, :])
        delta_plot2.set_ydata(delta[iter_value, :])

        for i in range(8):
            ax2[i].relim()
            ax2[i].autoscale_view()

        ax3[0].relim()
        ax3[0].autoscale_view()
        ax3[1].relim()
        ax3[1].autoscale_view()
        ax3[2].relim()
        ax3[2].autoscale_view()
        ax3[3].relim()
        ax3[3].autoscale_view()

        # ax2[1].set_ylim([-1, 1])
        # ax2[2].set_ylim([-0.1, 0.1])
        # ax2[3].set_ylim([7, 17])
        # ax2[4].set_ylim([-1, 1])
        # ax2[5].set_ylim([-2, 2])
        # ax2[7].set_ylim([-15, 15])
        fig.canvas.draw_idle()
        fig2.canvas.draw_idle()
        fig3.canvas.draw_idle()

    iter_slider.on_changed(update)

    plt.interactive(False)
    plt.show()
    return iter_slider


def plot_track_realtime(x_hist, track_data, boarder_distance):
    s = x_hist[0, :, :].T
    w = x_hist[1, :, :].T
    mu = x_hist[2, :, :].T
    v_x = x_hist[3, 0, :].T

    # Calculate Cartesian Position
    x = np.ndarray([x_hist.shape[2], x_hist.shape[1]])
    y = np.ndarray([x_hist.shape[2], x_hist.shape[1]])

    for i in range(x_hist.shape[2]):
        for j in range(x_hist.shape[1]):
            x[i, j], y[i, j], _ = para2global(s[i, j], w[i, j], mu[i, j], track_data)

    # Create Plot
    fig, ax = plt.subplots(1, 1)
    plt.subplots_adjust(bottom=0.25)
    plt.ylabel('y[m]')
    plt.xlabel('x[m]')

    # Plot Position Heatmap with velocity
    position = ax.scatter(x[:, 0], y[:, 0], c=v_x, cmap=cm.rainbow, edgecolor='none', marker='o')
    color_bar = plt.colorbar(position, fraction=0.1)
    color_bar.set_label("Velocity in m/s")

    # Plot Track Referenceline
    plt.plot(track_data[:, 1], track_data[:, 2], ':', color='k')

    # Plot Vehicle Prediction
    position_prediction, = plt.plot([], '-', color='r')

    # Shape Box
    ax.axis('equal')

    # Create Animation
    def init():
        position.set_offsets([])
        plt.plot([], ':', color='k')
        return position

    def animate(i):
        iter_position = np.vstack((x[0:i, 0], y[0:i, 0]))
        position.set_offsets(iter_position.T)

        iter_prediction = np.vstack((x[i, :], y[i, :]))
        position_prediction.set_data(iter_prediction)
        return position

    anim = FuncAnimation(fig, animate, init_func=init, frames=s.shape[0] - 1, interval=50, blit=False, repeat=False)

    plt.interactive(False)
    plt.show()
    return anim


def para2global(s, w, mu, track_data):
    s_i = np.argmin(abs(track_data[:, 0] - s))

    x_glob = track_data[s_i, 1] - w * np.sin(track_data[s_i, 3] + np.pi/2)
    y_glob = track_data[s_i, 2] + w * np.cos(track_data[s_i, 3] + np.pi/2)
    phi_glob = track_data[s_i, 3] + mu
    return x_glob, y_glob, phi_glob
