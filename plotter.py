import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, ImageMagickWriter, PillowWriter
import do_mpc
import config
import seaborn as sns
import numpy as np

class Plotter:
    def __init__(self, controller):
        self.controller = controller
        self.mpc = controller.mpc

    def plot_results(self):
        fig, ax, graphics = do_mpc.graphics.default_plot(self.mpc.data, figsize=(9, 8))
        graphics.plot_results()
        ax[0].legend(['x-position', 'y-position', 'theta', 'x-velocity', 'y-velocity', 'omega'], title='states')
        ax[1].legend(['F_x', 'F_y', 'tau'], title='inputs')
        ax[2].set_xlabel('Time [s]')
        ax[0].set_ylabel('State')
        ax[1].set_ylabel('Input')
        ax[2].set_ylabel('Cost')
        fig.suptitle('Trajectories')

        colors = sns.color_palette()
        ax[1].hlines(y=config.u_max[0], xmin=0, xmax=len(self.mpc.data['_time'])*config.Ts, linewidth=1, color=colors[0], linestyle='--', label='F_x limit')
        ax[1].hlines(y=config.u_max[1], xmin=0, xmax=len(self.mpc.data['_time'])*config.Ts, linewidth=1, color=colors[1], linestyle='--', label='F_y limit')
        ax[1].hlines(y=config.u_max[2], xmin=0, xmax=len(self.mpc.data['_time'])*config.Ts, linewidth=1, color=colors[2], linestyle='--', label='tau limit')
        ax[1].hlines(y=-config.u_max[0], xmin=0, xmax=len(self.mpc.data['_time'])*config.Ts, linewidth=1, color=colors[0], linestyle='--')
        ax[1].hlines(y=-config.u_max[1], xmin=0, xmax=len(self.mpc.data['_time'])*config.Ts, linewidth=1, color=colors[1], linestyle='--')
        ax[1].hlines(y=-config.u_max[2], xmin=0, xmax=len(self.mpc.data['_time'])*config.Ts, linewidth=1, color=colors[2], linestyle='--')

        plt.savefig('images/plot_results.png')
        plt.show()

    def create_path_animation(self):
        global ax
        sns.set_theme()
        fig, ax = plt.subplots(figsize=(9, 5))
        ax.set_xlabel('x [m]')
        ax.set_ylabel('y [m]')
        plt.title("Robot path")
        ax.set_xlim([-2.5, 2.5])
        ax.set_ylim([-2.5, 2.5])
        # plt.tight_layout()
        ax.set_aspect('equal')

        # Plot initial position
        ax.plot(config.x0[0], config.x0[1], 'r.', label="Initial position")
        
        globals()['var_hold_pos'] = plt.Circle((0,0), config.rad_hold, color='g', linestyle='--')
        ax.add_patch(globals()['var_hold_pos'])

        if config.obs_on:
            for x_obs, y_obs, r_obs in config.obs:
                ax.add_patch(plt.Circle((x_obs, y_obs), r_obs, color='k', label="Obstacle"))
        # Plot moving goal and robot trajectory
        robot_position = ax.add_patch(plt.Circle((self.mpc.data['_x'][0, 0], self.mpc.data['_x'][0, 1]), config.rad_chaser, color='b'))
        # Draw rotating target
        ax.add_patch(plt.Circle((0,0), config.rad_target, color='g'))
        robot_path, = ax.plot(self.mpc.data['_x'][0, 0], self.mpc.data['_x'][0, 1], label="Robot path")
        goal, = ax.plot(self.mpc.data['_tvp', 'x_target'][0], self.mpc.data['_tvp', 'y_target'][0], marker="x", color='g', markersize=10, label="Varying Hold Position", zorder=0)
        globals()['robot_heading'] = plt.Arrow(x=config.x0[0], y=config.x0[1], dx=np.cos(config.x0[2]), dy=np.sin(config.x0[2]), color='k', width=0.1, linewidth=0.6)
        ax.add_patch(globals()['robot_heading'])
        globals()['target_heading'] = plt.Arrow(x=0, y=0, dx=0, dy=0, color='k', width=0.1, linewidth=0.6)
        ax.add_patch(globals()['target_heading'])

        animation = FuncAnimation(fig, self.update, frames=len(self.mpc.data['_x'][:, 0]), interval=config.Ts*1000, repeat=False, fargs=(goal, robot_path, robot_position))

        # Only show unique legends
        handles, labels = plt.gca().get_legend_handles_labels()
        plt.legend(handles, labels)

        animation.save('images/path_animation.gif', writer=PillowWriter(fps=100))
        plt.show()
    def update(self, frame, goal, robot_path, robot_position):
        goal.set_data(self.mpc.data['_tvp', 'x_target'][frame], self.mpc.data['_tvp', 'y_target'][frame])
        robot_path.set_data(self.mpc.data['_x'][:frame+1, 0], self.mpc.data['_x'][:frame+1, 1])
        robot_position.set_center((self.mpc.data['_x'][frame, 0], self.mpc.data['_x'][frame, 1]))
        ax.patches.remove(globals()['robot_heading'])
        globals()['robot_heading'] = plt.Arrow(x=self.mpc.data['_x'][frame, 0],
                                               y=self.mpc.data['_x'][frame, 1],
                                               dx=np.cos(self.mpc.data['_x'][frame, 2])/8,
                                               dy=np.sin(self.mpc.data['_x'][frame, 2])/8,
                                               color='k',
                                               width=0.1,
                                               linewidth=0.6)
        ax.add_patch(globals()['robot_heading'])
        ax.patches.remove(globals()['var_hold_pos'])
        globals()['var_hold_pos'] = plt.Circle((0,0),
                                (self.mpc.data['_tvp', 'x_target'][frame]**2 + self.mpc.data['_tvp', 'y_target'][frame]**2)**0.5,
                                linestyle='--', edgecolor='g', fill=False)
        ax.add_patch(globals()['var_hold_pos'])
        ax.patches.remove(globals()['target_heading'])
        globals()['target_heading'] = plt.Arrow(x=0,
                                                 y=0,
                                                 dx=config.rad_target*self.mpc.data['_tvp', 'x_target'][frame] / np.sqrt(self.mpc.data['_tvp', 'x_target'][frame]**2 + self.mpc.data['_tvp', 'y_target'][frame]**2),
                                                 dy=config.rad_target*self.mpc.data['_tvp', 'y_target'][frame] / np.sqrt(self.mpc.data['_tvp', 'x_target'][frame]**2 + self.mpc.data['_tvp', 'y_target'][frame]**2),
                                                 color='k',
                                                 width=0.1,
                                                 linewidth=0.6)
        ax.add_patch(globals()['target_heading'])
        return goal, robot_path