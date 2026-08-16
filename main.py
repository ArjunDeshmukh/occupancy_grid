import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, RadioButtons
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Rectangle


class VehicleGridVisualizer:
    def __init__(self):
        # Vehicle parameters
        self.L = 4.0  # Vehicle length
        self.W = 2.0  # Vehicle width

        # Initial State [x, y, heading]
        self.state = np.array([50.0, 50.0, 0.0])

        # Grid parameters
        self.grid_size = 100
        self.resolution = 0.2  # 1 meter per cell

        # Trajectory prediction parameters
        self.pred_time = 6.0
        self.pred_steps = 60
        self.dt = 0.1  # Simulation time step

        self.setup_plot()

    def setup_plot(self):
        # Setup Figure and Subplots
        self.fig = plt.figure(figsize=(10, 10))
        self.ax = self.fig.add_axes([0.05, 0.4, 0.9, 0.55])

        # Setup GUI Control Axes
        ax_color = 'lightgoldenrodyellow'
        self.ax_v = self.fig.add_axes([0.15, 0.30, 0.65, 0.03], facecolor=ax_color)
        self.ax_steer = self.fig.add_axes([0.15, 0.25, 0.65, 0.03], facecolor=ax_color)
        self.ax_decay_s = self.fig.add_axes([0.15, 0.20, 0.65, 0.03], facecolor=ax_color)
        self.ax_decay_lat = self.fig.add_axes([0.15, 0.15, 0.65, 0.03], facecolor=ax_color)
        self.ax_thresh = self.fig.add_axes([0.15, 0.10, 0.65, 0.03], facecolor=ax_color)
        self.ax_radio = self.fig.add_axes([0.85, 0.10, 0.12, 0.20], facecolor=ax_color)

        # Sliders
        self.sl_v = Slider(self.ax_v, 'Speed', -5.0, 15.0, valinit=5.0)
        self.sl_steer = Slider(self.ax_steer, 'Steering', -0.5, 0.5, valinit=0.1)
        self.sl_decay_s = Slider(self.ax_decay_s, 'Long. Decay', 0.01, 0.5, valinit=0.08)
        self.sl_decay_lat = Slider(self.ax_decay_lat, 'Lat. Decay', 0.01, 1.0, valinit=0.3)
        self.sl_thresh = Slider(self.ax_thresh, 'Threshold', 0.01, 0.99, valinit=0.6)

        # Radio Buttons for Grid Mode
        self.radio = RadioButtons(self.ax_radio, ('World Fixed', 'Host Translating'))

        # Animation hook
        self.anim = FuncAnimation(self.fig, self.update, interval=50, blit=False, cache_frame_data=False)

    def predict_trajectory(self, state, v, steer):
        """ Predicts the path ahead based on current speed and steering. """
        x, y, theta = state
        traj_x, traj_y = [x], [y]

        for _ in range(self.pred_steps):
            theta += (v / self.L) * np.tan(steer) * (self.pred_time / self.pred_steps)
            x += v * np.cos(theta) * (self.pred_time / self.pred_steps)
            y += v * np.sin(theta) * (self.pred_time / self.pred_steps)
            traj_x.append(x)
            traj_y.append(y)

        return np.array(traj_x), np.array(traj_y)

    def compute_reachability(self, X, Y, traj_x, traj_y, v):
        """ Computes the heat map for the grid. """
        # Flatten grid for vectorized distance calculation
        x_flat = X.flatten()
        y_flat = Y.flatten()

        # Compute distances from every grid cell to every point in the trajectory
        dx = x_flat[:, np.newaxis] - traj_x[np.newaxis, :]
        dy = y_flat[:, np.newaxis] - traj_y[np.newaxis, :]
        dist_sq = dx ** 2 + dy ** 2

        # Find closest trajectory point for each grid cell
        min_idx = np.argmin(dist_sq, axis=1)
        min_dist = np.sqrt(np.take_along_axis(dist_sq, min_idx[:, np.newaxis], axis=1).flatten())

        # Compute longitudinal distance (s) along the path
        # Multiply by approx step distance
        step_dist = v * (self.pred_time / self.pred_steps) if v != 0 else 0
        s_dist = min_idx * step_dist

        # Compute effective lateral distance (ignoring distance inside vehicle width)
        lat_eff = np.maximum(0, min_dist - (self.W / 2))

        # Compute heat with Exponential decay
        k_s = self.sl_decay_s.val
        k_lat = self.sl_decay_lat.val

        heat_flat = np.exp(-k_s * s_dist) * np.exp(-k_lat * lat_eff ** 2)

        # Hard cut-off if behind the vehicle or beyond prediction horizon
        heat_flat[s_dist > (self.pred_time * abs(v))] = 0.0

        return heat_flat.reshape(X.shape)

    def update(self, frame):
        v = self.sl_v.val
        steer = self.sl_steer.val
        thresh = self.sl_thresh.val
        mode = self.radio.value_selected

        # 1. Kinematic Update (update host vehicle position)
        self.state[2] += (v / self.L) * np.tan(steer) * self.dt
        self.state[0] += v * np.cos(self.state[2]) * self.dt
        self.state[1] += v * np.sin(self.state[2]) * self.dt

        # Wrap around for World Fixed mode to prevent driving off completely
        if mode == 'World Fixed':
            self.state[0] = self.state[0] % 100
            self.state[1] = self.state[1] % 100

        x, y, theta = self.state

        # 2. Determine Grid Extents based on Mode
        if mode == 'World Fixed':
            grid_x_min, grid_x_max = 0, 100
            grid_y_min, grid_y_max = 0, 100
        else:
            # Translating grid (no rotation)
            grid_x_min, grid_x_max = x - 30, x + 30
            grid_y_min, grid_y_max = y - 30, y + 30

        x_lin = np.linspace(grid_x_min, grid_x_max, self.grid_size)
        y_lin = np.linspace(grid_y_min, grid_y_max, self.grid_size)
        X, Y = np.meshgrid(x_lin, y_lin)

        # 3. Predict Trajectory & Compute Heatmap
        traj_x, traj_y = self.predict_trajectory(self.state, v, steer)
        R = self.compute_reachability(X, Y, traj_x, traj_y, v)

        # 4. Draw Everything
        self.ax.clear()
        self.ax.set_title("Vehicle Reachability Grid Prediction")

        # Plot Heatmap (Blue to Red)
        pcm = self.ax.pcolormesh(X, Y, R, cmap='jet', vmin=0.0, vmax=1.0, shading='auto')

        # Plot Threshold Boundary & highlight "Double Resolution" zone using hatching
        if R.min() < thresh < R.max():
            # Solid Yellow Boundary
            self.ax.contour(X, Y, R, levels=[thresh], colors='yellow', linewidths=2.5)
            # Finer mesh/hatch pattern inside the boundary to represent "doubled resolution"
            self.ax.contourf(X, Y, R, levels=[thresh, 1.0], colors='none', hatches=['xx'])

        # Plot Vehicle Body
        # Compute bottom-left corner from center
        bl_x = x - (self.L / 2) * np.cos(theta) + (self.W / 2) * np.sin(theta)
        bl_y = y - (self.L / 2) * np.sin(theta) - (self.W / 2) * np.cos(theta)

        rect = Rectangle((bl_x, bl_y), self.L, self.W, angle=np.degrees(theta),
                         edgecolor='white', facecolor='black', lw=2, zorder=5)
        self.ax.add_patch(rect)

        # Plot Trajectory Line
        self.ax.plot(traj_x, traj_y, color='white', linestyle='--', linewidth=2, zorder=4)

        # Set limits
        self.ax.set_xlim(grid_x_min, grid_x_max)
        self.ax.set_ylim(grid_y_min, grid_y_max)
        self.ax.set_aspect('equal')
        self.ax.grid(True, color='white', alpha=0.2)


if __name__ == '__main__':
    viz = VehicleGridVisualizer()
    plt.show()