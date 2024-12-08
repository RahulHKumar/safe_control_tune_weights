import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Polygon
import casadi as ca
import sys
sys.path.append('.')  # Ensure local imports work

from robots.dynamic_unicycle2D import DynamicUnicycle2D, angle_normalize

# Simulation parameters
dt = 0.05  # Time step
robot_spec = {
    "model": "DynamicUnicycle2D",
    "v_max": 1.0,  # Maximum velocity
    "w_max": 0.5,  # Maximum angular velocity
    "a_max": 0.5,  # Maximum acceleration
    "radius": 0.2,  # Robot radius for collision detection
    "fov_angle": 60,  # Field of view angle
    "cam_range": 1.0  # Camera range
}

# Initial robot state [x, y, theta, v]
# Note the change in initial state format to match dynamic unicycle model
X = np.array([[1.0], [7.5], [0.0], [0.0]]).reshape(-1, 1)

# Create dynamic unicycle model
model = DynamicUnicycle2D(dt, robot_spec)

# Define the obstacle regions (rest remains the same as before)
obstacle_regions = [
    {'start': 0, 'end': 5, 'num_obstacles': 0, 'obstacle_radius_range': (0, 0)},
    {'start': 5, 'end': 15, 'num_obstacles': 4, 'obstacle_radius_range': (0.2, 1.5)},
    {'start': 15, 'end': 25, 'num_obstacles': 10, 'obstacle_radius_range': (0.2, 1.0)},
    {'start': 25, 'end': 40, 'num_obstacles': 10, 'obstacle_radius_range': (0.2, 1.0), 'dynamic': True, 'velocity_range': (-0.2, 0.2)}
]

# Initialize the obstacles (same as before)
obstacles = []
for region in obstacle_regions:
    for _ in range(region['num_obstacles']):
        x = np.random.uniform(region['start'], region['end'])
        y = np.random.uniform(0, 15)
        radius = np.random.uniform(*region['obstacle_radius_range'])
        if region.get('dynamic', False):
            velocity_x = np.random.uniform(*region['velocity_range'])
            velocity_y = np.random.uniform(*region['velocity_range'])
        else:
            velocity_x, velocity_y = 0, 0
        obstacles.append({'x': x, 'y': y, 'radius': radius, 'dynamic': region.get('dynamic', False), 'velocity_x': velocity_x, 'velocity_y': velocity_y})

# Set up the plot
fig, ax = plt.subplots(figsize=(10, 5))
ax.set_xlim(0, 40)
ax.set_ylim(0, 15)
ax.set_aspect('equal')
ax.set_title('Navigation Simulation - Dynamic Unicycle Model')

# Plotted dotted lines to separate the regions
for region in obstacle_regions:
    ax.plot([region['start'], region['start']], [0, 15], 'k--')
    ax.plot([region['end'], region['end']], [0, 15], 'k--')

# Plot the robot
robot_plot = ax.add_artist(plt.Circle((X[0, 0], X[1, 0]), robot_spec['radius'], fill=False, color='red'))

# Plot the obstacles
obstacle_plots = []
for obstacle in obstacles:
    obstacle_plots.append(ax.add_artist(plt.Circle((obstacle['x'], obstacle['y']), obstacle['radius'], fill=True, linewidth=1, edgecolor='k', facecolor='grey')))

# Field of view (FOV) polygon
# Adjusted to use the robot's current yaw
fov_angle = np.deg2rad(robot_spec['fov_angle'])
fov_range = robot_spec['cam_range']
fov_polygon = Polygon([(X[0, 0], X[1, 0]), 
                       (X[0, 0] + fov_range * np.cos(X[2, 0] + fov_angle / 2), X[1, 0] + fov_range * np.sin(X[2, 0] + fov_angle / 2)),
                       (X[0, 0] + fov_range * np.cos(X[2, 0] - fov_angle / 2), X[1, 0] + fov_range * np.sin(X[2, 0] - fov_angle / 2))],
                      alpha=0.2, color='yellow')
ax.add_artist(fov_polygon)

# Control variables
collision_detected = False

# User control inputs
# Modified to represent acceleration and angular velocity more precisely
user_accel = 0.0
user_omega = 0.0

def check_collision(X, obstacles, robot_radius):
    """Check if robot collides with any obstacle"""
    for obstacle in obstacles:
        distance = np.sqrt((X[0, 0] - obstacle['x'])**2 + (X[1, 0] - obstacle['y'])**2)
        if distance <= (robot_radius + obstacle['radius']):
            return True
    return False

# Key press and release event handlers
def on_key_press(event):
    global user_accel, user_omega

    # Adjust acceleration and angular velocity based on key presses
    # Constrain inputs to match the robot specification
    if event.key == 'up':
        user_accel = min(user_accel + 0.1, robot_spec['a_max'])  # Increase forward acceleration
    elif event.key == 'down':
        user_accel = max(user_accel - 0.1, -robot_spec['a_max'])  # Increase backward acceleration
    elif event.key == 'right':
        user_omega = max(user_omega - 0.1, -robot_spec['w_max'])  # Rotate counterclockwise
    elif event.key == 'left':
        user_omega = min(user_omega + 0.1, robot_spec['w_max'])  # Rotate clockwise
    elif event.key == 'r':
        # Reset acceleration and angular velocity
        user_accel = 0.0
        user_omega = 0.0

    print('user_accel: {:.4f}, user_omega: {:.4f}'.format(user_accel, user_omega))

def animate(frame):
    global X, collision_detected, user_accel, user_omega

    if collision_detected:
        # Stop the animation if collision is detected
        ani.event_source.stop()
        plt.title('Navigation Simulation - COLLISION DETECTED!')
        return robot_plot, *obstacle_plots, fov_polygon

    # Create control input based on user input
    # Ensure the input is a 2x1 numpy array as expected by the step function
    U = np.array([[user_accel], [user_omega]])

    # Update robot state using dynamic unicycle model
    # This line matches the implementation in robot.py
    X = model.step(X, U)

    # Check for collision
    if check_collision(X, obstacles, robot_spec['radius']):
        collision_detected = True
        print('Collision detected!')
        plt.title('Navigation Simulation - COLLISION DETECTED!')
        robot_plot.set_color('red')

    # Update robot position and orientation
    robot_plot.center = (X[0, 0], X[1, 0])

    # Update obstacles
    for i, obstacle in enumerate(obstacles):
        if obstacle['dynamic']:
            # Check and adjust the obstacle's position to stay within the boundary and reflect back
            obstacle['x'] += obstacle['velocity_x']
            obstacle['y'] += obstacle['velocity_y']

            # Reflect the obstacle back if it reaches the boundary
            if obstacle['x'] <= obstacle['radius'] + obstacle_regions[3]['start'] or obstacle['x'] >= obstacle_regions[3]['end'] - obstacle['radius']:
                obstacle['velocity_x'] *= -1
            if obstacle['y'] <= obstacle['radius'] or obstacle['y'] >= 15 - obstacle['radius']:
                obstacle['velocity_y'] *= -1

            obstacle_plots[i].center = (obstacle['x'], obstacle['y'])

    # Update the field of view polygon
    # Now using the updated robot yaw from the state vector
    fov_polygon.set_xy([(X[0, 0], X[1, 0]), 
                        (X[0, 0] + fov_range * np.cos(X[2, 0] + fov_angle / 2), X[1, 0] + fov_range * np.sin(X[2, 0] + fov_angle / 2)),
                        (X[0, 0] + fov_range * np.cos(X[2, 0] - fov_angle / 2), X[1, 0] + fov_range * np.sin(X[2, 0] - fov_angle / 2))])

    return robot_plot, *obstacle_plots, fov_polygon

# Create the animation
fig.canvas.mpl_connect('key_press_event', on_key_press)
ani = FuncAnimation(fig, animate, frames=400, interval=50, blit=True)

# # Plotting the value of user_accel and user_omega on the plot
# ax.text(0.95, 0.95, 'user_accel: {:.4f}'.format(user_accel),
#     horizontalalignment='right', verticalalignment='top', transform=ax.transAxes)
# ax.text(0.95, 0.90, 'user_omega: {:.4f}'.format(user_omega),
#     horizontalalignment='right', verticalalignment='top', transform=ax.transAxes)
plt.show()