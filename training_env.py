import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Polygon

# Set up the simulation parameters
robot_x, robot_y = 1, 7.5
robot_radius = 0.2
robot_linear_acc = 0.0
robot_angular_acc = 0.0
fov_angle = 60  # field of view angle in degrees
fov_range = 1.0  # field of view range in meters

# Define the obstacle regions
obstacle_regions = [
    {'start': 0, 'end': 5, 'num_obstacles': 0, 'obstacle_radius_range': (0, 0)},
    {'start': 5, 'end': 15, 'num_obstacles': 4, 'obstacle_radius_range': (0.2, 1.5)},
    {'start': 15, 'end': 25, 'num_obstacles': 10, 'obstacle_radius_range': (0.2, 1.0)},
    {'start': 25, 'end': 40, 'num_obstacles': 10, 'obstacle_radius_range': (0.2, 1.0), 'dynamic': True, 'velocity_range': (-0.2, 0.2)}
]

# Initialize the obstacles
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
ax.set_title('Navigation Simulation- Dataset collection Env')

# Plotted dotted lines to separate the regions
for region in obstacle_regions:
    ax.plot([region['start'], region['start']], [0, 15], 'k--')
    ax.plot([region['end'], region['end']], [0, 15], 'k--')

# Plot the robot
robot_plot = ax.add_artist(plt.Circle((robot_x, robot_y), robot_radius, fill=False, color='red'))

# Plot the obstacles
obstacle_plots = []
for obstacle in obstacles:
    obstacle_plots.append(ax.add_artist(plt.Circle((obstacle['x'], obstacle['y']), obstacle['radius'], fill=True, linewidth=1, edgecolor='k', facecolor='grey')))

# Field of view (FOV) polygon
fov_polygon = Polygon([(robot_x, robot_y), 
                      (robot_x + fov_range * np.cos(np.radians(fov_angle / 2)), robot_y + fov_range * np.sin(np.radians(fov_angle / 2))),
                      (robot_x + fov_range * np.cos(np.radians(-fov_angle / 2)), robot_y + fov_range * np.sin(np.radians(-fov_angle / 2)))],
                     alpha=0.2, color='yellow')
ax.add_artist(fov_polygon)

def fov_feedback(obstacles):
    """
    Returns the centroid and radius of observable obstacles in the robot's field of view.
    """
    observable_obstacles = []
    for obstacle in obstacles:
        # Check if the obstacle is within the robot's field of view
        if ((obstacle['x'] - robot_x) ** 2 + (obstacle['y'] - robot_y) ** 2) ** 0.5 <= fov_range and \
           np.arctan2(obstacle['y'] - robot_y, obstacle['x'] - robot_x) * 180 / np.pi >= -fov_angle / 2 and \
           np.arctan2(obstacle['y'] - robot_y, obstacle['x'] - robot_x) * 180 / np.pi <= fov_angle / 2:
            observable_obstacles.append(obstacle)

    if observable_obstacles:
        # Calculate the centroid and radius of the observable obstacles
        centroid_x = np.mean([obs['x'] for obs in observable_obstacles])
        centroid_y = np.mean([obs['y'] for obs in observable_obstacles])
        radius = max([obs['radius'] for obs in observable_obstacles])
        return (centroid_x, centroid_y), radius
    else:
        return None, None

# Animation function
def animate(frame):
    global robot_x, robot_y, robot_linear_acc, robot_angular_acc

    # Update the robot's position based on the control inputs
    robot_x += robot_linear_acc * np.cos(np.radians(robot_angular_acc))
    robot_y += robot_linear_acc * np.sin(np.radians(robot_angular_acc))
    robot_plot.center = (robot_x, robot_y)

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
        obstacle_plots[i].set_radius(obstacle['radius'])

    # Update the field of view polygon
    fov_polygon.set_xy([(robot_x, robot_y), 
                        (robot_x + fov_range * np.cos(np.radians(fov_angle / 2)), robot_y + fov_range * np.sin(np.radians(fov_angle / 2))),
                        (robot_x + fov_range * np.cos(np.radians(-fov_angle / 2)), robot_y + fov_range * np.sin(np.radians(-fov_angle / 2)))])

    # Get the centroid and radius of observable obstacles
    centroid, radius = fov_feedback(obstacles)
    if centroid is not None and radius is not None:
        print(f"Centroid: ({centroid[0]:.2f}, {centroid[1]:.2f}), Radius: {radius:.2f}")

    return robot_plot, *obstacle_plots, fov_polygon

# Key press event handler
def on_key_press(event):
    global robot_linear_acc, robot_angular_acc

    if event.key == 'w':
        robot_linear_acc = min(robot_linear_acc + 0.05, 0.5)
    elif event.key == 's':
        robot_linear_acc = max(robot_linear_acc - 0.05, -0.5)
    elif event.key == 'a':
        robot_angular_acc = max(robot_angular_acc - 0.05, -0.5)
    elif event.key == 'd':
        robot_angular_acc = min(robot_angular_acc + 0.05, 0.5)

# Create the animation
fig.canvas.mpl_connect('key_press_event', on_key_press)
ani = FuncAnimation(fig, animate, frames=200, interval=50, blit=True)

plt.show()