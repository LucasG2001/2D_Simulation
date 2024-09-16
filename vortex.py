import numpy as np
import matplotlib.pyplot as plt

# Parameters
goal = np.array([0.5, 0.5])  # Goal position
vorticity_center = np.array([0.0, 0.0])  # Center of vorticity field
vorticity_strength = 200.0  # Strength of vorticity field

# Initial conditions
position = np.array([-0.5, -0.5])  # Initial position
velocity = np.array([0.0, 0.0])  # Initial velocity

# Function to calculate the attraction force at a given position
def calculate_attraction_force(position, velocity):
    K = 20
    D = 10
    return -K * (position - goal) - D * velocity

# Function to calculate the vorticity force at a given position
def calculate_vorticity_force(position):
    x = position[0]
    y = position[1]
    vorticity_force = 1/np.linalg.norm(position) * np.array((position[1], -position[0]))
    if x**2 + y**2 < 0.2**2:
        return vorticity_force * vorticity_strength
    else:
        return 0.0 * position

# Function to plot the vorticity field
def plot_vorticity_field():
    x = np.linspace(-3, 3, 10)
    y = np.linspace(-3, 3, 10)
    X, Y = np.meshgrid(x, y)
    U = -np.sin(np.arctan2(Y, X)) * np.sqrt(np.square(X) + np.square(Y)) * 0.1
    V = np.cos(np.arctan2(Y, X)) * np.sqrt(np.square(X) + np.square(Y)) * 0.1
    plt.figure(figsize=(8, 6))
    plt.quiver(X, Y, U, V, color='blue', angles='xy', scale_units='xy', scale=1)
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('Vorticity Field')
    plt.grid(True)
    plt.axis('equal')
    plt.show()

# Simulation parameters
dt = 0.01  # Time step
num_steps = 10*100
  # Number of simulation steps

# Arrays to store trajectory
x_traj = [position[0]]
y_traj = [position[1]]

# Simulation loop
for _ in range(num_steps):
    # Update position and velocity using Euler integration
    attraction_force = calculate_attraction_force(position, velocity)
    vorticity_force = calculate_vorticity_force(position)
    total_force = attraction_force + vorticity_force
    acceleration = total_force
    velocity += acceleration * dt
    position += velocity * dt
    
    # Store position for plotting
    x_traj.append(position[0])
    y_traj.append(position[1])

# Plot trajectory
plt.figure(figsize=(8, 6))
plt.plot(x_traj, y_traj, label='Trajectory')
plt.scatter(goal[0], goal[1], color='red', label='Goal')
plt.xlabel('X')
plt.ylabel('Y')
plt.title('Trajectory with Vorticity Field')
plt.legend()
plt.grid(True)
plt.axis('equal')
# Plot circle with radius 0.2 around the origin
circle = plt.Circle((0, 0), 0.2, color='r', fill=False, linestyle='--')
plt.gca().add_patch(circle)
plt.show()

# Plot vorticity field
plot_vorticity_field()
