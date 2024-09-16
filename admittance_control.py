import numpy as np
import matplotlib.pyplot as plt

# Simulation parameters
T = 0.001  # Time step
total_time = 10  # Total simulation time
steps = int(total_time / T)  # Number of steps

# System parameters
m = 1.0  # Mass of the end-effector
Kp = 10.0  # Proportional gain for the controller
theta = 1.0  # Some parameter for the desired velocity calculation
b = 0.5  # Damping coefficient

# Sinusoidal force parameters
amplitude = 1.0
frequency = 250.0  # Frequency in Hz


# Initialize arrays to store the state of the system
F_ext = np.zeros((2, steps))  # External force
F_u = np.zeros((2, steps))  # Control input force
v = np.zeros((2, steps))  # Velocity of the end-effector
x = np.zeros((2, steps))  # Position of the end-effector
x_dot_dot = np.zeros((2, steps))  # Acceleration of the end-effector
v_desired = np.zeros((2, steps))  # Desired velocity

# Add a sinusoidal external force
time = np.arange(steps) * T
F_ext[0, :] = amplitude * np.sin(2 * np.pi * frequency * time)
F_ext[1, :] = amplitude * np.sin(2 * np.pi * frequency * time)
# # Add a constant external force from 2 to 3 seconds (commented out)
# start_time = int(2 / T)
# end_time = int(3 / T)
# F_ext[:, start_time:end_time] = np.array([[1.0], [1.0]])  # Example constant force in both x and y directions

# Simulation loop
for k in range(1, steps):
    # Compute the desired velocity
    v_desired[:, k] = F_ext[:, k] * T / (2 * theta + b * T) + F_ext[:, k-1] * T / (2 * theta + b * T) + v[:, k-1]
    
    # Compute the control force
    F_u[:, k] = Kp * (v_desired[:, k] - v[:, k-1])
    
    # Compute the acceleration
    x_dot_dot[:, k] = (F_ext[:, k] + F_u[:, k]) / m
    
    # Update the velocity
    v[:, k] = x_dot_dot[:, k] * T + v[:, k-1]
    
    # Update the position
    x[:, k] = v[:, k] * T + x[:, k-1]

# Plot the trajectory of the end-effector
plt.figure(figsize=(18, 12))

plt.subplot(2, 2, 1)
plt.plot(x[0, :], x[1, :], label="Trajectory of the end-effector")
plt.xlabel("X Position (m)")
plt.ylabel("Y Position (m)")
plt.title("Trajectory of the End-Effector")
plt.legend()
plt.grid(True)

# Plot the velocity of the end-effector
plt.subplot(2, 2, 2)
plt.plot(time, v[0, :], label="X Velocity")
plt.plot(time, v[1, :], label="Y Velocity")
plt.xlabel("Time (s)")
plt.ylabel("Velocity (m/s)")
plt.title("Velocity of the End-Effector")
plt.legend()
plt.grid(True)

# Plot the acceleration of the end-effector
plt.subplot(2, 2, 3)
plt.plot(time, x_dot_dot[0, :], label="X Acceleration")
plt.plot(time, x_dot_dot[1, :], label="Y Acceleration")
plt.xlabel("Time (s)")
plt.ylabel("Acceleration (m/s^2)")
plt.title("Acceleration of the End-Effector")
plt.legend()
plt.grid(True)

# Plot the desired velocity of the end-effector
plt.subplot(2, 2, 4)
plt.plot(time, v_desired[0, :], label="Desired X Velocity")
plt.plot(time, v_desired[1, :], label="Desired Y Velocity")
plt.xlabel("Time (s)")
plt.ylabel("Desired Velocity (m/s)")
plt.title("Desired Velocity of the End-Effector")
plt.legend()
plt.grid(True)

plt.tight_layout()
plt.show()
