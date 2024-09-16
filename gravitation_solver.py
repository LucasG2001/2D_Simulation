import numpy as np
import matplotlib.pyplot as plt

# Parameters
K = 20.0  # spring constant
M = 0.7  # mass
D = 9.0  # damping coefficient
P = 0.0  # force magnitude
R = 0.0  # safety bubble radius

# Initial conditions
a = -0.0
b = 0.0
x0 = 0.2
y0 = -0.3
x_dot0 = 0.0
y_dot0 = 4

# Time parameters
timestep = 0.001
total_time = 30.0

    

# Function to compute acceleration in x-direction
def acceleration_x(x, y, x_dot, y_dot):
    if x ** 2 + y ** 2 <= R ** 2:
        # print("true")
        return -K / M * x - D / M * x_dot + K / M * a 
    else:
        # print("false")
        return -K / M * x - D / M * x_dot + K / M * a


# Function to compute acceleration in y-direction
def acceleration_y(x, y, x_dot, y_dot):
    if x ** 2 + y ** 2 <= R ** 2:
        return  -K / M * y - D / M * y_dot + K / M * b 
    else:
        return -K / M * y - D / M * y_dot + K / M * b


# Finite difference solver
def solve():
    num_steps = int(total_time / timestep)
    x = np.zeros(num_steps)
    y = np.zeros(num_steps)
    x_dot = np.zeros(num_steps)
    y_dot = np.zeros(num_steps)

    x[0] = x0
    y[0] = y0
    x_dot[0] = x_dot0
    y_dot[0] = y_dot0

    for i in range(1, num_steps):
        x_dot_dot = acceleration_x(x[i - 1], y[i - 1], x_dot[i - 1], y_dot[i - 1])
        y_dot_dot = acceleration_y(x[i - 1], y[i - 1], x_dot[i - 1], y_dot[i - 1])

        x_dot[i] = x_dot[i - 1] + x_dot_dot * timestep
        y_dot[i] = y_dot[i - 1] + y_dot_dot * timestep

        x[i] = x[i - 1] + x_dot[i] * timestep
        y[i] = y[i - 1] + y_dot[i] * timestep

    return x, y


# Plot trajectory
x_traj, y_traj = solve()
plt.plot(x_traj, y_traj, 'b', label='Trajectory')

# calculate equilibrium radius
r_eq = P*R/(K+P)
print("equilibrium radius is ", r_eq)

# Plot circle with radius 0.4 around the origin
circle = plt.Circle((0, 0), 0.4, color='r', fill=False, linestyle='--')
plt.gca().add_patch(circle)

# Plot equilibruim radius circle with radius r_eq around the origin
circle = plt.Circle((0, 0), r_eq, color='k', fill=False)
plt.gca().add_patch(circle)

# Plot point (a, b)
plt.plot(a, b, 'go', label='(a, b)')

# Plot x0
plt.plot(x0, y0, 'g+', label='x0')

# Plot final trajectory point
plt.plot(x_traj[-1], y_traj[-1], 'g+', markersize=30, label='final point')

# Plot circle with radius 0.4 around the origin
circle = plt.Circle((0, 0), 0.4, color='r', fill=False)
plt.gca().add_patch(circle)

plt.xlabel('x')
plt.ylabel('y')
plt.title('Trajectory in x-y Plane')
plt.legend()
plt.axis('equal')
plt.grid(True)
plt.show()