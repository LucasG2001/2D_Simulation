import numpy as np
import matplotlib.pyplot as plt
from equilibrium_radius import get_K_hat

# Initial conditions
a = 0.3 # goal x
b = 0.3 # goal y
#x0 = -0.282005
#y0 = -0.282005
x0 = -0.4
y0 = 0.0
x_dot0 = 3.0
y_dot0 = 0.0

# Time parameters
timestep = 0.001
total_time = 10.0

# safety Bubble
r_eq = 0.1
R = 0.4  # safety bubble radius [m]

# Impedance Parameters
K = 200.0  # spring constant [N/m]
M = 0.7  # mass [kg]
D = 2 * np.sqrt(M*K)  # damping coefficient [Ns/m]
Px = get_K_hat(R, r_eq, K, a, b, x0, y0)
Py = Px # choose same stiffness in all directions
D_h = 2.1 * np.sqrt(M)*np.sqrt(K+Px) - D # safety bubble damping [Ns/m]

# Function to compute hand repulsive force in x diraction
def F_h_x(x, y, x_dot=0, y_dot=0):
    if x ** 2 + y ** 2 <= R ** 2:
        # print("true")
        return Px * R * np.cos(np.arctan2(y, x)) - Px * x - D_h * x_dot * x **2 / (x**2 + y**2) # take only the projected velocity
    else:
        # print("false")
        return 0
    
# Function to compute hand repulsive force in y direction
def F_h_y(x, y, x_dot=0, y_dot=0):
    if x ** 2 + y ** 2 <= R ** 2:
        # print("true")
        return  Py * R * np.sin(np.arctan2(y, x)) - Py * y - D_h * y_dot * y**2 / (x**2 + y**2)  # take only the projected velocity
    else:
        # print("false")
        return 0
    

# Function to compute acceleration in x-direction
def acceleration_x(x, y, x_dot, y_dot):
    if x ** 2 + y ** 2 <= R ** 2:
        # print("true")
        return -K / M * x - D / M * x_dot + K / M * a + F_h_x(x, y, x_dot, y_dot)/M 
    else:
        # print("false")
        return -K / M * x - D / M * x_dot + K / M * a


# Function to compute acceleration in y-direction
def acceleration_y(x, y, x_dot, y_dot):
    if x ** 2 + y ** 2 <= R ** 2:
        return  -K / M * y - D / M * y_dot + K / M * b + F_h_y(x, y, x_dot, y_dot)/M
    else:
        return -K / M * y - D / M * y_dot + K / M * b


# Finite difference solver
def solve():
    num_steps = int(total_time / timestep)
    overshoot_error = np.ones(num_steps) * (r_eq-R) # initialize with initial values
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

        # calculate overshoot of r_eq (penetration into critical safety zone)
        overshoot_error[i] = r_eq - np.sqrt(x[i]**2 + y[i]**2) 

    return x, y, overshoot_error


# Plot trajectory
x_traj, y_traj, max_penetration_error = solve()
plt.plot(x_traj, y_traj, 'b', label='Trajectory')

print("equilibrium radius is ", r_eq)
print("maximum penetration error is ", np.max(max_penetration_error), "meters ")

# Plot circle with radius R around the origin
circle = plt.Circle((0, 0), R, color='r', fill=False, linestyle='--')
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

