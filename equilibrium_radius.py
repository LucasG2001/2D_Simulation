import numpy as np
import math
def get_K_hat(R, r_eq, K, g_x, g_y, x_0, y_0):
    G = np.sqrt(g_x **2 + g_y ** 2)
    beta = np.arctan2(y_0,x_0)
    epsilon = np.cos(beta)
    rho = np.sin(beta)
    # polynomial equation coefficients
    A = R**2 - r_eq**2
    B = 2*K*(R*(g_x * epsilon + g_y * rho) - r_eq**2)
    B_alt = -2 * K * (G*R + r_eq**2)
    C = K**2 * (G**2 - r_eq**2)
    # solution for P (K_hat)
    P1 = (-B + np.sqrt(B**2 - 4 * A * C)) /(2 * A)
    P2 = (-B - np.sqrt(B**2 - 4 * A * C)) /(2 * A)
    # return maximum value
    print("K_hat is ", np.max([P1, P2]))
    return np.max([P1, P2])

if __name__ == "__main__":
    R = 0.4
    r_eq = 0.1
    K = 200
    M = 0.7
    D = 2 * np.sqrt(M*K)
    g_x = 0.3 # goal x
    g_y = 0.3 # goal y
    G = np.sqrt(g_x**2 + g_y**2)
    print("G is ", G)
    x_0 = -0.4
    y_0 = -0.01
    beta = np.arctan2(y_0,x_0)
    print("Beta is ", beta * 180 / 3.14156, " DEGREES")
    epsilon = np.cos(beta)
    rho = np.sin(beta)
    print("epsilon is ", epsilon)
    print("rho is ", rho)
    # polynomial equation coefficients
    A = R**2 - r_eq**2
    B = 2*K*(R*(g_x * epsilon + g_y * rho) - r_eq**2)
    B_alt = -2 * K * (G*R + r_eq**2)
    print("B is ", B)
    print("B_alt is ", B_alt)
    C = K**2 * (G**2 - r_eq**2)

    # solution for P (K_hat)
    P1 = (-B + np.sqrt(B**2 - 4 * A * C)) /(2 * A)
    P2 = (-B - np.sqrt(B**2 - 4 * A * C)) /(2 * A)

    print(A, B, C)
    print("K_hat1 is ", P1)
    print("K_hat2 is ", P2)

