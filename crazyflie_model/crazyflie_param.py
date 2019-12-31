import numpy as np

g = 9.81 # [m/s^2]

# Physical Parameters of the Crazyflie 2.0
m = 0.35        # Crazyflie 2.0 mass w/ 4 Vicon markers [g]
Ct = 3.1582e-10 # Thrust coeff [N/rpm^2]
Cd = 7.9379e-10 # Drag coeff [Nm/rpm^2]
Ixx = 1.395e-5  # [Kg x m^2]
Iyy = 1.436e-5  # [Kg x m^2]
Izz = 2.173e-5  # [Kg x m^2]
d = 39e-3       # Crazyflie 2.0 arm length [m]

# Initial Conditions, state vector formed following:
# https://arxiv.org/pdf/1608.05786.pdf
x0 = 0.0     # X position [m]
y0 = 0.0     # Y position [m]
z0 = 0.0     # Z position [m]
u0 = 0.0     # Body frame X linear velocity [m/s]
v0 = 0.0     # Body frame Y linear velocity [m/s]
w0 = 0.0     # Body frame Z linear velocity [m/s]
phi0 = 0.0   # Roll angle [rad]
theta0 = 0.0 # Pitch angle [rad]
psi0 = 0.0   # Yaw angle [rad]
p0 = 0.0     # Body frame roll angular velocity [rad/s]
q0 = 0.0     # Body frame pitch angular velocity [rad/s]
r0 = 0.0     # Body frame yaw angular velocity [rad/s]

# Simulation Parameters
t_start = 0.0 # [s]
t_end = 50.0  # [s]
Ts = 0.01     # [s]
t_plot = 0.1  # [s]

# Saturation Limits
phi_max = 30.0       # [rad]
theta_max = 30.0     # [rad]
r_max = 200.0        # [rad/s]
thrust_max = 60000.0 # mapped to PWM output [N/A]

# # Construction of the necessary matrices
# N = 12
# M = 4
# A = np.zeros((N, N))

# A = np.zeros((N, N))
# A[0, 6] = 1.
# A[1, 7] = 1.
# A[2, 8] = 1.
# A[3, 9] = 1.
# A[4, 10] = 1.
# A[5, 11] = 1.
# A[6, 4] = g
# A[7, 5] = -g

# B = np.zeros((N, M))
# B[8, 0] = 2. * (Ct/m)
# B[8, 1] = 2. * (Ct/m)
# B[8, 2] = 2. * (Ct/m)
# B[8, 3] = 2. * (Ct/m)
# B[9, 0] = -2. * (Cd/Izz)
# B[9, 1] = 2. * (Cd/Izz)
# B[9, 2] = -2. * (Cd/Izz)
# B[9, 3] = 2. * (Cd/Izz)
# B[10, 0] = -np.sqrt(2) * d * (Ct/Iyy)
# B[10, 1] = np.sqrt(2) * d * (Ct/Iyy)
# B[10, 2] = np.sqrt(2) * d * (Ct/Iyy)
# B[10, 3] = -np.sqrt(2) * d * (Ct/Iyy)
# B[11, 0] = -np.sqrt(2) * d * (Ct/Ixx)
# B[11, 1] = -np.sqrt(2) * d * (Ct/Ixx)
# B[11, 2] = np.sqrt(2) * d * (Ct/Ixx)
# B[11, 3] = np.sqrt(2) * d * (Ct/Ixx)

# C = np.eye(12)
