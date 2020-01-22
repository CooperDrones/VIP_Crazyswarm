import numpy as np
import matplotlib.pyplot as plt 

from crazyflie_dynamics import CrazyflieDynamics
from data_plotter import DataPlotter
import crazyflie_param as P

# self.state = np.array([
#     [P.x0],     # 0
#     [P.y0],     # 1
#     [P.z0],     # 2
#     [P.psi0],   # 3
#     [P.theta0], # 4
#     [P.phi0],   # 5
#     [P.u0],     # 6
#     [P.v0],     # 7
#     [P.w0],     # 8
#     [P.r0],     # 9
#     [P.q0],     # 10
#     [P.p0],     # 11
# ])

class RateController:
    def __init__(self, kp_p=70.0, kp_q=70.0, kp_r=70.0, ki_r=16.7):
        self.kp_p = kp_p    # Roll Rate Proportional Gain
        
        self.kp_q = kp_q    # Pitch Rate Proportional Gain
        
        self.kp_r = kp_r    # Yaw Rate Proportional Gain
        self.ki_r = ki_r    # Yaw Rate Integral Gain
        self.e_r_hist = 0.0 # Initialize Historical Error

    def update(self, p_c, q_c, r_c, state):
        del_phi = self.kp_p * (p_c - state.item(11))
        
        del_theta = self.kp_q * (q_c - state.item(10))

        e_r = r_c - state.item(9)
        self.e_r_hist += e_r
        del_psi = self.kp_r * (r_c - state.item(9)) + (self.ki_r * self.e_r_hist)

        return del_phi, del_theta, del_psi
        # used in control mixer

class AttitudeController:
    def __init__(self, kp_phi=3.5, ki_phi=3.5, kp_theta=3.5, ki_theta=2.0):
        self.kp_phi = kp_phi     # Roll Attitude Proportional Gain
        self.ki_phi = ki_phi     # Roll Attitude Integral Gain
        self.e_phi_hist = 0.0
        
        self.kp_theta = kp_theta # Pitch Attitude Proportional Gain
        self.ki_theta = ki_theta # Pitch Attitude Integral Gain
        self.e_theta_hist = 0.0

    def update(self, phi_c, theta_c, state):
        # Calculate errors
        e_phi = phi_c - state.item(5)
        self.e_phi_hist += e_phi
        p_c = self.kp_phi * (e_phi) + (self.ki_phi * self.e_phi_hist)
        
        e_theta = theta_c - state.item(4)
        self.e_theta_hist += e_theta
        q_c = self.kp_theta * (e_theta) + (self.ki_theta * self.e_theta_hist)
   
        return p_c, q_c
        # used in the rate controller

class ControlMixer:
    def __init__(self):
        self.temp = 0.0

    def update(self, omega, del_phi, del_theta, del_psi):
        u_pwm = np.array([
            [omega - del_phi/2 - del_theta/2 - del_psi],
            [omega + del_phi/2 - del_theta/2 + del_psi],
            [omega + del_phi/2 + del_theta/2 - del_psi],
            [omega - del_phi/2 - del_theta/2 + del_psi], 
        ])
        return u_pwm

class AltitudeController:
    def __init__(self, kp=11000, ki=3500, kd=9000):
        self.omega_e = 44705 # Eq. 3.1.8
        self.kp = kp
        self.ki = ki
        self.e_hist = 0.0
        self.kd = kd
        self.e_prev = 0.0

        self.t = P.Ts

    def update(self, z_c, z):
        e = z_c - z
        self.e_hist += e # historical error
        e_der = (e - self.e_prev) / self.t # dirty derivative error
        self.e_prev = e
        del_cap_omega = self.omega_e + (self.kp * e) + (self.ki * self.e_hist) + (self.kd * e_der)
        return del_cap_omega

if __name__ == "__main__":
    cf = CrazyflieDynamics()
    plot = DataPlotter()

    rate_ctrl = RateController()
    attitude_ctrl = AttitudeController()
    ctrl_mixer = ControlMixer()
    altitiude_ctrl = AltitudeController()

    # off-borad controller input values
    u_ob = np.array([
        [0.0], # roll
        [0.0], # pitch
        [0.0], # yaw rate
        [0.0], # thrust
    ])

    # reference values
    r = np.array([
        [0.0], # x
        [0.0], # y
        [0.5], # z
        [0.0], # psi
        [0.0], # theta 
        [0.0], # phi
    ])

    r_c = 0.0
    phi_c = 0.0; theta_c = 0.0

    t = P.t_start

    for _ in range(100):
        t = t + P.Ts

        u_ob[3,0]                   = altitiude_ctrl.update(r.item(2), cf.state.item(2))
        print(u_ob)
        
        p_c, q_c                    = attitude_ctrl.update(phi_c, theta_c, cf.state)
        del_phi, del_theta, del_psi = rate_ctrl.update(p_c, q_c, r_c, cf.state)
        u_pwm                       = ctrl_mixer.update(u_ob.item(3), del_phi, del_theta, del_psi)

        u = cf.pwm_to_rpm(u_pwm)
        y = cf.update(u)

        plot.update(t, r, cf.state, u)
        plt.pause(0.01)
    
    print('Press key to close')
    plt.waitforbuttonpress()
    plt.close()