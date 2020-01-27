import numpy as np
import matplotlib.pyplot as plt 
import sys
sys.path.append("../")

from data_plotter import DataPlotter
from crazyflie_dynamics import CrazyflieDynamics
from crazyflie_controller import RateController, AttitudeController, ControlMixer, AltitudeController
import crazyflie_param as P


def test_altitude():
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
        [1.0], # z
        [0.0], # psi
        [0.0], # theta 
        [0.0], # phi
    ])

    r_c = 0.0
    phi_c = 0.0; theta_c = 0.0
    del_phi = 0.0; del_theta = 0.0; del_psi = 0.0

    t = P.t_start

    counter = 0.0

    while t < P.t_end: # plotter can run slowly
        t_next_plot = t + P.t_plot
        
        while t < t_next_plot: # offboard controller is slowest at 100 hz
            counter += P.t_ob
            print("time is at: ", counter)
            t_next_ob = t + P.t_ob
            u_ob[3,0] = altitiude_ctrl.update(r.item(2), cf.state.item(2))
            print("thrust:, \n", u_ob.item(3))


            u = ctrl_mixer.update(u_ob.item(3), del_phi, del_theta, del_psi) # output is PWM signal
            # print("u_pwm: \n", u_pwm.item(0))
            
            # u = cf.pwm_to_rpm(u_pwm) # output is converted to rpm through Eq. 2.6.1
            print("u: \n", u.item(0))
            
            y = cf.update(u) # rpm is used in cf state update equations
            t = t + P.t_ob
            
        plot.update(t, r, cf.state, u)
        plt.pause(0.01)
    
    print('Press key to close')
    plt.waitforbuttonpress()
    plt.close()

def test_attitude():
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
        [1.0], # z
        [0.0], # psi
        [0.0], # theta 
        [0.0], # phi
    ])

    r_c = 0.0
    phi_c = 0.0; theta_c = 0.0
    del_phi = 0.0; del_theta = 0.0; del_psi = 0.0

    t = P.t_start

    counter = 0.0

    # for _ in range(100):
    while t < P.t_end: # plotter can run slowly
        t_next_plot = t + P.t_plot
        
        while t < t_next_plot: # offboard controller is slowest at 100 hz
            counter += P.t_ob
            print("time is at: ", counter)
            t_next_ob = t + P.t_ob
            u_ob[3,0] = altitiude_ctrl.update(r.item(2), cf.state.item(2))
            print("thrust:, \n", u_ob.item(3))

            # print("thrust value", u_ob.item(3))
            # print("z error ", altitiude_ctrl.e_hist)

            # while t < t_next_ob: # attitude controller is intermediate at 250 hz
            #     t_next_attitude = t + P.t_att
            #     p_c, q_c = attitude_ctrl.update(phi_c, theta_c, cf.state)
            #     # print("p_c and q_c ", p_c, q_c) # these are 0 for now with now x/y ref or noise

            #     while t < t_next_attitude: # rate controller is fastest at 500 hz
            #         del_phi, del_theta, del_psi = rate_ctrl.update(p_c, q_c, r_c, cf.state)
            #         # print("del_phi, del_theta, del_psi", del_phi, del_theta, del_psi) # these are 0 without x/y command or noise

                    # u_pwm = ctrl_mixer.update(u_ob.item(3), del_phi, del_theta, del_psi) # output is PWM signal
                    # u = cf.pwm_to_rpm(u_pwm) # output is converted to rpm through Eq. 2.6.1
                    # y = cf.update(u) # rpm is used in cf state update equations
                    # t = t + P.t_rate


            u = ctrl_mixer.update(u_ob.item(3), del_phi, del_theta, del_psi) # output is PWM signal
            # print("u_pwm: \n", u_pwm.item(0))
            
            # u = cf.pwm_to_rpm(u_pwm) # output is converted to rpm through Eq. 2.6.1
            print("u: \n", u.item(0))
            
            y = cf.update(u) # rpm is used in cf state update equations
            t = t + P.t_ob

            
        plot.update(t, r, cf.state, u)
        plt.pause(0.01)
    
    print('Press key to close')
    plt.waitforbuttonpress()
    plt.close()

if __name__ == "__main__":
    test_altitude()