import numpy as np
import matplotlib.pyplot as plt 

# Import crazyflie model modules
import sys
sys.path.append("../")
sys.path.append("../crazyflie_demo")
from data_plotter import DataPlotter
from crazyflie_dynamics import CrazyflieDynamics
from crazyflie_controller import RateController, AttitudeController, ControlMixer, AltitudeController, XYController, YawController
import crazyflie_param as P

def test_altitude(z_c):
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
        [z_c], # z
        [0.0], # psi
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
            # print("time is at: ", counter)
            t_next_ob = t + P.t_ob
            u_ob[3,0] = altitiude_ctrl.update(r.item(2), cf.state.item(2))
            # print("thrust:, \n", u_ob.item(3))

            u = ctrl_mixer.update(u_ob.item(3), del_phi, del_theta, del_psi) # output is PWM signal
            # print("u_pwm: \n", u_pwm.item(0))
            
            y = cf.update(u) # rpm is used in cf state update equations
            t = t + P.t_ob
            
        plot.update(t, r, cf.state, u)
        plt.pause(0.0001)
    
    print('Press key to close')
    plt.waitforbuttonpress()
    plt.close()

def test_xy(x_c, y_c, z_c, psi_c):
    cf = CrazyflieDynamics()
    plot = DataPlotter()

    # Create class objects
    rate_ctrl = RateController()
    attitude_ctrl = AttitudeController()
    rate_ctrl = RateController()
    ctrl_mixer = ControlMixer()
    altitiude_ctrl = AltitudeController()
    xy_ctrl = XYController()
    yaw_ctrl = YawController

    # off-borad controller input values
    u_ob = np.array([
        [0.0], # pitch (phi -> x)  - 0
        [0.0], # roll (theta -> y) - 1
        [0.0], # yaw rate          - 2
        [0.0], # thrust            - 3
    ])

    # reference values
    r = np.array([
        [x_c], # x                 - 0
        [y_c], # y                 - 1
        [z_c], # z                 - 2
        [psi_c], # psi             - 3
    ]) 

    # r_c = 0.0
    # phi_c = 0.0; theta_c = 0.0
    # del_phi = 0.0; del_theta = 0.0; del_psi = 0.0

    t = P.t_start

    # for _ in range(100):
    while t < P.t_end: # plotter can run the slowest
        t_next_plot = t + P.t_plot
        
        while t < t_next_plot: # offboard controller is slowest at 100 hz
            t_next_ob = t + P.t_ob
            
            # Altitude off-board controller update
            u_ob[3,0] = altitiude_ctrl.update(r.item(2), cf.state.item(2))
            print("thrust value", u_ob.item(3))
            # print("z error ", altitiude_ctrl.e_hist)

            # XY off-borad controller update
            # TODO: cap is not working
            u_ob[0,0], u_ob[1,0] = xy_ctrl.update(r.item(0), cf.state.item(0), \
                r.item(1), cf.state.item(1), cf.state.item(3), P.t_ob)
            print("x control ", u_ob[0,0])
            print("y control ", u_ob[1,0])

            while t < t_next_ob: # attitude controller runs at 250 hz
                t_next_att = t + P.t_att
                # phi controls x, theta controls y
                # TODO: cap rate controllers
                p_c, q_c = attitude_ctrl.update(u_ob.item(0), u_ob.item(1), cf.state)
                print("roll angular velocity control, ", p_c)
                print("pitch angular velocity control, ", q_c)

                del_phi, del_theta, del_psi = rate_ctrl.update(p_c, q_c, u_ob.item(3), cf.state)
                print("del phi ", del_phi)
                print("del theta ", del_theta)
                # print("del psi ", del_psi)

                while t < t_next_att: # rate controller is the fastest running at 500 hz
                    u = ctrl_mixer.update(u_ob.item(3), del_phi, del_theta, del_psi)
                    y = cf.update(u)
                    t = t + P.t_rate

        plot.update(t, r, cf.state, u)
        plt.pause(0.5)
    
    print('Press key to close')
    plt.waitforbuttonpress()
    plt.close()

if __name__ == "__main__":
    # test_altitude(0.5) # works! 2/16/2020

    # Fly to x reference value
    test_xy(10.0, 0.0, 0.0, 0.0)