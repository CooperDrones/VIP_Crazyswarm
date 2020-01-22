import numpy as np


# TODO: determine how to define ref inputs

class RateController:
    def __init__(self, ref, state):
        self.kp_p = 70.0 # Roll Rate Proportional Gain
        self.kp_q = 70.0 # Pitch Rate Proportional Gain
        self.kp_r = 70.0 # Yaw Rate Proportional Gain
        self.ki_r = 16.7 # Yaw Rate Integral Gain

    def rate_ctrl():
        # Calculate errors
        e_p = ref.item(11) - state.item(11)
        e_q = ref.item(10) - state.item(10)
        e_r = ref.item(9)  - state.item(9)

        # returns del phi, del theta, del psi 
        # used in control mixer

class AttitudeController:
    def __init__(self, ref, state):
        self.kp_phi = 3.5   # Roll Attitude Proportional Gain
        self.ki_phi = 2.0   # Roll Attitude Integral Gain
        self.kp_theta = 3.5 # Pitch Attitude Proportional Gain
        self.ki_theta = 2.0 # Pitch Attitude Integral Gain

    def attitude_control():
        # Calculate errors
        e_phi = ref.item(5) - state.item(5)
        e_theta = ref.item(4) - state.item(4)

        # returns p_c and q_c 
        # used in the rate controller

class ControlMixer:
    def __init__(self, ob_ref, state)

if __name__ == "__main__":
    rate_ctrl = RateController()
