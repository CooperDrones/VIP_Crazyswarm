import time

class Control_system():
    def __init__(self, kp = 1, ki = 0, kd = 0):
        self.kp = kp    # Proportional gain
        self.ki = ki    # Integral gain
        self.kd = kd    # Derivative gai
        self.state = 0
        self.acc_error = 0      # error integral
        self.der_error = 0      # error derivative
        self.prev_error = 0
        self.u_prev = 0
        self.prev_pos = 0
        self.t0 = 0

    def pid(self, des, curr):
        if self.state == 0:              # Start initial time
            self.state = 1
            self.t0 = time.clock()

        t1 = time.clock()           # Offset time
        dt = t1 - self.t0

        error = des - curr          # error
        up = self.kp * error        # actuator signal for proportional

        # sigma = 1.0 # Dirty-bandwidth
        # velocity = (2*sigma - dt)/(2*sigma + dt) * self.u_prev + 2/(2*sigma + dt) * (curr - prev_pos)


        if self.ki != 0:
            self.acc_error = self.acc_error + (error + self.prev_error) * dt / 2    # integral error
        ui = self.ki * self.acc_error        # actuator signal for integral


        if self.kd != 0:
            self.der_error = (error - self.prev_error) / dt                         # derivative error
        ud = self.kd * self.der_error        # actuator signal for derivative

        self.t0 = t1
        self.prev_error = error
        u = up + ui + ud
        self.u_prev = u
        self.prev_pos = curr
        return u
