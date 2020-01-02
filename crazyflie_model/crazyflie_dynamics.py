import numpy as np
import crazyflie_param as P

class Quadrotor:
    def __init__(self):
        # Initial state conditions
        self.state = np.array([
            [P.x0],
            [P.y0],
            [P.z0],
            [P.psi0],
            [P.theta0],
            [P.phi0],
            [P.u0],
            [P.v0],
            [P.w0],
            [P.r0],
            [P.q0],
            [P.q0],
        ])
        # Time step
        self.Ts = P.Ts
        # Crazyflie 2.0 mass
        self.m = P.m
        # Gravitation acceleration
        self.g = P.g
        # Hover RPM
        self.omega_e = P.omega_e

        # Hover state space representation 
        # A and B matrix
        self.A = P.A
        self.B = P.B
        
        # Outer Loop Actuator limits
        self.ol_input_limits = P.ol_input_limits
        # Prop RPM Actuator Limits
        self.input_limits = P.input_limits

        # TODO inner loop rate controller 
        # IN: pc, qc, rc, p, q, r 
        # OUT: del_phi, del_theta, del_gamma

    def update(self, u):
        """
        External method that takes outer loop control commands
        and updates the state
        """
        u = self.saturate(u, self.input_limits)
        self.rk4_step(u) # propagate the state by one time sample
        # self.euler_step(u)
        y = self.h() # return the corresponding output
        return y
    
    def state_dot(self, state, u):
        # Returns the deivative of the state vector
        # Uses state-space equations provided on pg. 15
        xdot = self.A @ self.state + self.omega_e * (self.B @ u)
        return xdot
    
    def h(self):
        # Returns y = h(x) - Finds position and orientation
        # from state and combines into the output vector
        y = np.array([
            [self.state.item(0)], # x
            [self.state.item(1)], # y
            [self.state.item(2)], # z
            [self.state.item(3)], # psi
            [self.state.item(4)], # theta
            [self.state.item(5)], # pi
        ])
        return y

    def euler_step(self, u):
        # Integrate ODE using Euler's method
        xdot = self.state_dot(self.state, u)
        self.state += self.Ts * xdot

    def rk4_step(self, u):
        # Integrate ODE using Runge-Kutta RK4 algorithm
        F1 = self.state_dot(self.state, u)
        F2 = self.state_dot(self.state + self.Ts / 2 * F1, u)
        F3 = self.state_dot(self.state + self.Ts / 2 * F2, u)
        F4 = self.state_dot(self.state + self.Ts * F3, u)
        self.state += self.Ts / 6 * (F1 + 2 * F2 + 2 * F3 + F4)

    def saturate(self, u, limit):
        for idx in range(u.shape[0]):
            if abs(u[idx,0]) > limit[idx,0]:
                u[idx,0] = limit[idx,0]*np.sign(u[idx,0])
        return u

# Run some tests to see functionality
if __name__ == "__main__":
    cf1 = Quadrotor()

    # # Test saturation function with external control commands
    # # roll, pitch, yaw rate, thrust
    # olu = np.array([
    #     [40.0],
    #     [-50.0],
    #     [500.0],
    #     [70000.0],
    # ])
    # olu = cf1.saturate(olu, cf1.input_limits)
    # print(olu)

    # Symmetric input
    u = np.array([
        [10000],
        [10000],
        [10000],
        [10000],
    ])

    # # Test euler integrator with symmetric input [RPM] 
    # print("Euler's method\n", cf1.state)
    # for _ in range(3):
    #     cf1.euler_step(u)
    #     print(cf1.state)

    # # Test rk4 integrator with symmetric input [RPM]
    cf2 = Quadrotor()
    print("RK4\n", cf2.state)
    for _ in range(3):
        cf2.rk4_step(u)
        print(cf2.state)

    # for _ in range(3):
    #     y = cf1.update(u)
    #     print(y)