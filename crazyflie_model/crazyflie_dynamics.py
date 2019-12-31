import numpy as np
import crazyflie_param as P

class Quadrotor:
    def __init__(self):
        # Initial state conditions
        self.state = np.array([
            [P.x0],
            [P.y0],
            [P.z0],
            [P.u0],
            [P.v0],
            [P.w0],
            [P.phi0],
            [P.theta0],
            [P.psi0],
            [P.p0],
            [P.q0],
            [P.r0],
        ])
        # Time step
        self.Ts = P.Ts
        # Crazyflie 2.0 mass
        self.m = P.m
        # Gravitation acceleration
        self.g = P.g
        # Actuator limits
        self.input_limits = np.array([
            [P.phi_max],
            [P.theta_max],
            [P.r_max],
            [P.thrust_max],
        ])

        # TODO inner loop rate controller 
        # IN: pc, qc, rc, p, q, r 
        # OUT: del_phi, del_theta, del_gamma
    

    def update(self, u):
        # External method that takes input u at time t
        # and returns the output y at time t

        # TODO u refers to omega1-4 here, NOT control inputs
        # Find relationship from control inputs 
        u = self.saturate(u, self.input_limits)
        # self.rk4_step(u) # propagate the state by one time sample
        self.euler_step(u)
        y = self.h() # return the corresponding output
        return y
    
    def euler_step(self, u):
        
        self.state += self.Ts * ()

    def saturate(self, u, limit):
        for idx in range(u.shape[0]):
            if abs(u[idx,0]) > limit[idx,0]:
                u[idx,0] = limit[idx,0]*np.sign(u[idx,0])
        return u

# Run some tests to see functionality
if __name__ == "__main__":
    cf1 = Quadrotor()
    # print(cf1.state)
    u = np.array([
        [40.0],
        [-50.0],
        [500.0],
        [70000.0],
    ])
    u = cf1.saturate(u, cf1.input_limits)
    cf1.euler_step(u)

    print(cf1.state)
