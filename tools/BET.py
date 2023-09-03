import numpy as np
import math
roAir = 1.204   # kg/m3 density of air
roWater = 1000  # kg/m3
g = 9.8         # m/s2
Cd = 0.015
class BETheory:
    def __init__(self, no_of_blades, chord_mean, angular_vel, radius, lift_slope, linear_twist,  climb_vel, induced_vel, root_cutouts) -> None:
        self.linear_twist = linear_twist
        self.root_cutouts = root_cutouts
        self.climb_vel    = climb_vel;    self.induced_vel = induced_vel
        self.omega = angular_vel;         self.R = radius; self.R_co = root_cutouts
        self.a = lift_slope
        self.b = no_of_blades
        self.c = chord_mean
        # self.alpha_optimum  = alpha_optimum * np.pi/180 # Got this from NACA2412 data
        self.sigma = self.b*self.c/ (np.pi * self.R)

    def T(self):
        lamda = (self.climb_vel + self.induced_vel)/(self.omega*self.R)
        self.lamda = lamda
        T = 0.5*self.b*self.c*self.a*roAir*((self.omega*self.R)**2)*self.R*((1/3)*self.linear_twist(0.75) - lamda/2)
        self.thrust = round(T,5)
        return self.thrust
    def C_T(self):
        T = self.T()
        coeffThrust = T/(roAir*np.pi*(self.R**2)*((self.omega*self.R)**2))
        self.c_t = round(coeffThrust, 5)
        return self.c_t
    def C_P(self):
        lamda = (self.climb_vel + self.induced_vel)/(self.omega*self.R)
        coeffThrust = self.C_T()
        coeffPower = lamda*coeffThrust + self.sigma*Cd/8
        self.c_p = round(coeffPower, 5)
        return self.c_p
    def P(self):
        area = np.pi *(self.R**2)
        coeffPower = self.C_P()
        power = coeffPower*roAir*area*np.power(self.omega*self.R, 3)
        self.power = round(power, 5)
        return self.power
    def Q(self):
        P = self.P()
        self.Q = P/self.omega
        return self.Q
    def C_Q(self):
        Q = self.Q()
        coeffTorque = Q/(roAir*np.pi*(self.R**2)*self.R*((self.omega*self.R)**2))
        self.c_q = coeffTorque
        return self.c_q