import numpy as np
import math
roAir = 1.225   # kg/m3 density of air
roWater = 1000  # kg/m3
g = 9.8         # m/s2
# All the formula's took straight from the slides
class BEMTheory:
    def __init__(self, no_of_blades, angular_vel, radius, lift_slope,drag, linear_twist,  climb_vel, root_cutouts , linear_taper, medium = 'air') -> None:
        self.linear_twist = linear_twist; self.linear_taper = linear_taper
        self.root_cutouts = root_cutouts
        self.climb_vel    = climb_vel
        self.omega = angular_vel; self.R = radius; self.R_co = root_cutouts
        self.a = lift_slope
        self.b = no_of_blades
        self.c_d = drag
        if medium == 'air':
            self.ro = roAir
        elif medium == 'water':
            self.ro = roWater
    
    def calcLamda(self, r_ratio, F):
        l_c = self.climb_vel/(self.omega*self.R)
        sigma = self.b*self.linear_taper(r_ratio)/(self.omega*self.R)
        self.sigma = sigma
        self.F = F
        lamda = np.sqrt( ((self.sigma*self.a/(16*self.F) - l_c/2)**2)+ (self.sigma*self.a*self.linear_twist(r_ratio)*r_ratio/8) ) - (self.sigma*self.a/(16*self.F) - l_c/2)
        self.lamda = round(lamda, 5)
        return self.lamda
    
    def tip_loss(self,r_ratio, lamda):
        f = self.b*(1- r_ratio)/(2*lamda)
        F = (2/np.pi)*np.arccos(np.exp(-1*f))
        self.F = round(F,5)
        return self.F
    # With a initialization of F = 0.9 we calucalte lamda_1 and with lamda_1 we calculate F_2 then later lamda_2..... To converge lamda, F that's what this below function is doing
    # We decide convergence if prev and current value diffrence is less than 1e-4
    def lamda_tipLoss(self,r_ratio):
        fs = [0.9]
        lamdas = [self.calcLamda(r_ratio, 0.9)]
        for _ in range(10):
            F = self.tip_loss(r_ratio, lamdas[-1])
            lamda = self.calcLamda(r_ratio, F)
            lamdas.append(lamda); fs.append(F)
            if abs(lamda-lamdas[-2])<1e-4 and abs(F-fs[-2])<1e-4:
                break
        self.F = fs[-1]
        self.lamda = lamdas[-1]
        return self.lamda, self.F
    
    def T(self):
        T = 0
        radius = np.linspace(self.R_co, self.R, 1000)
        dr = radius[1] - radius[0]
        for i,r in enumerate(radius):
            if i == len(radius)-1:
                break
            r_ratio = r/self.R
            self.dr = dr
            lamda , F = self.lamda_tipLoss(r_ratio)
            dT = 0.5*self.ro*self.a*self.b*self.linear_taper(r_ratio)*((lamda*self.omega*self.R)**2 + (self.omega*r)**2)*(self.linear_twist(r_ratio) - lamda/r_ratio)*dr
            T += dT
        return T
    def AoA (self, r_ratio):
        lamda , F = self.lamda_tipLoss(r_ratio)
        phi = lamda/r_ratio
        AoA = self.linear_twist(r_ratio) - phi
        return AoA
    def P(self):
        dP = 0
        radius = np.linspace(self.R_co, self.R, 1000)
        dr = radius[1] - radius[0]
        for i,r in enumerate(radius):
            if i == len(radius) -1:
                break
            r_ratio = r/self.R
            lamda , F = self.lamda_tipLoss(r_ratio)
            phi = np.arctan(lamda/r_ratio)
            AoA = self.linear_twist(r_ratio) - phi
            dP += 0.5*self.ro*self.omega*self.b*self.linear_taper(r_ratio)*r*((lamda*self.omega*self.R)**2 + (self.omega*r)**2)*(self.c_d(AoA) + self.a*AoA*phi)*dr
        return dP
    def C_T(self):
        T = self.T()
        coeffThrust = T/(self.ro*np.pi*(self.R**2)*((self.omega*self.R)**2))
        self.c_t = round(coeffThrust, 5)
        return self.c_t
    def C_P(self):
        P = self.P()
        coeffPower = P/(self.ro*np.pi*(self.R**2)*((self.omega*self.R)**3))
        self.c_p = round(coeffPower, 5)
        return self.c_p
    def Q(self):
        P = self.P()
        self.Q = P/self.omega
        return self.Q
    def C_Q(self):
        Q = self.Q()
        coeffTorque = Q/(roAir*np.pi*(self.R**2)*self.R*((self.omega*self.R)**2))
        self.c_q = coeffTorque
        return self.c_q