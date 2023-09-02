import numpy as np
import math
roAir = 1.204   # kg/m3 density of air
roWater = 1000  # kg/m3
g = 9.8         # m/s2
Cd = 0.01
class BEMTheory:
    def __init__(self, no_of_blades, angular_vel, radius, lift_slope, linear_twist,  climb_vel, root_cutouts , linear_taper= None) -> None:
        self.linear_twist = linear_twist; self.linear_taper = linear_taper
        self.root_cutouts = root_cutouts
        self.climb_vel    = climb_vel
        self.omega = angular_vel; self.R = radius; self.R_co = root_cutouts
        self.a = lift_slope
        self.b = no_of_blades
        # self.alpha_optimum  = alpha_optimum * np.pi/180 # Got this from NACA2412 data
        
        self.F = None # Prandtl Lift loss coefficient
    
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
    
    def lamda_tipLoss(self,r_ratio):
        fs = [1]
        lamdas = [self.calcLamda(r_ratio, 1)]
        for _ in range(10):
            F = self.tip_loss(r_ratio, lamdas[-1])
            lamda = self.calcLamda(r_ratio, F)
            lamdas.append(lamda); fs.append(F)
            if abs(lamda-lamdas[-2])<1e-4 and abs(F-fs[-2])<1e-4:
                break
        self.F = fs[-1]
        self.lamda = lamdas[-1]
        return self.lamda, self.F
    
    # def T_tipLoss(self):
    #     dT = 0
    #     radius = np.linspace(self.R_co, self.R, 1000)
    #     dr = radius[1] - radius[0]
    #     for i,r in enumerate(radius):
    #         if i == len(radius) -1:
    #             break
    #         lamda , F = self.lamda_tipLoss(r/self.R)
    #         v = lamda*self.omega*self.R - self.climb_vel
    #         dT += 4*np.pi*roAir*r*F*(self.climb_vel + v)*v*dr
    #     return dT
    
    def T_traditional(self):
        T = 0
        radius = np.linspace(self.R_co, self.R, 1000)
        dr = radius[1] - radius[0]
        for i,r in enumerate(radius):
            if i == len(radius)-1:
                break
            r_ratio = r/self.R
            self.dr = dr
            lamda , F = self.lamda_tipLoss(r_ratio)
            dT = 0.5*roAir*self.a*self.b*self.linear_taper(r_ratio)*((lamda*self.omega*self.R)**2 + (self.omega*r)**2)*(self.linear_twist(r_ratio) - lamda/r_ratio)*dr
            # if abs(r_ratio*self.R -0.15) < 1e-3:
            #     print(r_ratio*self.R, dT/dr)
            T += dT
        return T
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
            dP += 0.5*roAir*self.omega*self.b*self.linear_taper(r_ratio)*r*((lamda*self.omega*self.R)**2 + (self.omega*r)**2)*(Cd + self.a*AoA*phi)*dr
        return dP
    def C_T(self):
        T = self.T_traditional()
        coeffThrust = T/(roAir*np.pi*(self.R**2)*((self.omega*self.R)**2))
        self.c_t = round(coeffThrust, 5)
        return self.c_t
    def C_P(self):
        P = self.P()
        coeffPower = P/(roAir*np.pi*(self.R**2)*((self.omega*self.R)**3))
        self.c_p = round(coeffPower, 5)
        return self.c_p