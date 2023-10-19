import numpy as np
import math
roAir = 1.225   # kg/m3 density of air
roWater = 1000  # kg/m3
g = 9.8         # m/s2
# All the formula's took straight from the slides

class ForwardFlight:
    def __init__(self, pitch, pitch_climb, twist, forward_vel, climb_vel, omega_forward, omega_climb,
                 no_of_blades, chord, lift_slope, radius,root_cutouts,drag_coeff = 0.1, 
                 a_TPP=None, induced_vel = None, mass =40, medium = 'air') -> None:
        self.omega = omega_forward; self.R = radius; self.R_co = root_cutouts
        self.omega_climb = omega_climb
        self.a = lift_slope
        self.b = no_of_blades ; self.c= chord
        self.c_d = drag_coeff
        # ---------
        self.a_TPP = a_TPP
        self.forward_vel = forward_vel; self.induced_vel= induced_vel 
        self.climb_vel = climb_vel

        self.linear_twist = twist; 
        self.t_0 , self.t_1c, self.t_1s = pitch #these are wrt TPP plane.
        self.tita_0_climb, self.tita_1c_climb, self.tita_1s_climb = pitch_climb

        self.W = mass*9.8
        if medium == 'air':
            self.ro = roAir
        elif medium == 'water':
            self.ro = roWater

        self.beta_0 = None
        self.A = np.pi*(self.R**2)
        self.sigma = self.b*self.c/(self.omega_climb*self.R) # we use sigma for hover only..
        
    def TPPAngle(self):
        # Drag Estimation on the entire helicopter
        # two terms in Area one from the frontal area of the body(cube assumed)
        # second is assuming rotor to be a disk and it's projected area for drag. /2*np.pi (blade is at one azimuth not a disk)
        tpp_angles = [0.1] ; 
        while len(tpp_angles)<10:
            tpp = tpp_angles[-1]
            projected_area = np.power(0.01,2/3)+ self.b*(self.R- self.R_co)*self.c
            # Cd for a flate plate from NASA data is 1.28
            bD = 0.5*self.ro*(self.forward_vel**2)*projected_area* np.sin(tpp)*1.28
            bD = 1.7*bD
            new = np.arctan(bD/self.W)
            tpp_angles.append(new)
            if abs(tpp_angles[-1] - tpp_angles[-2]) < 1e-3:
                self.a_TPP = tpp_angles[-1]
                break
        if not self.a_TPP:
            self.a_TPP = tpp_angles[-1]
        
    def coning_angle(self):
        list_beta = [10*np.pi/180]
        def Intgl(li):
            b = li[-1]
            integral = 0
            si = np.linspace(0, 2*np.pi, 360)
            d_si = si[1]-si[0]
            r = np.linspace(self.R_co, self.R, 25)
            dr = r[1]- r[0]
            for azim in si:
                for radius in r:
                    integral += self.dL(azim, radius, b)*radius*dr*d_si
            I = (self.R**2- self.R_co**2)/2 # Blades unit length to be taken as 4 kg
            self.I = I
            return integral/(2*np.pi*(self.omega**2)*I)
            
        while len(list_beta)< 10:
            val = Intgl(list_beta)
            list_beta.append(val)
            if abs(list_beta[-1] - list_beta[-2]) < 1e-3:
                self.beta_0 = list_beta[-1]
                break
        if self.beta_0 is None:
            self.beta_0 = list_beta[-1]
        return self.beta_0
    
    def calculate_lad_i(self, ld_i_glauert, ld_f, r, azimuth):
        ld_g = ld_f + ld_i_glauert
        return ld_i_glauert*( 1 + (4/3)*(1/(1+ 1.2*ld_g/ld_f))*(r/self.R)*np.cos(azimuth) )
     
    def AoA_sectional(self, azimuth , r, b_0= None):
        if b_0 is None :
            if not self.beta_0:
                # don't need to go throught the whole process, if someother function called it
                self.coning_angle() 
            b_0 = self.beta_0
        # finding a_TPP
        if not self.a_TPP:
            self.TPPAngle()
        if not self.induced_vel:
            # This Function could be a lot better but the value of U_P changing is not my biggest concern
            self.induced_vel = 0.5*self.W/(self.ro*self.A*self.forward_vel*np.cos(self.a_TPP))

        tita = self.t_0 + self.linear_twist*r/self.R + (self.t_1c)*np.cos(azimuth) + (self.t_1s)*np.sin(azimuth)
        
        U_T = self.forward_vel*np.cos(self.a_TPP)*np.sin(azimuth) + self.omega*r

        ### GLAUERT'S METHOD
        prev_ld_i = 0.02
        ld_f = U_T/(self.omega*self.R)
        for _ in  range(20):
            new_ld_i = self.calculate_lad_i(prev_ld_i, ld_f, r, azimuth)
            if abs(new_ld_i - prev_ld_i) < 1e-2:
                break
            if abs(new_ld_i*self.omega*self.R) > 10:
                break 
                # This was blowing up for some cases to 1000 ms-1, so I put a limit, after that I don't care about .
            else:
                prev_ld_i = new_ld_i
        self.induced_vel = new_ld_i*self.omega*self.R


        U_P = self.forward_vel*np.sin(self.a_TPP)+ self.induced_vel + self.forward_vel*np.sin(b_0)*np.cos(azimuth)
        self.U_T = U_T; self.U_P = U_P; self.tita = tita

        if self.U_T <0: # The reversed flow region
            return np.arctan(U_P/U_T) + tita # Not sure about + for tita here
        else:
            return tita - np.arctan(U_P/U_T)

    def dL(self, azimuth, r, b_0= None):
        if b_0 is None :
            if not self.beta_0:
                self.coning_angle()
            b_0 = self.beta_0
        alpha = self.AoA_sectional(azimuth, r, b_0)
        if abs(alpha) < 14*np.pi/180:
            lift_coefficient = self.a
        elif abs(alpha) < 30*np.pi/180:
            lift_coefficient = 0.9/abs(alpha) # Make Cl constant = 0.9 for higher AoA 
            # I made a mistake of removing abs, but the lift is -ve ==> a*alpha, so if alpha is negative we gotta have the slope positive
        else:
            # raise ValueError('Yoo! hold ur horses AoA too high')
            lift_coefficient = 0 #This is unavoidable when ploting T vs pitch angles
        lift_sectional = 0.5*self.ro*self.c*lift_coefficient*alpha*(self.U_T**2 + self.U_P**2)
        return lift_sectional
    
    def dD(self,azimuth, r):
        self.AoA_sectional(azimuth, r) # AoA is not needed but calling this function updates the U_T,U_P values
        drag_sectional = 0.5*self.ro*self.c*self.c_d*(self.U_T**2 + self.U_P**2)
        return drag_sectional
    def dD_induced(self, azimuth, r):
        self.AoA_sectional(azimuth, r) # AoA is not needed, don't panic about lift slope.
        return self.dL(azimuth,r)*self.U_P/self.U_T
    def dT(self, azimuth, r, dsi,dr):
        return self.dL(azimuth,r)* dsi*dr
    
    def T(self):
        if not self.beta_0:
            self.coning_angle()
        T = 0
        si = np.linspace(0, 2*np.pi, 360)
        d_si = si[1]-si[0]
        r = np.linspace(self.R_co, self.R, 25)
        dr = r[1]- r[0]
        for azim in si:
            for radius in r:
                T += self.dL(azim, radius, self.beta_0)*dr*d_si*np.cos(self.beta_0)
        T = T*self.b/(2*np.pi)
        return T
    def Torque(self):
        Q = 0
        si = np.linspace(0, 2*np.pi, 360)
        d_si = si[1]-si[0]
        r = np.linspace(self.R_co, self.R, 25)
        dr = r[1]- r[0]
        for azim in si:
            for radius in r:
                Q+= self.dD(azim, radius)*dr*d_si
        Q = Q*self.b/(2*np.pi)
        return Q
    def Torque_i(self):
        Q = 0
        si = np.linspace(0, 2*np.pi, 360)
        d_si = si[1]-si[0]
        r = np.linspace(self.R_co, self.R, 25)
        dr = r[1]- r[0]
        for azim in si:
            for radius in r:
                Q += self.dD_induced(azim, radius)*dr*d_si
        Q = Q*self.b/(2*np.pi)
        return Q

    def PitchMomentHub(self):
        beta = self.coning_angle()
        M_p = 0
        si = np.linspace(0, 2*np.pi, 360)
        d_si = si[1]-si[0]
        r = np.linspace(self.R_co, self.R, 25)
        dr = r[1]- r[0]
        for azim in si:
            for radius in r:
                M_p += self.dL(azim, radius, beta)*radius*np.cos(azim)*dr*d_si
        M_p = M_p*self.b/(2*np.pi)
        self.M_p = M_p
        return self.M_p
    
    def RollMomentHub(self):
        beta = self.coning_angle()
        M_r = 0
        si = np.linspace(0, 2*np.pi, 360)
        d_si = si[1]-si[0]
        r = np.linspace(self.R_co, self.R, 25)
        dr = r[1]- r[0]
        for azim in si:
            for radius in r:
                M_r += self.dL(azim, radius, beta)*radius*np.sin(azim)*dr*d_si
        M_r = M_r*self.b/(2*np.pi)
        self.M_r = M_r
        return self.M_r
    def dT_climb(self, azimuth):
        induced_vel = np.sqrt(self.W/(2*self.ro*self.A))
        lamda = (self.climb_vel + induced_vel)/(self.omega_climb*self.R)
        azimuthal = np.linspace(0, 2*np.pi, 360)
        dsi = azimuthal[1] - azimuthal[0]
        si = azimuth
        tita = self.tita_0_climb +self.tita_1c_climb*np.cos(si) + self.tita_1s_climb*np.sin(si) + self.linear_twist*0.75
        dT = 0.5*self.b*self.c*self.a*self.ro*((self.omega_climb*self.R)**2)*self.R*((1/3)*tita - lamda/2)*dsi
        return dT
    def T_climb(self):
        induced_vel = np.sqrt(self.W/(2*self.ro*self.A))
        lamda = (self.climb_vel + induced_vel)/(self.omega_climb*self.R)
        
        azimuthal = np.linspace(0, 2*np.pi, 360)
        dsi = azimuthal[1] - azimuthal[0]
        T =0
        for si in azimuthal:
            tita = self.tita_0_climb +self.tita_1c_climb*np.cos(si) + self.tita_1s_climb*np.sin(si) + self.linear_twist*0.75
            T += 0.5*self.b*self.c*self.a*self.ro*((self.omega_climb*self.R)**2)*self.R*((1/3)*tita - lamda/2)*dsi
        T = T/(2*np.pi)
        self.thrust = round(T,5)
        return self.thrust
    def Roll_climb(self):
        induced_vel = np.sqrt(self.W/(2*self.ro*self.A))
        lamda = (self.climb_vel + induced_vel)/(self.omega_climb*self.R)
        azimuthal = np.linspace(0, 2*np.pi, 360)
        radii = np.linspace(self.R_co, self.R, 20)
        dsi = azimuthal[1] - azimuthal[0] ; dr = radii[1] - radii[0]
        R =0
        for si in azimuthal:
            for r in radii:
                tita = self.tita_0_climb +self.tita_1c_climb*np.cos(si) + self.tita_1s_climb*np.sin(si) + self.linear_twist*0.75
                R += 0.5*self.b*self.c*self.a*self.ro*((self.omega_climb*self.R)**2)*self.R*((1/3)*tita - lamda/2)*dsi*dr*r*np.sin(si)
        R = R/(2*np.pi)
        return round(R,5)
    def Pitch_climb(self):
        induced_vel = np.sqrt(self.W/(2*self.ro*self.A))
        lamda = (self.climb_vel + induced_vel)/(self.omega_climb*self.R)
        azimuthal = np.linspace(0, 2*np.pi, 360)
        radii = np.linspace(self.R_co, self.R, 20)
        dsi = azimuthal[1] - azimuthal[0] ; dr = radii[1] - radii[0]
        P =0
        for si in azimuthal:
            for r in radii:
                tita = self.tita_0_climb +self.tita_1c_climb*np.cos(si) + self.tita_1s_climb*np.sin(si) + self.linear_twist*0.75
                P += 0.5*self.b*self.c*self.a*self.ro*((self.omega_climb*self.R)**2)*self.R*((1/3)*tita - lamda/2)*dsi*dr*r*np.cos(si)
        P = P/(2*np.pi)
        return round(P,5)

class TailRotor:
    def __init__(self, pitch_angle, forward_vel, no_of_blades, chord, angular_vel, 
                 lift_slope, radius,root_cutouts, medium = 'air') -> None:
        self.omega = angular_vel
        self.R = radius; self.R_co = root_cutouts
        self.a = lift_slope
        self.b = no_of_blades
        self.c= chord
        self.tita = pitch_angle
        self.forward_vel = forward_vel
        
        if medium == 'air':
            self.ro = roAir
        elif medium == 'water':
            self.ro = roWater
        self.A = np.pi*(self.R**2)

    def dL(self, azimuth, r):
        V_infi = self.omega*r + self.forward_vel*np.sin(azimuth)
        if V_infi >0:
            AoA = self.tita
        else:
            AoA = -self.tita
        return 0.5*self.ro*(V_infi**2)*self.c* (AoA*self.a)
    def T(self):
        T = 0
        azimmuth_angles = np.linspace(0, 2*np.pi, 360)
        radii = np.linspace(self.R_co, self.R, 20)
        dsi = azimmuth_angles[1] - azimmuth_angles[0]; dr = radii[1]- radii[0]
        for si in azimmuth_angles:
            for r in radii:
                T += self.dL(si,r)*dsi*dr
        T = T*self.b/(2*np.pi)
        return T
class Plots:
    # This could be written with much more sophistication but for now, I just wanted those values
    # Idea is if u use key word PitchMoment and tita_1c you will get the plot.
    def __init__(self, rotor , default_pitch) -> None:
        self.rotor = rotor
        self.default_pitch = default_pitch
        self.rotor.t_0 , self.rotor.t_1c, self.rotor.t_1s = default_pitch
    def reset(self):
        self.rotor.t_0 , self.rotor.t_1c, self.rotor.t_1s = self.default_pitch
        return
    
    def plot(self, y ,x):
        if y == 'Thrust':
            return self.Thrust(x)
        elif y == 'Torque':
            return self.Torque(x)
        elif y == 'PitchMoment':
            return self.PitchMoment(x)
        elif y == 'RollMoment':
            return self.RollMoment(x)
        else:
            raise NameError('Either Thrust, Torque, PitchMoment, RollMoment')

    def Thrust(self,x):
        self.reset()
        T = []
        if x == 'tita':
            tita = np.linspace(-10*np.pi/180, 30*np.pi/180, 80)
            for collectivePitch in tita:
                self.rotor.t_0 = collectivePitch
                T.append(self.rotor.T())
            return tita, T
        elif x== 'tita_1c':
            tita_1c = np.linspace(-15*np.pi/180, 15*np.pi/180, 60)
            for cycPitch in tita_1c:
                self.rotor.t_1c = cycPitch
                T.append(self.rotor.T())
            return tita_1c, T
        elif x == 'tita_1s':
            tita_1s = np.linspace(-15*np.pi/180, 15*np.pi/180, 60)
            for cycPitch in tita_1s:
                self.rotor.t_1s = cycPitch
                T.append(self.rotor.T())
            return tita_1s, T
        else:
            raise NameError('should either tita, tita_1c or tita_1s')
        
    def Torque(self,x):
        self.reset()
        Q = []
        if x == 'tita':
            tita = np.linspace(-10*np.pi/180, 30*np.pi/180, 80)
            for collectivePitch in tita:
                self.rotor.t_0 = collectivePitch
                Q.append(self.rotor.Torque())
            return tita, Q
        elif x== 'tita_1c':
            tita_1c =np.linspace(-15*np.pi/180, 15*np.pi/180, 60)
            for cycPitch in tita_1c:
                self.rotor.t_1c = cycPitch
                Q.append(self.rotor.Torque())
            return tita_1c,Q
        elif x == 'tita_1s':
            tita_1s = np.linspace(-15*np.pi/180, 15*np.pi/180, 60)
            for cycPitch in tita_1s:
                self.rotor.t_1s = cycPitch
                Q.append(self.rotor.Torque())
            return tita_1s, Q
        else:
            raise NameError('should either tita, tita_1c or tita_1s')
        
    def PitchMoment(self,x):
        self.reset()
        pitchMoment = []
        if x == 'tita':
            tita = np.linspace(-10*np.pi/180, 30*np.pi/180, 80)
            for collectivePitch in tita:
                self.rotor.t_0 = collectivePitch
                pitchMoment.append(self.rotor.PitchMomentHub())
            return tita, pitchMoment
        elif x== 'tita_1c':
            tita_1c =np.linspace(-15*np.pi/180, 15*np.pi/180, 60)
            for cycPitch in tita_1c:
                self.rotor.t_1c = cycPitch
                pitchMoment.append(self.rotor.PitchMomentHub())
            return tita_1c,pitchMoment
        elif x == 'tita_1s':
            tita_1s = np.linspace(-15*np.pi/180, 15*np.pi/180, 60)
            for cycPitch in tita_1s:
                self.rotor.t_1s = cycPitch
                pitchMoment.append(self.rotor.PitchMomentHub())
            return tita_1s, pitchMoment
        else:
            raise NameError('should either tita, tita_1c or tita_1s')
        
    def RollMoment(self,x):
        self.reset()
        rollMom = []
        if x == 'tita':
            tita = np.linspace(-10*np.pi/180, 30*np.pi/180, 80)
            for collectivePitch in tita:
                self.rotor.t_0 = collectivePitch
                rollMom.append(self.rotor.RollMomentHub())
            return tita, rollMom
        elif x== 'tita_1c':
            tita_1c = np.linspace(-15*np.pi/180, 15*np.pi/180, 60)
            for cycPitch in tita_1c:
                self.rotor.t_1c = cycPitch
                rollMom.append(self.rotor.RollMomentHub())
            return tita_1c,rollMom
        elif x == 'tita_1s':
            tita_1s = np.linspace(-15*np.pi/180, 15*np.pi/180, 60)
            for cycPitch in tita_1s:
                self.rotor.t_1s = cycPitch
                rollMom.append(self.rotor.RollMomentHub())
            return tita_1s, rollMom
        else:
            raise NameError('should either tita, tita_1c or tita_1s')

class Climb:
    def __init__(self, no_of_blades, chord, angular_vel, lift_slope, drag_coeff, 
                 pitch_climb, twist, climb_vel, radius,  root_cutouts , medium = 'air') -> None:
        self.linear_twist = twist
        self.tita_0_climb, self.tita_1c_climb, self.tita_1s_climb = pitch_climb
        self.c = chord
        self.root_cutouts = root_cutouts
        self.climb_vel    = climb_vel
        self.omega = angular_vel; self.R = radius; self.R_co = root_cutouts
        self.a = lift_slope
        self.b = no_of_blades
        self.c_d = drag_coeff
        if medium == 'air':
            self.ro = roAir
        elif medium == 'water':
            self.ro = roWater
    
    def calcLamda(self, r_ratio, F, azimuth):
        l_c = self.climb_vel/(self.omega*self.R)
        sigma = self.b*self.c/(self.omega*self.R)
        self.sigma = sigma
        self.F = F
        tita = self.tita_0_climb + self.tita_1c_climb*np.cos(azimuth) + self.tita_1s_climb*np.sin(azimuth)
        lamda = np.sqrt( ((self.sigma*self.a/(16*self.F) - l_c/2)**2)+ (self.sigma*self.a*(tita - self.linear_twist*r_ratio)*r_ratio/8) ) - (self.sigma*self.a/(16*self.F) - l_c/2)
        self.lamda = round(lamda, 5)
        return self.lamda
    
    def tip_loss(self, r_ratio, lamda):
        f = self.b*(1- r_ratio)/(2*lamda)
        F = (2/np.pi)*np.arccos(np.exp(-1*f))
        self.F = round(F,5)
        return self.F
    # With a initialization of F = 0.9 we calucalte lamda_1 and with lamda_1 we calculate F_2 then later lamda_2..... To converge lamda, F that's what this below function is doing
    # We decide convergence if prev and current value diffrence is less than 1e-4
    def lamda_tipLoss(self,r_ratio, azimuth):
        fs = [0.9]
        lamdas = [self.calcLamda(r_ratio, 0.9, azimuth)]
        for _ in range(10):
            F = self.tip_loss(r_ratio, lamdas[-1])
            lamda = self.calcLamda(r_ratio, F, azimuth)
            lamdas.append(lamda); fs.append(F)
            if abs(lamda-lamdas[-2])<1e-4 and abs(F-fs[-2])<1e-4:
                break
        self.F = fs[-1]
        self.lamda = lamdas[-1]
        return self.lamda, self.F
    
    def T_climb(self):
        T = 0
        azimuthal = np.linspace(0, 2*np.pi, 360)
        dsi = azimuthal[1] - azimuthal[0]
        radius = np.linspace(self.R_co, self.R, 25)
        dr = radius[1] - radius[0]
        for si in azimuthal:
            for r in radius:
                r_ratio = r/self.R
                self.dr = dr
                lamda , F = self.lamda_tipLoss(r_ratio, si)
                tita = self.tita_0_climb + self.tita_1c_climb*np.cos(si) + self.tita_1s_climb*np.sin(si)
                dT = 0.5*self.ro*self.a*self.b*self.c*((lamda*self.omega*self.R)**2 + (self.omega*r)**2)*(self.sigma*self.a*(tita - self.linear_twist*r_ratio) - lamda/r_ratio)*dr
                T += dT
        return T
    def AoA_climb (self, r_ratio, azimuth):
        lamda , F = self.lamda_tipLoss(r_ratio, azimuth)
        phi = lamda/r_ratio
        AoA = self.linear_twist(r_ratio) - phi
        return AoA

