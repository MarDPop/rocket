import numpy as np

class Atmosphere:

    def __init__(self):
        self.pres = 0
        self.dens = 0
        self.sound_speed = 0

        self.pres_table = []
        self.dens_table = []
        self.a_table = []

    def set_alt(self,alt):
        idx = int(alt)
        delta = alt - idx
        idx1 = idx+1
        self.pres = self.pres_table[idx] + delta*(self.pres_table[idx1] - self.pres_table[idx])
        self.dens = self.dens_table[idx] + delta*(self.dens_table[idx1] - self.dens_table[idx])
        self.sound_speed = self.sound_speed[idx] + delta*(self.sound_speed[idx1] - self.sound_speed[idx])

class Motor:

    def __init__(self):
        self.exit_radius = 1
        self.throat_radius = 1
        self.chamber_radius = 1
        self.chamber_length = 1
        self.nozzle_factor = 1
        self.fuel_density = 1000 #kg / m3
        self.fuel_burn_rate0 = 0.0038 # m/s at 1bar
        self.fuel_burn_rate_exp = 0.31 # approx for knsu
        self.fuel_gamma = 1.055
        self.fuel_MW = 0.041
        self.mass_rates = []
        self.exit_velocity = []
        self.exit_pressure = []
        self.times = []
        self.fuel_mass = 0

    # pressure in bar
    def burn_rate(self,pressure): 
        return self.fuel_burn_rate0*(pressure**self.burn_rate_exp)

    @staticmethod
    def aratio(mach,gamma):
        ex = (gamma+1)*0.5/(gamma-1)
        num = 2/(gamma+1)*(1 + 0.5*(gamma-1)*mach*mach)
        return (num**ex) / mach

    @staticmethod
    def mach_aratio(aratio,gamma):
        M_lo = 1.001
        M_hi = 10
        M = (M_lo + M_hi)*0.5
        for i in range(10):
            a = Motor.aratio(M,gamma)
            if a > aratio:
                M_hi = M
            else:
                M_lo = M

            M = (M_lo + M_hi)*0.5

        return M
            

    def compute(self,DT):
        t = 0

        Astar = np.pi*self.throat_radius**2
        R = 8.31445/self.fuel_MW
        T0 = 1300 #guess chamber temp
        rburn = chamber_radius*0.3
        v0 = np.pi*rburn**2
        p0 = 1e5
        rho0 = p0/(R*T0)
        Aburn = 2*np.pi*rburn*chamber_length
        const_val = Astar*np.sqrt(self.fuel_gamma/(R*T0) * (2/(self.gamma+1))**((self.gamma+1)/(self.gamma-1)))

        A_ratio = (self.exit_radius/self.throat_radius)**2
        exit_mach = Motor.mach_aratio(A_ratio,self.fuel_gamma)

        exit_beta = 1 + 0.5*(self.fuel_gamma-1)*exit_mach**2

        supersonic_pressure_ratio = exit_beta**(self.fuel_gamma/(self.fuel_gamma-1))
        exit_temp = T0/exit_beta
        exit_speed = exit_mach*np.sqrt(self.fuel_gamma*R*exit_temp)

        self.exit_pressure = [1e5]
        self.exit_velocity = [0]
        self.mass_rates = [0]
        self.times = [0]

        t_record = DT
        dt = DT/10
        while t < 10000:
            burn_rate = self.burn_rate(p0*1e-5)

            if(t > t_record):
                self.exit_pressure = 
            t+=dt

class Rocket:

    def __init__(self):
        self.mass_rates = np.array([])
        self.exit_velocity = np.array([])
        self.exit_pressure = np.array([])
        self.times = np.array([])
        self.ndata = 0
        self.CD_A = 0
        self.exit_area = 0
        self.mass_full = 0
        self.atm = Atmosphere()
        self.t_idx = 0
        self.state = np.array([0,0,0]) # alt, vel, mass
        self.state_rate = np.array([0,0,0])

    def set_motor(self,motor):
        self.mass_rates = np.array(motor.mass_rates)
        self.exit_velocity = np.array(motor.exit_velocity)
        self.exit_pressure = np.array(motor.exit_pressure)
        self.times = np.array(motor.times)
        self.ndata = len(motor.times) - 1

    def get_state_rate(self,state,time):
        self.state_rate[0] = state[1]
        thrust = 0

        self.atm.set_alt(state[0])
        
        if state[2] > self.mass_full:
            while self.t_idx < self.ndata and time > self.times[self.t_idx + 1]:
                self.t_idx += 1

            self.state_rate[2] = -self.mass_rates[self.t_idx]
            pressure_force = self.exit_area*(self.exit_pressure[self.t_idx] - self.atm.pres)
            thrust = -self.state_rate[2]*self.exit_velocity[self.t_idx] + pressure_force
            
        drag = self.CD_A*self.atm.dens*state[1]*abs(state[1])
        mach = abs(state[1])/self.atm.sound_speed

        if (mach > 0.7):
            if mach > 3:
                drag *= 1.5
            elif mach > 1:
                drag *= (mach - 1)*-0.5 + 2.5
            else:
                drag *= (mach - 0.7)*5 + 1

        self.state_rate[1] = (thrust - drag)/state[2] - 9.806

        return self.state_rate
