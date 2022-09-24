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
        self.fuel_burn_rate0 = 0.001 # m/s at 1bar
        self.fuel_burn_rate_exp = 0.6
        self.mass_rates = []
        self.exit_velocity = []
        self.exit_pressure = []
        self.times = []
        self.fuel_mass = 0

    # pressure in bar
    def burn_rate(self,pressure): 
        return self.fuel_burn_rate0*(pressure**self.burn_rate_exp)

    def compute(self,dt):
        t = 0
        while t < 10000:

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
