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

class Rocket:

    def __init__(self):
        self.mass_rates = []
        self.exit_velocity = []
        self.exit_pressure = []
        self.times = []
        self.CD_A = 0
        self.exit_area = 0
        self.mass_full = 0
        self.atm = Atmosphere()
        self.t_idx = 0
        self.state = np.array([0,0,0]) # alt, vel, mass

    def get_state_rate(self,state,time):
        state_rate = [state[1], 0, 0]
        thrust = 0

        self.atm.set_alt(state[0])
        
        if state[2] > mass_full:
            while time > self.times[self.t_idx + 1]:
                self.t_idx += 1

            state_rate[2] = -self.mass_rates[self.t_idx]

            thrust = -state_rate[2]*self.exit_velocity[self.t_idx] + self.exit_area*(self.exit_pressure(self.t_idx) - self.atm.pres)

        drag = self.CD_A*self.atm.dens*state[1]*abs(state[1])

        mach = abs(state[1])/self.atm.sound_speed

        if (mach > 0.7):
            if mach > 3:
                drag *= 1.5
            elif mach > 1:
                drag *= (mach - 1)*-0.5 + 2.5
            else:
                drag *= (mach - 0.7)*5 + 1

        state_rate[1] = (thrust - drag)/state[2] - 9.806

        return state_rate
