import numpy as np

class ODE:

    def __init__(self):
        self.dt = 1
        self.time = 0
        self.function = None
        self.state = []
        self.history = []
        self.record_interval = 0
        self.initial_state = []

    def set_initial_state(self,initial_state):
        self.initial_state = initial_state

    def reset_time(self):
        self.time = 0

    def step(self):
        state_rate = self.function.get_state_rate(self.state,self.time)
        self.state = state*dt

    def run(self,time_final):
        self.state = self.initial_state
        t_record = 0

        while (self.time < time_final):
            self.step()
            self.time += self.dt
