import time

class PID:
    def __init__(self):
        self.P = 0.0
        self.I = 0.0
        self.D = 0.0
        self.max_state = 0.0
        self.min_state = 0.0
        self.pre_state = 0.0
        self.dt = 0.0
        self.integrated_state = 0.0
        self.pre_time = time.time()
        
    def update(self, state):
        self.dt = time.time() - self.pre_time

        if self.dt == 0.:
            state_D = 0.
        else:
            state_D = (state - self.pre_state) / self.dt

        state_I = state + self.integrated_state

        out = self.P*state + self.D*state_D + self.I*state_I * self.dt

        if abs(out) > self.max_state:
            out = self.max_state if out > 0 else -self.max_state
        elif abs(out) < self.min_state:
            out = self.min_state if out > 0 else -self.min_state

        self.pre_state = state
        self.integrated_state = state_I
        self.pre_time = time.time()

        return out