from abc import ABC, abstractmethod

class Controller(ABC):
    @abstractmethod
    def compute_control(self, state, reference, t, dt):
        pass

class PIDController(Controller):
    def __init__(self, P=1.0, I=0.0, D=0.0):
        self.P = P
        self.I = I
        self.D = D
        self.integral = 0
        self.targets = None
        self.last_error = 0

    def compute_control(self, state, reference, t, dt):
        error = reference - state
        derivative = (error - self.last_error) / dt 
        self.integral += error * dt
        control = self.P * error + self.I * self.integral + self.D * derivative
        self.last_error = error
        return control
    
    def add_target(self, object):
        if self.targets == None:
            self.targets = [object]
        else:
            self.targets.append(object)