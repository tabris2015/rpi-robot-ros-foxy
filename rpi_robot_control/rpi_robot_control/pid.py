import math


class PidController:
    def __init__(self, kp: float, ki: float, kd: float, sample_time:float, is_angle: bool):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.sample_time = sample_time
        self.is_angle = is_angle
        self.out_min = 0
        self.out_max = 1
        self.output = 0
        self.out_sum = 0
        self.input = 0
        self.setpoint = 0
        self.error = 0
        self.last_input = 0
        self.set_output_limits(-1.0, 1.0)
        self.set_gains(self.kp, self.ki, self.kd)

    def set_sample_time(self, new_sample_time):
        if new_sample_time <= 0.0:
            return
        
        ratio = new_sample_time / self.sample_time
        self.ki *= ratio
        self.kd /= ratio
        self.sample_time = new_sample_time

    def set_gains(self, kp, ki, kd):
        if kp < 0 or ki < 0 or kd < 0:
            return

        self.kp = kp
        self.ki = ki * self.sample_time
        self.kd = kd / self.sample_time

    def set_output_limits(self, min, max):
        if min >= max:
            return
        
        self.out_min = min
        self.out_max = max

        if self.output > self.out_max:
            self.output = self.out_max
        elif self.output < self.out_min:
            self.output = self.out_min

        if self.out_sum > self.out_max:
            self.out_sum = self.out_max
        elif self.out_sum < self.out_min:
            self.out_sum = self.out_min

    def compute(self):
        self.error = self.setpoint - self.input
        if self.is_angle:
            self.error = math.atan2(math.sin(self.error), math.cos(self.error))
        
        delta_input = self.input - self.last_input
        self.out_sum += self.ki * self.error

        if self.out_sum > self.out_max:
            self.out_sum = self.out_max
        elif self.out_sum < self.out_min:
            self.out_sum = self.out_min

        self.output = self.kp * self.error + self.out_sum - self.kd * delta_input

        if self.output > self.out_max:
            self.output = self.out_max
        elif self.output < self.out_min:
            self.output = self.out_min

        self.last_input = self.input

    def set_input(self, new_input):
        self.input = new_input
    
    def set_setpoint(self, new_setpoint):
        self.setpoint = new_setpoint

    def get_output(self):
        return self.output