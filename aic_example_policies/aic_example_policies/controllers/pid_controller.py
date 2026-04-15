import numpy as np

class PIDController:
    def __init__(self, kp, ki, kd, max_windup=0.05):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_windup = max_windup
        self.integral = 0.0
        self.prev_error = 0.0
        self.last_error = 0.0 # Added for logging

    def update(self, setpoint, measurement, dt):
        error = setpoint - measurement
        self.last_error = error 

        self.integral = np.clip(
            self.integral + (error * dt), 
            -self.max_windup, 
            self.max_windup
        )
        
        derivative = (error - self.prev_error) / dt
        self.prev_error = error
        
        return (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0
        self.last_error = 0.0