import numpy as np


class PID:
        def __init__(self, KP, KD, KI, I_MAX=0.0):
                self.KP = KP
                self.KD = KD
                self.KI = KI
                self.I_MAX = I_MAX



                self.position_error = 0.0
                self.integral_error = 0.0
                self.derivative_error = 0.0
                

        def compute_errors(self, setpoint, position, previous_position, time_step):
                self.position_error = setpoint - position


                self.integral_error += self.position_error * time_step
                self.integral_error = np.clip(self.integral_error, -self.I_MAX, self.I_MAX)

                # Differentiate position to avoid derivative kick
                self.derivative_error = (position - previous_position) / time_step
                
                self.previous_position = position
                return self.position_error, self.integral_error, self.derivative_error
        
        def compute_output(self):
                output = ((self.KP * self.position_error) + (self.KI * self.integral_error)
                                - (self.KD * self.derivative_error))
                return output   