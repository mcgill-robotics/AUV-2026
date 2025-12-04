import numpy as np


# GLOBAL PARAMETERS
TIME_STEP = 0.005 # [s]
MAX_THRUST_X = 10.0 # [N]
MAX_THRUST_Y = 10.0 # [N]
MAX_THRUST_Z = 10.0 # [N]
# will need original state values for position and velocity from subscribed topics


# PID GAINS FOR X-AXIS
KP_X = 1.0
KD_X = 1.0
KI_X = 1.0


# PID GAINS FOR Y-AXIS
KP_Y = 1.0
KD_Y = 1.0
KI_Y = 1.0


# PID GAINS FOR Z-AXIS
KP_Z = 1.0
KD_Z = 1.0
KI_Z = 1.0

        
class PID_z(Node):
        def __init__(self, KP_z, KD_z, KI_z, setpoint_z):
                self.KP_z = KP_z
                self.KD_z = KD_z
                self.KI_z = KI_z

                self.setpoint_z = setpoint_z

                self.position_error_z = 0.0
                self.integral_error_z = 0.0
                self.previous_error_z = 0.0
                self.derivative_error_z = 0.0
                
                self.output_z = 0.0

                # Filter constant and anti-windup clamp
                self.tau = 0.1
                self.I_MAX = 20.0 # [m s]
        
        
        def compute_errors_z(self, pos_z):
                self.position_error_z = self.setpoint_z - pos_z
                self.integral_error_z += self.position_error_z * TIME_STEP
                self.integral_error_z = np.clip(self.integral_error_z, -self.I_MAX, self.I_MAX)
                raw_derivative = (self.position_error_z - self.previous_error_z) / TIME_STEP
                self.derivative_error_z = (
                        self.tau * self.derivative_error_z +
                        (1 - self.tau) * raw_derivative
                        )
                self.previous_error_z = self.position_error_z


        def compute_output_z(self):
                self.output_z = ((self.KP_z * self.position_error_z) + (self.KI_z * self.integral_error_z)
                                + (self.KD_z * self.derivative_error_z))
                self.output_z = np.clip(self.output_z, -MAX_THRUST_Z, MAX_THRUST_Z)
                return self.output_z