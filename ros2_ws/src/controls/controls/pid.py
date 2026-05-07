import numpy as np
import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Wrench

class PID:
        def __init__(self, KP, KD, KI, I_MAX=0.0):
                self.KP = KP
                self.KD = KD
                self.KI = KI
                self.I_MAX = I_MAX



                self.position_error = 0.0
                self.integral_error = 0.0
                self.derivative_error = 0.0

                self.last_setpoint = 0.0
                

        def compute_errors(self, setpoint, position, previous_position, time_step):
                if setpoint != self.last_setpoint: # Reset integral term for a new setpoint
                        self.integral_error = 0.0
                        self.last_setpoint = setpoint

                self.position_error = setpoint - position


                self.integral_error += self.position_error * time_step
                self.integral_error = np.clip(self.integral_error, -self.I_MAX, self.I_MAX)

                # Differentiate position to avoid derivative kick
                self.derivative_error = (position - previous_position) / time_step
                
                self.previous_position = position
                return self.position_error, self.integral_error, self.derivative_error
        
        def compute_effort(self):
                output = ((self.KP * self.position_error) + (self.KI * self.integral_error)
                                - (self.KD * self.derivative_error))
                return output   
        


class CascadedController:
        def __init__(self, max_outer_output, KP_outer, KD_outer, KI_outer, KP_inner, KD_inner, KI_inner):
                self.max_outer_output = max_outer_output
                self.pid_outer = PID(KP_outer, KD_outer, KI_outer)
                self.pid_inner = PID(KP_inner, KD_inner, KI_inner)

        def compute_effort(self, target_position, current_position, current_velocity, dt):
                # Outer loop computes desired velocity
                self.pid_outer.compute_errors(target_position, current_position, self.pid_outer.previous_position, dt)
                desired_velocity = self.pid_outer.compute_effort()
                desired_velocity = np.clip(desired_velocity, -self.max_outer_output, self.max_outer_output)

                # Inner loop computes control effort to achieve desired velocity
                self.pid_inner.compute_errors(desired_velocity, current_velocity, self.pid_inner.previous_position, dt)
                control_effort = self.pid_inner.compute_effort()

                return control_effort
        



