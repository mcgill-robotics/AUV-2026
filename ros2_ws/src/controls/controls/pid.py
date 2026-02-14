import numpy as np
import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Wrench
# need target position, current position, current velocity for x and y from DVL

# Global Variables
MAX_SPEED = 1.0 # [m/s] max speed for saturation block

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
        
class ControllerNode(Node):
        def __init__(self):
                super().__init__('planar_controller')
                self.declare_parameter('control_loop_hz', 20.0)
                self.control_loop_hz = float(self.get_parameter('control_loop_hz').value)

                self.x_control = PlanarCascadedController(max_speed=MAX_SPEED)
                self.y_control = PlanarCascadedController(max_speed=MAX_SPEED)

                self.effort_pub = self.create_publisher(Wrench, '/controls/planar_effort', 10) # need to subscribe to this topic in superimposer

                self.time_step = 1.0 / self.control_loop_hz
                self.timer = self.create_timer(self.time_step, self.control_loop)

        def control_loop(self):
                dt = self.time_step

                # Get current position and velocity from DVL (placeholder values for now)

                fx = self.x_control.compute_control(target_x, current_x, current_vx, dt) # variables will come from DVL subscriptions
                fy = self.y_control.compute_control(target_y, current_y, current_vy, dt)

                msg = Wrench()
                msg.force.x = fx
                msg.force.y = fy
                msg.force.z, msg.torque.x, msg.torque.y, msg.torque.z = 0, 0, 0, 0

                self.effort_pub.publish(msg)

class PlanarCascadedController:
        def __init__(self, max_speed=MAX_SPEED):
                self.max_speed = max_speed

                # Gains
                self.p_gain_pos = 0.5 # moderate gain for smoothness for position loop
                self.p_gain_vel = 1.0 # high gain for instant reaction for velocity
                self.k_vel = 0.1 # integral gain for velocity

                # State variables
                self.vel_error_integral = 0.0

                def compute_control(self, target_pos, current_pos, current_vel, dt):
                        # 1. OUTER LOOP: position error -> desired velocity
                        pos_error = target_pos - current_pos
                        v_des = self.p_gain_pos * pos_error

                        # 2. SATURATION BLOCK: caps the velocity
                        v_capped = np.clip(v_des, -self.max_speed, self.max_speed,)

                        # 3. INNER LOOP: velocity error -> force/torque
                        vel_error = v_capped - current_vel
                        self.vel_error_integral += vel_error * dt
                        
                        thrust = (self.p_gain_vel * vel_error) + (self.k_vel * self.vel_error_integral) # need to reset to avoid windup error

                        return thrust