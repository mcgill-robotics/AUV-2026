from rclpy.parameter import Parameter
from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_updater import DiagnosticStatusWrapper

from dvl_msgs.msg import DVL
from sensors.health_monitor import HealthMonitor

class DVLVelocityMonitor(HealthMonitor):
    def __init__(self, node):
        super().__init__(node)
        self.node.declare_parameter("dvl_beam_count",Parameter.Type.INTEGER)
        self.node.declare_parameter("velocity_topic", Parameter.Type.STRING)
        
        self.dvl_beam_count = self.node.get_parameter("dvl_beam_count").get_parameter_value().integer_value
        self.velocity_topic = self.node.get_parameter("velocity_topic").get_parameter_value().string_value
        
        self.velocity_sub = self.node.create_subscription(
            DVL,
            self.velocity_topic,
            self.sensor_callback,
            self.qos
        )
        
        self.healthy_vel_status = (DiagnosticStatus.OK, "DVL velocity healthy")
        self.vel_status_msgs:dict = {}
        self.vel_status: tuple[bytes, str] = self.healthy_vel_status

    def check_status(self, stat:DiagnosticStatusWrapper) -> DiagnosticStatusWrapper:
        if self.vel_status_msgs:
            for key, value in self.vel_status_msgs.items():
                stat.add(key, value)
        stat.summary(*self.vel_status)
        return stat
    
    def sensor_callback(self, msg:DVL) -> None:
        self.last_update_time = self.node.get_clock().now()
        if not msg.velocity_valid:
            self.velocity_status = (DiagnosticStatus.WARN, "DVL velocity invalid")
            return
        if msg.status > 0:
            self.vel_status_msgs["Velocity status"] = f"{msg.status}"
            if msg.status == 1:
                self.velocity_status = (DiagnosticStatus.ERROR, "DVL may be overheating")
            else:
                self.velocity_status = (DiagnosticStatus.WARN, f"Invalid DVL velocity status code")
            return
        if msg.altitude < 0:
            self.vel_status_msgs["Velocity altitude"] = f"{msg.altitude}"
            self.velocity_status = (DiagnosticStatus.WARN, "DVL velocity altitude negative")
            return
        valid_beams = 0
        for beam in msg.beams:
            if beam.valid:
                valid_beams += 1
            else:
                self.vel_status_msgs[f"Beam {beam.id}"] = "Invalid"
        if valid_beams < self.dvl_beam_count:
            self.vel_status_msgs["Valid beams"] = f"{valid_beams}/{self.dvl_beam_count}"
            self.velocity_status = (DiagnosticStatus.WARN, "DVL velocity has at least one invalid beam")
            return
        
        self.velocity_status = self.healthy_vel_status
        
            