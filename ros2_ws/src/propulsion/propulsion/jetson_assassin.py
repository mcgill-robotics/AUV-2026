#!/usr/bin/env python3
import subprocess
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool


class JetsonAssassin(Node):
    def __init__(self):
        super().__init__('jetson_assassin')
        self.declare_parameter('enable_host_shutdown', True)
        self.enable_host_shutdown = self.get_parameter('enable_host_shutdown').value

        self.sub_kill = self.create_subscription(
            Bool,
            '/power/kill',
            self.kill_callback,
            10
        )
        self.shutdown_triggered = False
        self.get_logger().info('Target acquired. Jetson Assassin standing by on /power/kill...')

    def kill_callback(self, msg: Bool):
        if msg.data and not self.shutdown_triggered:
            self.shutdown_triggered = True
            self.get_logger().fatal('KILL ORDER CONFIRMED (/power/kill=True)! Low battery voltage detected. Terminating Jetson...')

            if not self.enable_host_shutdown:
                self.get_logger().warn('enable_host_shutdown is False. Jetson spared.')
                return

            # Because the container runs with pid: host and privileged: true,
            # PID 1 is the host systemd init process. We enter host namespaces via nsenter
            # to trigger a clean host shutdown.
            commands_to_try = [
                ['nsenter', '-t', '1', '-m', '-u', '-i', '-n', '-p', '--', 'systemctl', 'poweroff'],
                ['nsenter', '-t', '1', '-m', '-u', '-i', '-n', '-p', '--', 'shutdown', '-h', 'now']
            ]

            for cmd in commands_to_try:
                try:
                    self.get_logger().info(f"Executing lethal command: {' '.join(cmd)}")
                    res = subprocess.run(cmd, check=False)
                    if res.returncode == 0:
                        self.get_logger().info('Target neutralized. Jetson shutting down.')
                        return
                except Exception as e:
                    self.get_logger().error(f"Strike failed ({' '.join(cmd)}): {e}")

            # Emergency SysRq fallback if nsenter failed
            try:
                self.get_logger().warn('Fallback: attempting emergency SysRq assassination via /proc/sysrq-trigger')
                with open('/proc/sysrq-trigger', 'w') as f:
                    f.write('o')
            except Exception as e:
                self.get_logger().error(f"Emergency SysRq assassination failed: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = JetsonAssassin()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
