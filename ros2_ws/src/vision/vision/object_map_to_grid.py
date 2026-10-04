#!/usr/bin/env python3

import math

import rclpy
from auv_msgs.msg import VisionObjectArray
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


class ObjectMapToGrid(Node):
    def __init__(self):
        super().__init__("object_map_to_grid")

        self.declare_parameter("object_map_topic", "/vision/object_map")
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("resolution", 0.1)
        self.declare_parameter("x_min", -2.0)
        self.declare_parameter("x_max", 30.0)
        self.declare_parameter("y_min", -25.0)
        self.declare_parameter("y_max", 25.0)
        self.declare_parameter("object_size_labels", [""])
        self.declare_parameter("object_size_x", [0.0])
        self.declare_parameter("object_size_y", [0.0])

        self.object_map_topic = self.get_parameter("object_map_topic").value
        self.map_topic = self.get_parameter("map_topic").value
        self.resolution = self.get_parameter("resolution").value
        self.x_min = self.get_parameter("x_min").value
        self.x_max = self.get_parameter("x_max").value
        self.y_min = self.get_parameter("y_min").value
        self.y_max = self.get_parameter("y_max").value

        labels = self.get_parameter("object_size_labels").value
        sizes_x = self.get_parameter("object_size_x").value
        sizes_y = self.get_parameter("object_size_y").value
        self.fallback_sizes = {
            label: (sizes_x[index], sizes_y[index])
            for index, label in enumerate(labels)
            if label and index < len(sizes_x) and index < len(sizes_y)
        }

        if self.resolution <= 0.0 or self.x_max <= self.x_min or self.y_max <= self.y_min:
            raise ValueError("Map bounds and resolution must define a positive grid")
        self.width = math.ceil((self.x_max - self.x_min) / self.resolution)
        self.height = math.ceil((self.y_max - self.y_min) / self.resolution)

        map_qos = QoSProfile(depth=1)
        map_qos.reliability = ReliabilityPolicy.RELIABLE
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.map_publisher = self.create_publisher(
            OccupancyGrid, self.map_topic, map_qos
        )
        self.object_map_subscription = self.create_subscription(
            VisionObjectArray,
            self.object_map_topic,
            self.object_map_callback,
            10,
        )

        self.get_logger().info(
            f"Converting {self.object_map_topic} to {self.map_topic} "
            f"({self.width}x{self.height} cells at {self.resolution:.2f} m)"
        )

    def object_map_callback(self, object_map):
        time_start = self.get_clock().now()
        grid = OccupancyGrid()
        grid.header.stamp = object_map.header.stamp
        grid.header.frame_id = object_map.header.frame_id or "pool_link"
        grid.info.map_load_time = object_map.header.stamp
        grid.info.resolution = self.resolution
        grid.info.width = self.width
        grid.info.height = self.height
        grid.info.origin.position.x = self.x_min
        grid.info.origin.position.y = self.y_min
        grid.info.origin.orientation.w = 1.0
        grid.data = [0] * (self.width * self.height)

        for obj in object_map.array:
            size_x = obj.size.x
            size_y = obj.size.y
            if size_x <= 0.0 or size_y <= 0.0:
                size_x, size_y = self.fallback_sizes.get(obj.label, (0.0, 0.0))
            if size_x <= 0.0 or size_y <= 0.0:
                continue

            center_x = obj.pose.position.x
            center_y = obj.pose.position.y
            quat = obj.pose.orientation
            yaw = math.atan2(
                2.0 * (quat.w * quat.z + quat.x * quat.y),
                1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z),
            )
            cosine = math.cos(yaw)
            sine = math.sin(yaw)

            # Expand sub-cell footprints enough to guarantee nearby grid cell centers hit.
            half_x = max(size_x / 2.0, self.resolution / math.sqrt(2.0))
            half_y = max(size_y / 2.0, self.resolution / math.sqrt(2.0))
            radius = math.hypot(half_x, half_y)
            min_x = max(
                0,
                math.floor((center_x - radius - self.x_min) / self.resolution),
            )
            max_x = min(
                self.width - 1,
                math.floor((center_x + radius - self.x_min) / self.resolution),
            )
            min_y = max(
                0,
                math.floor((center_y - radius - self.y_min) / self.resolution),
            )
            max_y = min(
                self.height - 1,
                math.floor((center_y + radius - self.y_min) / self.resolution),
            )

            for cell_y in range(min_y, max_y + 1):
                world_y = self.y_min + (cell_y + 0.5) * self.resolution
                for cell_x in range(min_x, max_x + 1):
                    world_x = self.x_min + (cell_x + 0.5) * self.resolution
                    delta_x = world_x - center_x
                    delta_y = world_y - center_y
                    local_x = cosine * delta_x + sine * delta_y
                    local_y = -sine * delta_x + cosine * delta_y
                    if abs(local_x) <= half_x and abs(local_y) <= half_y:
                        grid.data[cell_y * self.width + cell_x] = 100

        time_end = self.get_clock().now()
        self.get_logger().info(
            f"Converted {len(object_map.array)} objects to grid in "
            f"{(time_end - time_start).nanoseconds / 1e6:.2f} ms"
        )
        self.map_publisher.publish(grid)


def main():
    rclpy.init()
    node = ObjectMapToGrid()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
