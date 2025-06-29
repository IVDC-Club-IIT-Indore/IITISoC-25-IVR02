#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from mavros_msgs.msg import WaypointReached
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String
import json

class GeotagPublisher(Node):
    def __init__(self):
        super().__init__('geotag_publisher')

        self.waypoints = self.load_waypoints('mission_with_loiter_5s.plan')
        self.get_logger().info(f"Loaded {len(self.waypoints)} waypoints.")

        # Publishing geotag
        self.geotag_pub = self.create_publisher(String, '/coverage/geotag', 10)

        self.sub_wp = self.create_subscription(
            WaypointReached,
            '/mavros/mission/reached',
            self.reached_callback,
            10
        )

        self.last_wp_seq = -1

    def reached_callback(self, msg):
        self.get_logger().info("...")
        
        wp_seq = msg.wp_seq

        if wp_seq == self.last_wp_seq:
            return 

        self.last_wp_seq = wp_seq

        if wp_seq < len(self.waypoints):
            lat, lon, alt = self.waypoints[wp_seq]
            geotag = f"Waypoint {wp_seq}: lat={lat:.6f}, lon={lon:.6f}, alt={alt:.2f}m"
            self.get_logger().info(f"Reached {geotag}")
            self.geotag_pub.publish(String(data=geotag))
        else:
            self.get_logger().warn(f"Received unknown waypoint index: {wp_seq}")

    def load_waypoints(self, path):
        with open(path, 'r') as f:
            data = json.load(f)
        waypoints = []
        for item in data.get("mission", {}).get("items", []):
            if item.get("type") == "SimpleItem" and item.get("command") == 16:
                params = item.get("params", [])
                if len(params) >= 7:
                    lat, lon, alt = float(params[4]), float(params[5]), float(params[6])
                    waypoints.append((lat, lon, alt))
        return waypoints


def main(args=None):
    rclpy.init(args=args)
    node = GeotagPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
