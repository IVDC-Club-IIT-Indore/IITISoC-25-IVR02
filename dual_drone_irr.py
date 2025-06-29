#!/usr/bin/env python3
 
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from sensor_msgs.msg import NavSatFix
import json
import time
import os
from std_msgs.msg import String
from geographic_msgs.msg import GeoPoseStamped
import re
from mavros_msgs.srv import CommandBool, SetMode
from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSProfile

class GeoFollower(Node):
    def __init__(self):
        super().__init__('geo_follower')

        self.namespace = '/drone2'
        qos = QoSProfile(depth=10)

        # Geotag subscriber
        self.geotag_sub = self.create_subscription(
            String,
            '/coverage/geotag',
            self.geotag_callback,
            qos
        )

        # Publisher to PX4 global setpoint
        self.geo_pub = self.create_publisher(
            GeoPoseStamped,
            f'{self.namespace}/mavros/setpoint_position/global',
            qos
        )

        # Services for arming and mode
        self.arming_client = self.create_client(CommandBool, '/drone2/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, '/drone2/set_mode')

        self.waypoint_queue = []
        self.takeoff_done = False
        self.current_waypoint = None

        # Main loop timer
        self.timer = self.create_timer(1.0, self.send_next_waypoint)
        self.get_logger().info("GeoFollower initialized and waiting for geotags...")

    def geotag_callback(self, msg):
        match = re.search(r'lat=([-0-9.]+), lon=([-0-9.]+), alt=([0-9.]+)', msg.data)
        if match:
            lat = float(match.group(1))
            lon = float(match.group(2))
            alt = float(match.group(3))
            self.get_logger().info(f"Received geotag: lat={lat}, lon={lon}, alt={alt}")
            self.waypoint_queue.append((lat, lon, alt))
        else:
            self.get_logger().warn("Invalid geotag format received.")

    def send_next_waypoint(self):
        if not self.takeoff_done and self.waypoint_queue:
            lat, lon, alt = self.waypoint_queue[0]  # Peek first waypoint
            self.get_logger().info("Starting takeoff sequence...")

            # Set mode to OFFBOARD
            if self.set_mode_client.service_is_ready():
                set_mode_req = SetMode.Request()
                set_mode_req.custom_mode = 'OFFBOARD'
                future = self.set_mode_client.call_async(set_mode_req)
                rclpy.spin_until_future_complete(self, future)
                if future.result() and future.result().mode_sent:
                    self.get_logger().info("OFFBOARD mode set")
                else:
                    self.get_logger().error("Failed to set OFFBOARD mode")
                    return

            # Arm the drone
            if self.arming_client.service_is_ready():
                arm_req = CommandBool.Request()
                arm_req.value = True
                future = self.arming_client.call_async(arm_req)
                rclpy.spin_until_future_complete(self, future)
                if future.result() and future.result().success:
                    self.get_logger().info("Drone armed")
                else:
                    self.get_logger().error("Arming failed")
                    return

            self.takeoff_done = True

        # Send next waypoint
        if self.waypoint_queue:
            lat, lon, alt = self.waypoint_queue.pop(0)
            geo_msg = GeoPoseStamped()
            geo_msg.pose.position.latitude = lat
            geo_msg.pose.position.longitude = lon
            geo_msg.pose.position.altitude = alt
            geo_msg.header.frame_id = "map"
            geo_msg.header.stamp = self.get_clock().now().to_msg()

            self.geo_pub.publish(geo_msg)
            self.get_logger().info(f"Sent waypoint: lat={lat}, lon={lon}, alt={alt}")



def main(args=None):
    rclpy.init(args=args)
    node = GeoFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
