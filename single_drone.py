#!/usr/bin/env python3
 
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from sensor_msgs.msg import NavSatFix
import json
import time
import os
from mavros_msgs.msg import WaypointReached

class WaypointWaterTracker(Node):
    def __init__(self):
        super().__init__('waypoint_water_tracker')

        # Parameters
        self.plan_path = '/home/vimal-pranav/Downloads/mission_with_loiter_5s.plan'
        self.initial_water = 30.0  #in litres
        self.water_decrement = 0.5  
        self.loiter_time = 5.0      

        # Publishers
        self.gps_pub = self.create_publisher(NavSatFix, 'current_waypoint', 10)
        self.water_pub = self.create_publisher(Float32, 'water_level', 10)

        self.waypoints = self.load_waypoints(self.plan_path)
        self.water_level = self.initial_water

        self.current_index = 0
        self.last_wp_seq = -1 

        self.sub_wp = self.create_subscription(
            WaypointReached,
            '/mavros/mission/reached',
            self.waypoint_callback,
            10
        )

        self.get_logger().info("Subscribed to PX4")

    def waypoint_callback(self, msg):
        self.get_logger().info("...")

        current_wp = msg.wp_seq

        if current_wp == self.last_wp_seq:
            return

        self.last_wp_seq = current_wp

        if self.current_index >= len(self.waypoints):
            self.get_logger().info("Mission complete.")
            return

        lat, lon, alt = self.waypoints[self.current_index]
        self.publish_gps(lat, lon, alt)

        self.get_logger().info(
            f"Reached waypoint {msg.wp_seq}: ({lat:.6f}, {lon:.6f}, {alt} m), Water remaining: {self.initial_water:.1f} L"
        )
        self.current_index += 1
        self.initial_water -= self.water_decrement 


    def load_waypoints(self, path):
        with open(path, 'r') as f:
            data = json.load(f)
        wps = []
        for item in data.get("mission", {}).get("items", []):
            if item.get("type") == "SimpleItem" and item.get("command") == 16:
                params = item.get("params", [])
                if len(params) >= 7:
                    lat, lon, alt = params[4], params[5], params[6]
                    wps.append((lat, lon, alt))
        return wps

    def publish_gps(self, lat, lon, alt):
        msg = NavSatFix()
        msg.latitude = float(lat)
        msg.longitude = float(lon)
        msg.altitude = float(alt)
        self.gps_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = WaypointWaterTracker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
