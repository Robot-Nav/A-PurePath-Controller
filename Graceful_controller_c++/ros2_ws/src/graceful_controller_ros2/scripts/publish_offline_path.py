#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point, Quaternion
import numpy as np
import math


class OfflinePathPublisher(Node):
    def __init__(self):
        super().__init__('offline_path_publisher')
        
        self.path_pub = self.create_publisher(Path, 'plan', 10)
        
        self.declare_parameter('path_type', 'straight')
        self.declare_parameter('path_length', 5.0)
        self.declare_parameter('path_points', 50)
        self.declare_parameter('start_x', 0.0)
        self.declare_parameter('start_y', 0.0)
        self.declare_parameter('start_yaw', 0.0)
        self.declare_parameter('end_x', 5.0)
        self.declare_parameter('end_y', 0.0)
        self.declare_parameter('end_yaw', 0.0)
        self.declare_parameter('circle_radius', 2.0)
        self.declare_parameter('circle_center_x', 2.0)
        self.declare_parameter('circle_center_y', 2.0)
        self.declare_parameter('sine_amplitude', 1.0)
        self.declare_parameter('sine_frequency', 0.5)
        self.declare_parameter('publish_rate', 1.0)
        self.declare_parameter('repeat_publish', False)
        
        path_type = self.get_parameter('path_type').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.repeat_publish = self.get_parameter('repeat_publish').value
        
        path = self.generate_path(path_type)
        
        if path:
            self.path_pub.publish(path)
            self.get_logger().info(f'Published {path_type} path with {len(path.poses)} poses')
            
            if self.repeat_publish:
                self.timer = self.create_timer(
                    1.0 / self.publish_rate,
                    lambda: self.path_pub.publish(path)
                )
                self.get_logger().info(f'Repeating path publication at {self.publish_rate} Hz')
        else:
            self.get_logger().error('Failed to generate path')
    
    def generate_path(self, path_type):
        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp = self.get_clock().now().to_msg()
        
        if path_type == 'straight':
            path = self.generate_straight_path()
        elif path_type == 'circle':
            path = self.generate_circle_path()
        elif path_type == 'sine':
            path = self.generate_sine_path()
        elif path_type == 'custom':
            path = self.generate_custom_path()
        else:
            self.get_logger().error(f'Unknown path type: {path_type}')
            return None
        
        return path
    
    def generate_straight_path(self):
        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp = self.get_clock().now().to_msg()
        
        start_x = self.get_parameter('start_x').value
        start_y = self.get_parameter('start_y').value
        start_yaw = self.get_parameter('start_yaw').value
        end_x = self.get_parameter('end_x').value
        end_y = self.get_parameter('end_y').value
        end_yaw = self.get_parameter('end_yaw').value
        num_points = self.get_parameter('path_points').value
        
        for i in range(num_points):
            t = i / (num_points - 1) if num_points > 1 else 0
            
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.get_clock().now().to_msg()
            
            pose.pose.position.x = start_x + (end_x - start_x) * t
            pose.pose.position.y = start_y + (end_y - start_y) * t
            pose.pose.position.z = 0.0
            
            yaw = start_yaw + (end_yaw - start_yaw) * t
            pose.pose.orientation = self.yaw_to_quaternion(yaw)
            
            path.poses.append(pose)
        
        return path
    
    def generate_circle_path(self):
        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp = self.get_clock().now().to_msg()
        
        center_x = self.get_parameter('circle_center_x').value
        center_y = self.get_parameter('circle_center_y').value
        radius = self.get_parameter('circle_radius').value
        num_points = self.get_parameter('path_points').value
        
        for i in range(num_points):
            angle = 2 * math.pi * i / num_points
            
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.get_clock().now().to_msg()
            
            pose.pose.position.x = center_x + radius * math.cos(angle)
            pose.pose.position.y = center_y + radius * math.sin(angle)
            pose.pose.position.z = 0.0
            
            tangent_angle = angle + math.pi / 2
            pose.pose.orientation = self.yaw_to_quaternion(tangent_angle)
            
            path.poses.append(pose)
        
        return path
    
    def generate_sine_path(self):
        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp = self.get_clock().now().to_msg()
        
        start_x = self.get_parameter('start_x').value
        start_y = self.get_parameter('start_y').value
        path_length = self.get_parameter('path_length').value
        amplitude = self.get_parameter('sine_amplitude').value
        frequency = self.get_parameter('sine_frequency').value
        num_points = self.get_parameter('path_points').value
        
        for i in range(num_points):
            t = i / (num_points - 1) if num_points > 1 else 0
            x = start_x + path_length * t
            y = start_y + amplitude * math.sin(2 * math.pi * frequency * t)
            
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.get_clock().now().to_msg()
            
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            
            dy_dx = amplitude * 2 * math.pi * frequency * math.cos(2 * math.pi * frequency * t)
            yaw = math.atan2(dy_dx, 1.0)
            pose.pose.orientation = self.yaw_to_quaternion(yaw)
            
            path.poses.append(pose)
        
        return path
    
    def generate_custom_path(self):
        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp = self.get_clock().now().to_msg()
        
        waypoints = [
            (0.0, 0.0, 0.0),
            (1.0, 0.0, 0.0),
            (2.0, 0.5, math.pi/6),
            (3.0, 1.0, math.pi/4),
            (4.0, 1.5, math.pi/3),
            (5.0, 2.0, math.pi/2),
        ]
        
        for i, (x, y, yaw) in enumerate(waypoints):
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.get_clock().now().to_msg()
            
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            pose.pose.orientation = self.yaw_to_quaternion(yaw)
            
            path.poses.append(pose)
        
        return path
    
    def yaw_to_quaternion(self, yaw):
        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(yaw / 2.0)
        q.w = math.cos(yaw / 2.0)
        return q


def main(args=None):
    rclpy.init(args=args)
    
    path_publisher = OfflinePathPublisher()
    
    try:
        rclpy.spin(path_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        path_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
