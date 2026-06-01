"""
Gazebo Harmonic interface for RL training.
"""

import subprocess
import math
import numpy as np
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.executors import SingleThreadedExecutor

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker, MarkerArray
import sensor_msgs_py.point_cloud2 as pc2

from .utils import process_point_cloud, quaternion_to_euler

from ros_gz_interfaces.srv import ControlWorld
from ros_gz_interfaces.msg import WorldControl
from geometry_msgs.msg import Pose

class GazeboInterface(Node):
    """Interface to control Gazebo Harmonic simulation for RL training."""

    def __init__(self, environment_dim=20):
        super().__init__('gazebo_interface')

        self.environment_dim = environment_dim
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_yaw = 0.0
        self.linear_vel = 0.0
        self.angular_vel = 0.0
        self.velodyne_data = np.ones(environment_dim) * 10.0
        self.last_odom = None
        self.data_lock = threading.Lock()
        self.new_lidar_event = threading.Event()


        self.odom_offset_x = 0.0
        self.odom_offset_y = 0.0
        self.odom_offset_yaw = 0.0


        sensor_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=1)

        self.vel_pub = self.create_publisher(Twist, 'hunter_se/cmd_vel', 10)
        self.goal_pub = self.create_publisher(MarkerArray, 'goal_marker', 3)



        self.odom_sub = self.create_subscription(
            Odometry, 'hunter_se/odom', self.odom_callback, sensor_qos)
        self.pointcloud_sub = self.create_subscription(
            PointCloud2, 'hunter_se/points', self.pointcloud_callback, sensor_qos)

        self.control_client = self.create_client(ControlWorld, '/world/obstacles_easy/control')
        self.control_client.wait_for_service(timeout_sec=5.0)
        self.get_logger().info('Connected to Gazebo control service')


        self.get_logger().info('Gazebo interface initialized')

        self._spin_thread = threading.Thread(target=self._spin_background, daemon=True)
        self._spin_thread.start()

    def _spin_background(self):
        """Background thread to continuously process callbacks."""
        executor = SingleThreadedExecutor()
        executor.add_node(self)
        while rclpy.ok():
            executor.spin_once(timeout_sec=0.01)

    def odom_callback(self, msg):
        with self.data_lock:
            self.last_odom = msg
            self.odom_x = msg.pose.pose.position.x
            self.odom_y = msg.pose.pose.position.y
            q = msg.pose.pose.orientation
            _, _, self.odom_yaw = quaternion_to_euler(q.w, q.x, q.y, q.z)
            self.linear_vel = msg.twist.twist.linear.x
            self.angular_vel = msg.twist.twist.angular.z

    def pointcloud_callback(self, msg):
        try:
            points = list(pc2.read_points(msg, skip_nans=False, field_names=('x', 'y', 'z')))
            with self.data_lock:
                self.velodyne_data = process_point_cloud(points, self.environment_dim)
                self.new_lidar_event.set()
                print(f"LIDAR min: {min(self.velodyne_data):.2f}")
        except Exception as e:
            self.get_logger().warn(f'Error processing point cloud: {e}')

    def publish_cmd_vel(self, linear_x, angular_z):
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        self.vel_pub.publish(msg)

    def stop_robot(self):
        self.publish_cmd_vel(0.0, 0.0)

    def get_robot_state(self):
        with self.data_lock:
            # corrected_yaw = self.odom_yaw + self.odom_offset_yaw
            # while corrected_yaw > math.pi: corrected_yaw -= 2 * math.pi
            # while corrected_yaw < -math.pi: corrected_yaw += 2 * math.pi
            
            return {
                'x': self.odom_x + self.odom_offset_x,
                'y': self.odom_y + self.odom_offset_y,
                'yaw': self.odom_yaw + self.odom_offset_yaw,
                'linear_vel': self.linear_vel,
                'angular_vel': self.angular_vel
            }


    def get_velodyne_data(self, wait_for_fresh=True):
        if wait_for_fresh:
            self.new_lidar_event.clear()  # Reset event
            self.new_lidar_event.wait(timeout=0.5)  # Wait up to 500ms for new data
        with self.data_lock:
            return self.velodyne_data.copy()

    def publish_goal_marker(self, goal_x, goal_y):
        marker_array = MarkerArray()
        marker = Marker()
        marker.header.frame_id = 'odom'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        marker.id = 0
        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.1
        marker.color.a = 1.0
        marker.color.g = 1.0
        marker.pose.position.x = goal_x
        marker.pose.position.y = goal_y
        marker.pose.position.z = 0.05
        marker.pose.orientation.w = 1.0
        marker_array.markers.append(marker)
        self.goal_pub.publish(marker_array)

    # def set_odom_offset(self, world_x, world_y, world_yaw):
    #     """Call after teleporting robot to sync odometry with world frame"""
    #     with self.data_lock:
    #         self.odom_offset_x = world_x - self.odom_x
    #         self.odom_offset_y = world_y - self.odom_y
    #         self.odom_offset_yaw = world_yaw - self.odom_yaw


    
    def _call_control_service(self, req, timeout=2.0):
        """Call ControlWorld service using the background executor."""
        future = self.control_client.call_async(req)
        event = threading.Event()
        future.add_done_callback(lambda f: event.set())
        event.wait(timeout=timeout)

    def pause_simulation(self):
        req = ControlWorld.Request()
        req.world_control.pause = True
        self._call_control_service(req, timeout=1.0)

    def unpause_simulation(self):
        req = ControlWorld.Request()
        req.world_control.pause = False
        self._call_control_service(req, timeout=1.0)

    def reset_world(self):
        req = ControlWorld.Request()
        req.world_control.reset.all = True
        self._call_control_service(req, timeout=2.0)


    
    def set_entity_pose(self, model_name, x, y, z, roll=0.0, pitch=0.0, yaw=0.0):
        cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
        cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
        cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)

        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy

        pose_req = (f'name: "{model_name}", position: {{x: {x}, y: {y}, z: {z}}}, '
                    f'orientation: {{w: {qw}, x: {qx}, y: {qy}, z: {qz}}}')

        try:
            subprocess.run(
                ['gz', 'service', '-s', '/world/obstacles_easy/set_pose',
                 '--reqtype', 'gz.msgs.Pose', '--reptype', 'gz.msgs.Boolean',
                 '--timeout', '1000', '--req', pose_req],
                capture_output=True, timeout=5)
        except Exception as e:
            print(f'Error setting entity pose: {e}')