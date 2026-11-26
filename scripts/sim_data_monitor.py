#!/usr/bin/env python3
"""Print the simulation data in the terminal, once per period.

Shows, for the car: position, heading, linear and angular velocity (from
odometry), LiDAR summary (closest obstacle and free distance ahead) and the
last command sent by the controller.

Usage (with the simulator and gap_node running):
    python3 scripts/sim_data_monitor.py
    python3 scripts/sim_data_monitor.py --period 0.5
"""
import argparse
import math

import numpy as np
import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class SimDataMonitor(Node):
    def __init__(self, args):
        super().__init__('sim_data_monitor')
        self.odom = None
        self.scan = None
        self.drive = None
        self.distance = 0.0
        self.prev_xy = None
        self.lap = 1
        self.prev_x = None
        self.dist_since_lap = 0.0
        self.t0 = None
        self.rows = 0
        self.create_subscription(Odometry, args.odom, self.odom_cb, 10)
        self.create_subscription(LaserScan, args.scan, self.scan_cb, 10)
        self.create_subscription(AckermannDriveStamped, args.drive, self.drive_cb, 10)
        self.create_timer(args.period, self.print_row)

    def odom_cb(self, msg):
        x, y = msg.pose.pose.position.x, msg.pose.pose.position.y
        if self.prev_xy is not None:
            step = math.hypot(x - self.prev_xy[0], y - self.prev_xy[1])
            self.distance += step
            self.dist_since_lap += step
        # Lap counter: forward crossing of the start line (x = 0 near y = 0)
        if (self.prev_x is not None and self.prev_x < 0.0 <= x and abs(y) < 1.3
                and self.dist_since_lap > 50.0):
            self.lap += 1
            self.dist_since_lap = 0.0
        self.prev_x = x
        self.prev_xy = (x, y)
        self.odom = msg

    def scan_cb(self, msg):
        self.scan = msg

    def drive_cb(self, msg):
        self.drive = msg

    def print_row(self):
        if self.odom is None:
            self.get_logger().info('Waiting for /ego_racecar/odom ...')
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.t0 is None:
            self.t0 = now
        p = self.odom.pose.pose
        tw = self.odom.twist.twist
        yaw = math.degrees(yaw_from_quaternion(p.orientation))

        lidar_min = lidar_front = float('nan')
        min_angle = float('nan')
        if self.scan is not None:
            r = np.asarray(self.scan.ranges, dtype=float)
            r = np.where(np.isfinite(r), r, self.scan.range_max)
            angles = self.scan.angle_min + np.arange(len(r)) * self.scan.angle_increment
            i = int(np.argmin(r))
            lidar_min, min_angle = r[i], math.degrees(angles[i])
            front = np.abs(angles) <= math.radians(5.0)
            lidar_front = float(np.min(r[front]))

        cmd_v = cmd_steer = float('nan')
        if self.drive is not None:
            cmd_v = self.drive.drive.speed
            cmd_steer = math.degrees(self.drive.drive.steering_angle)

        if self.rows % 20 == 0:
            print(f"{'time':>6} {'lap':>3} | {'x':>7} {'y':>7} {'yaw':>6} | "
                  f"{'v_x':>5} {'v_y':>5} {'w_z':>6} | {'lidar_min':>9} {'@ang':>5} "
                  f"{'front':>6} | {'cmd_v':>5} {'steer':>6} | {'dist':>7}")
            print(f"{'(s)':>6} {'':>3} | {'(m)':>7} {'(m)':>7} {'(deg)':>6} | "
                  f"{'(m/s)':>5} {'(m/s)':>5} {'(r/s)':>6} | {'(m)':>9} {'(deg)':>5} "
                  f"{'(m)':>6} | {'(m/s)':>5} {'(deg)':>6} | {'(m)':>7}")
        print(f"{now - self.t0:6.1f} {self.lap:>3} | {p.position.x:7.2f} {p.position.y:7.2f} "
              f"{yaw:6.1f} | {tw.linear.x:5.2f} {tw.linear.y:5.2f} {tw.angular.z:6.2f} | "
              f"{lidar_min:9.2f} {min_angle:5.0f} {lidar_front:6.2f} | "
              f"{cmd_v:5.2f} {cmd_steer:6.1f} | {self.distance:7.1f}", flush=True)
        self.rows += 1


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--period', type=float, default=1.0, help='seconds between rows')
    parser.add_argument('--odom', default='/ego_racecar/odom')
    parser.add_argument('--scan', default='/scan')
    parser.add_argument('--drive', default='/drive')
    args, ros_args = parser.parse_known_args()
    rclpy.init(args=ros_args)
    node = SimDataMonitor(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
