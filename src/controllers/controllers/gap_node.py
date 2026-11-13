import math

import numpy as np
import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker


class ReactiveFollowGap(Node):
    """
    Reactive Follow the Gap controller for F1TENTH.

    Steps:
      1. Preprocess the LiDAR scan (clean, clip, smooth) and crop it to the FOV.
      2. Find the closest point and zero out a safety bubble around it.
      3. Find the longest gap of free beams.
      4. Pick the best point inside that gap.
      5. Publish an AckermannDriveStamped message.

    It also counts laps from odometry and stops the car after target_laps laps.
    """

    def __init__(self):
        super().__init__('reactive_node')

        # ---------------- Topics ----------------
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('odom_topic', '/ego_racecar/odom')
        self.declare_parameter('drive_topic', '/drive')

        # ---------------- Perception parameters ----------------
        self.declare_parameter('fov_deg', 180.0)        # total field of view used (deg)
        self.declare_parameter('max_range', 3.0)        # m, longer readings are clipped
        self.declare_parameter('smooth_window', 5)      # beams
        self.declare_parameter('bubble_radius', 0.35)   # m
        self.declare_parameter('gap_threshold', 1.0)    # m, minimum range to count as free

        # ---------------- Control parameters ----------------
        self.declare_parameter('max_steer', 0.4189)     # rad (24 deg)
        self.declare_parameter('steer_alpha', 0.6)      # low-pass filter (0 = no filtering)
        # Dynamic speed profile: the car goes as fast as braking distance
        # and lateral grip allow, up to v_max
        self.declare_parameter('v_max', 8.0)            # m/s, top speed on long straights
        self.declare_parameter('v_min', 2.0)            # m/s, lowest speed in tight corners
        self.declare_parameter('max_decel', 6.0)        # m/s^2, braking used to plan speed
        self.declare_parameter('brake_margin', 0.8)     # m, safety distance kept before a wall
        self.declare_parameter('max_lat_accel', 5.0)    # m/s^2, lateral acceleration limit
        self.declare_parameter('wheelbase', 0.3302)     # m, F1TENTH wheelbase
        self.declare_parameter('lookahead_range', 10.0) # m, range used to measure clear distance ahead
        self.declare_parameter('front_cone_deg', 6.0)   # deg, half-width of the cone around the target

        # ---------------- Lap parameters ----------------
        self.declare_parameter('target_laps', 10)
        self.declare_parameter('stop_when_done', True)
        # Start/finish line, in map coordinates
        self.declare_parameter('start_x', 0.0)          # m, center of the line (map frame)
        self.declare_parameter('start_y', 0.0)          # m, center of the line (map frame)
        self.declare_parameter('start_yaw', 0.0)        # rad, driving direction across the line
        self.declare_parameter('start_line_length', 3.0)  # m, should span the track width
        self.declare_parameter('min_lap_distance', 20.0)  # m, ignores accidental double crossings
        self.declare_parameter('collision_dist', 0.20)  # m, minimum reading treated as a near-collision
        self.declare_parameter('status_period', 1.0)    # s, how often the live status line is printed

        p = self.get_parameter
        self.fov = math.radians(p('fov_deg').value)
        self.max_range = p('max_range').value
        self.smooth_window = int(p('smooth_window').value)
        self.bubble_radius = p('bubble_radius').value
        self.gap_threshold = p('gap_threshold').value
        self.max_steer = p('max_steer').value
        self.steer_alpha = p('steer_alpha').value
        self.v_max = p('v_max').value
        self.v_min = p('v_min').value
        self.max_decel = p('max_decel').value
        self.brake_margin = p('brake_margin').value
        self.max_lat_accel = p('max_lat_accel').value
        self.wheelbase = p('wheelbase').value
        self.lookahead_range = p('lookahead_range').value
        self.front_cone_deg = p('front_cone_deg').value
        self.target_laps = int(p('target_laps').value)
        self.stop_when_done = p('stop_when_done').value
        self.start_x = p('start_x').value
        self.start_y = p('start_y').value
        self.start_yaw = p('start_yaw').value
        self.start_line_length = p('start_line_length').value
        self.min_lap_distance = p('min_lap_distance').value
        self.collision_dist = p('collision_dist').value
        self.status_period = p('status_period').value

        # ---------------- State ----------------
        self.steer = 0.0
        self.fov_mask = None          # beam indices inside the FOV
        self.fov_angles = None        # angles of those beams
        self.angle_increment = None

        self.prev_s = 0.0             # previous position along the driving direction
        self.lap_count = 0
        self.lap_start_time = None
        self.finished = False
        self.near_collision_count = 0
        self.in_collision = False

        # Chronometer and distance tracking
        self.race_start_time = None   # time of the first odometry message
        self.prev_position = None     # last (x, y), used to integrate distance
        self.lap_distance = 0.0       # m traveled in the current lap
        self.total_distance = 0.0     # m traveled in the whole run
        self.current_speed = 0.0      # m/s, from odometry
        self.best_lap_time = None
        self.lap_records = []         # (lap_time, lap_distance) per completed lap

        # ---------------- Subscribe to LIDAR / publish to drive ----------------
        self.drive_pub = self.create_publisher(
            AckermannDriveStamped, p('drive_topic').value, 10)
        self.create_subscription(
            LaserScan, p('scan_topic').value, self.lidar_callback, 10)
        self.create_subscription(
            Odometry, p('odom_topic').value, self.odom_callback, 10)

        # Start line marker for RViz (add a Marker display on /start_line)
        self.marker_pub = self.create_publisher(Marker, '/start_line', 10)
        self.marker_timer = self.create_timer(1.0, self.publish_start_line)

        # Live chronometer / distance status line
        self.status_timer = self.create_timer(self.status_period, self.status_callback)

        self.get_logger().info(
            f'Follow the Gap ready. Target: {self.target_laps} laps.')

    # ======================================================
    #  Odometry: lap counting
    # ======================================================
    def line_coordinates(self, x, y):
        """Express (x, y) in the start-line frame.
        along_track: distance along the driving direction (0 on the line, + after it)
        cross_track: lateral offset along the line (0 at its center)
        """
        dx = x - self.start_x
        dy = y - self.start_y
        along_track = dx * math.cos(self.start_yaw) + dy * math.sin(self.start_yaw)
        cross_track = -dx * math.sin(self.start_yaw) + dy * math.cos(self.start_yaw)
        return along_track, cross_track

    def odom_callback(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        now = self.get_clock().now()
        self.current_speed = msg.twist.twist.linear.x

        along_track, cross_track = self.line_coordinates(x, y)

        # The first odometry message starts the chronometer
        if self.prev_position is None:
            self.prev_position = (x, y)
            self.prev_s = along_track
            self.race_start_time = now
            self.lap_start_time = now
            self.get_logger().info(
                f'Start line at x={self.start_x:.2f}, y={self.start_y:.2f}, '
                f'yaw={self.start_yaw:.2f} rad | car at x={x:.2f}, y={y:.2f}')
            return

        # Integrate the distance traveled between consecutive odometry samples
        if not self.finished:
            step = math.hypot(x - self.prev_position[0], y - self.prev_position[1])
            self.lap_distance += step
            self.total_distance += step
        self.prev_position = (x, y)

        # The line is crossed in the driving direction when the along-track
        # coordinate goes from negative to non-negative inside the line length
        crossed = (self.prev_s < 0.0 <= along_track
                   and abs(cross_track) <= self.start_line_length / 2.0)
        self.prev_s = along_track

        if not crossed or self.finished:
            return

        # Crossing the line before completing a real lap (car starting just
        # behind the line): restart the clocks exactly at the line
        if self.lap_count == 0 and self.lap_distance < self.min_lap_distance:
            self.race_start_time = now
            self.lap_start_time = now
            self.lap_distance = 0.0
            self.total_distance = 0.0
            self.get_logger().info('Start line crossed: timing started.')
            return

        # Ignore accidental double crossings (jitter around the line)
        if self.lap_distance < self.min_lap_distance:
            return

        self.lap_count += 1
        lap_time = (now - self.lap_start_time).nanoseconds * 1e-9
        lap_distance = self.lap_distance
        avg_speed = lap_distance / lap_time if lap_time > 0.0 else 0.0
        self.lap_records.append((lap_time, lap_distance))
        if self.best_lap_time is None or lap_time < self.best_lap_time:
            self.best_lap_time = lap_time
        self.get_logger().info(
            f'LAP {self.lap_count}/{self.target_laps} DONE | '
            f'time: {lap_time:.2f} s | distance: {lap_distance:.2f} m | '
            f'avg speed: {avg_speed:.2f} m/s | best: {self.best_lap_time:.2f} s | '
            f'near-collisions: {self.near_collision_count}')

        # Start the next lap
        self.lap_start_time = now
        self.lap_distance = 0.0
        if self.lap_count >= self.target_laps:
            self.finished = True
            total_time = (now - self.race_start_time).nanoseconds * 1e-9
            self.get_logger().info(
                f'*** {self.target_laps} laps completed | '
                f'total time: {total_time:.2f} s | '
                f'total distance: {self.total_distance:.2f} m | '
                f'best lap: {self.best_lap_time:.2f} s | '
                f'near-collisions: {self.near_collision_count} ***')

    def publish_start_line(self):
        """Publish the start/finish line as a marker (frame 'map') for RViz."""
        half = self.start_line_length / 2.0
        # Unit vector along the line (perpendicular to the driving direction)
        ux, uy = -math.sin(self.start_yaw), math.cos(self.start_yaw)

        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'start_line'
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.08                     # line thickness (m)
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = 0.1, 1.0, 0.2, 1.0
        marker.pose.orientation.w = 1.0
        marker.points = [
            Point(x=self.start_x - half * ux, y=self.start_y - half * uy, z=0.05),
            Point(x=self.start_x + half * ux, y=self.start_y + half * uy, z=0.05),
        ]
        self.marker_pub.publish(marker)

    # ======================================================
    #  Live chronometer
    # ======================================================
    def status_callback(self):
        """Print the running lap time, lap distance and speed once per status_period."""
        if self.finished:
            self.status_timer.cancel()
            return
        if self.prev_position is None:
            return
        now = self.get_clock().now()
        lap_elapsed = (now - self.lap_start_time).nanoseconds * 1e-9
        total_elapsed = (now - self.race_start_time).nanoseconds * 1e-9
        self.get_logger().info(
            f'[Lap {self.lap_count + 1}/{self.target_laps}] '
            f'lap time: {lap_elapsed:6.1f} s | lap distance: {self.lap_distance:7.2f} m | '
            f'speed: {self.current_speed:4.1f} m/s | '
            f'total time: {total_elapsed:6.1f} s | total distance: {self.total_distance:8.2f} m')

    # ======================================================
    #  Follow the Gap
    # ======================================================
    def init_scan_geometry(self, data):
        """Compute once which beams fall inside the field of view."""
        num_beams = len(data.ranges)
        angles = data.angle_min + np.arange(num_beams) * data.angle_increment
        self.fov_mask = np.where(np.abs(angles) <= self.fov / 2.0)[0]
        self.fov_angles = angles[self.fov_mask]
        self.angle_increment = data.angle_increment

    def preprocess_lidar(self, ranges):
        """ Preprocess the LiDAR scan array:
            1. Replace invalid readings (nan / inf) with max_range
            2. Reject high values (clip to max_range)
            3. Set each value to the mean over a sliding window
        """
        proc_ranges = np.array(ranges, dtype=np.float64)
        proc_ranges[~np.isfinite(proc_ranges)] = self.max_range
        proc_ranges = np.clip(proc_ranges, 0.0, self.max_range)
        if self.smooth_window > 1:
            pad = self.smooth_window // 2
            kernel = np.ones(self.smooth_window) / self.smooth_window
            proc_ranges = np.convolve(
                np.pad(proc_ranges, pad, mode='edge'), kernel, mode='valid')
        return proc_ranges

    def find_max_gap(self, free_space_ranges):
        """ Return the start index & end index (both inclusive) of the longest
            run of free beams in free_space_ranges. Free beams are those with
            a range greater than gap_threshold. Returns None if there is none.
        """
        free_beams = free_space_ranges > self.gap_threshold
        best_len, best_start = 0, 0
        current_len, current_start = 0, 0
        for i, is_free in enumerate(free_beams):
            if is_free:
                if current_len == 0:
                    current_start = i
                current_len += 1
                if current_len > best_len:
                    best_len, best_start = current_len, current_start
            else:
                current_len = 0
        if best_len == 0:
            return None
        return best_start, best_start + best_len - 1

    def find_best_point(self, start_i, end_i, ranges):
        """ start_i & end_i are the start and end indices of the max gap.
            Return the index of the best point in ranges.
            Instead of the naive "furthest point", use the centroid of the gap
            weighted by squared range: smoother steering, biased to the deepest area.
        """
        gap_indices = np.arange(start_i, end_i + 1)
        weights = ranges[start_i:end_i + 1] ** 2
        return int(round(np.sum(gap_indices * weights) / np.sum(weights)))

    def lidar_callback(self, data):
        """ Process each LiDAR scan as per the Follow Gap algorithm
            & publish an AckermannDriveStamped message
        """
        if self.fov_mask is None:
            self.init_scan_geometry(data)

        # Near-collision monitoring (uses the full raw scan)
        raw_ranges = np.array(data.ranges, dtype=np.float64)
        raw_ranges[~np.isfinite(raw_ranges)] = self.max_range
        if raw_ranges.min() < self.collision_dist:
            if not self.in_collision:
                self.near_collision_count += 1
                self.in_collision = True
                self.get_logger().warn(
                    f'Near-collision #{self.near_collision_count} '
                    f'(min range = {raw_ranges.min():.2f} m)')
        else:
            self.in_collision = False

        # Goal reached: stop the car
        if self.finished and self.stop_when_done:
            self.publish_drive(0.0, 0.0)
            return

        # Preprocess and crop to the field of view
        proc_ranges = self.preprocess_lidar(data.ranges)[self.fov_mask]
        angles = self.fov_angles

        # Long-range scan (cropped to the FOV), used only to measure the clear
        # distance ahead for speed planning (the gap search keeps using max_range)
        look_ranges = np.array(data.ranges, dtype=np.float64)
        look_ranges[~np.isfinite(look_ranges)] = self.lookahead_range
        look_ranges = np.clip(look_ranges, 0.0, self.lookahead_range)[self.fov_mask]

        # Find closest point to LiDAR
        closest_idx = int(np.argmin(proc_ranges))
        closest_dist = max(float(proc_ranges[closest_idx]), 0.05)

        # Eliminate all points inside the 'bubble' (set them to zero)
        bubble_half_width = int(
            math.atan2(self.bubble_radius, closest_dist) / self.angle_increment)
        bubble_start = max(0, closest_idx - bubble_half_width)
        bubble_end = min(len(proc_ranges), closest_idx + bubble_half_width + 1)
        proc_ranges[bubble_start:bubble_end] = 0.0

        # Find max length gap
        gap = self.find_max_gap(proc_ranges)

        # Find the best point in the gap
        if gap is None:
            # No free gap: head towards the farthest point
            best_idx = int(np.argmax(proc_ranges))
        else:
            best_idx = self.find_best_point(gap[0], gap[1], proc_ranges)

        target_angle = float(angles[best_idx])

        # Clipped and low-pass filtered steering
        raw_steer = float(np.clip(target_angle, -self.max_steer, self.max_steer))
        self.steer = (self.steer_alpha * self.steer
                      + (1.0 - self.steer_alpha) * raw_steer)

        # Clear distance along the chosen heading (small cone around the target beam)
        cone_half_width = max(
            1, int(math.radians(self.front_cone_deg) / self.angle_increment))
        cone_start = max(0, best_idx - cone_half_width)
        cone_end = min(len(look_ranges), best_idx + cone_half_width + 1)
        clear_dist = float(np.min(look_ranges[cone_start:cone_end]))

        # Speed limit 1: braking distance. Fast on long straights, slows down
        # early enough before a wall:  v^2 = 2 * a * (distance - margin)
        v_brake = math.sqrt(
            2.0 * self.max_decel * max(clear_dist - self.brake_margin, 0.0))

        # Speed limit 2: lateral grip. Curvature from the steering angle
        # (bicycle model) and  v^2 = a_lat / curvature
        curvature = abs(math.tan(self.steer)) / self.wheelbase
        v_curve = math.sqrt(self.max_lat_accel / max(curvature, 1e-3))

        speed = float(np.clip(min(v_brake, v_curve, self.v_max),
                              self.v_min, self.v_max))

        # Publish Drive message
        self.publish_drive(self.steer, speed)

    def publish_drive(self, steering_angle, speed):
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = self.get_clock().now().to_msg()
        drive_msg.header.frame_id = 'base_link'
        drive_msg.drive.steering_angle = float(steering_angle)
        drive_msg.drive.speed = float(speed)
        self.drive_pub.publish(drive_msg)


def main(args=None):
    rclpy.init(args=args)
    print("ReactiveFollowGap Initialized")
    reactive_node = ReactiveFollowGap()
    try:
        rclpy.spin(reactive_node)
    except KeyboardInterrupt:
        pass
    finally:
        reactive_node.publish_drive(0.0, 0.0)
        reactive_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
