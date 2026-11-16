"""Follow the Gap controller (disparity extender) for F1TENTH.

Drives the car around the track from the LiDAR scan only, counts laps with
the odometry, keeps each lap close to target_lap_time and, after target_laps
laps, stops the car and prints a summary table with the data of every lap
and the fastest (winning) lap.
"""
import math

import numpy as np
import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class GapFollower:
    """Pure (ROS-free) Follow the Gap logic, so it can also be tested offline.

    1. Clean the scan (nan/inf -> max range).
    2. Disparity extender: at every jump in range (an edge of a wall), the
       closer reading is copied over the beams the car would need to pass that
       edge (half the car width + margin), so the car never cuts corners.
    3. Pick the deepest beam inside the forward field of view.
    4. Steer towards it; never steer into a side wall that is too close.
    5. Speed from braking distance ahead and from the steering angle.
    """

    def __init__(self, **kw):
        self.fov = math.radians(kw.get('fov_deg', 180.0))
        self.max_range = kw.get('max_range', 30.0)
        self.disparity_threshold = kw.get('disparity_threshold', 0.3)
        self.car_half_width = kw.get('car_half_width', 0.155)
        self.safety_margin = kw.get('safety_margin', 0.25)
        self.max_steer = kw.get('max_steer', 0.4189)
        self.steer_gain = kw.get('steer_gain', 1.0)
        self.side_clearance = kw.get('side_clearance', 0.30)
        self.v_min = kw.get('v_min', 2.0)
        self.max_decel = kw.get('max_decel', 6.0)
        self.brake_margin = kw.get('brake_margin', 1.0)
        self.max_lat_accel = kw.get('max_lat_accel', 6.0)
        self.wheelbase = kw.get('wheelbase', 0.3302)
        self.front_cone_deg = kw.get('front_cone_deg', 4.0)
        # Pure-pursuit style steering towards the chosen direction. The aim
        # point moves further away with speed, which damps oscillations when
        # the commands arrive late (ROS latency).
        self.lookahead_min = kw.get('lookahead_min', 1.0)   # m
        self.lookahead_gain = kw.get('lookahead_gain', 0.3)  # s (m per m/s)
        self.lookahead_max = kw.get('lookahead_max', 4.0)   # m
        # Smooth speed changes: sudden braking at high speed makes the car spin
        self.accel_rate = kw.get('accel_rate', 4.0)    # m/s^2, max speed increase
        self.decel_rate = kw.get('decel_rate', 5.0)    # m/s^2, max speed decrease
        self.last_speed = 0.0
        self.angles = None

    def _init_geometry(self, n, angle_min, angle_inc):
        self.angles = angle_min + np.arange(n) * angle_inc
        self.angle_inc = angle_inc
        self.fov_idx = np.where(np.abs(self.angles) <= self.fov / 2.0)[0]
        self.left_idx = np.where((self.angles > math.radians(45)) &
                                 (self.angles < math.radians(135)))[0]
        self.right_idx = np.where((self.angles < -math.radians(45)) &
                                  (self.angles > -math.radians(135)))[0]

    def extend_disparities(self, r):
        out = r.copy()
        half = self.car_half_width + self.safety_margin
        jumps = np.where(np.abs(np.diff(r)) > self.disparity_threshold)[0]
        for i in jumps:
            near, far = (i, i + 1) if r[i] < r[i + 1] else (i + 1, i)
            d = max(r[near], 0.05)
            n_beams = int(math.ceil(math.atan2(half, d) / self.angle_inc))
            step = 1 if far > near else -1
            end = near + step * n_beams
            lo, hi = sorted((near, end))
            lo, hi = max(lo, 0), min(hi, len(r) - 1)
            seg = out[lo:hi + 1]
            np.minimum(seg, d, out=seg)
        return out

    def compute(self, ranges, angle_min, angle_inc, speed_cap, speed=None, dt=0.01):
        if speed is None:
            speed = self.last_speed
        r = np.asarray(ranges, dtype=np.float64)
        if self.angles is None or len(self.angles) != len(r):
            self._init_geometry(len(r), angle_min, angle_inc)
        r = np.where(np.isfinite(r), r, self.max_range)
        r = np.clip(r, 0.0, self.max_range)

        ext = self.extend_disparities(r)
        fov_r = ext[self.fov_idx]
        fov_a = self.angles[self.fov_idx]

        # Deepest direction; among (almost) equally deep beams take the middle
        best = float(np.max(fov_r))
        candidates = np.where(fov_r >= best - 0.05)[0]
        idx = candidates[np.argmin(np.abs(candidates - np.median(candidates)))]
        target = float(fov_a[idx])

        lookahead = float(np.clip(self.lookahead_min + self.lookahead_gain * abs(speed),
                                  self.lookahead_min, self.lookahead_max))
        lookahead = min(lookahead, max(float(fov_r[idx]), self.lookahead_min))
        steer = math.atan2(2.0 * self.wheelbase * math.sin(target), lookahead)
        steer = float(np.clip(self.steer_gain * steer, -self.max_steer, self.max_steer))
        # Do not turn into a wall that is right next to the car
        if steer > 0.0 and np.min(r[self.left_idx]) < self.side_clearance:
            steer = 0.0
        if steer < 0.0 and np.min(r[self.right_idx]) < self.side_clearance:
            steer = 0.0

        # Clear distance straight ahead (narrow cone, extended scan)
        cone = np.abs(self.angles) <= math.radians(self.front_cone_deg)
        ahead = float(np.min(ext[cone]))
        v_brake = math.sqrt(2.0 * self.max_decel * max(ahead - self.brake_margin, 0.0))
        curvature = math.tan(abs(steer)) / self.wheelbase
        v_curve = math.sqrt(self.max_lat_accel / max(curvature, 1e-3))
        speed = float(np.clip(min(v_brake, v_curve, speed_cap), self.v_min, speed_cap))
        speed = float(np.clip(speed, self.last_speed - self.decel_rate * dt,
                              self.last_speed + self.accel_rate * dt))
        self.last_speed = speed
        return steer, speed


class ReactiveFollowGap(Node):
    def __init__(self):
        super().__init__('reactive_node')

        # ---------------- Topics ----------------
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('odom_topic', '/ego_racecar/odom')
        self.declare_parameter('drive_topic', '/drive')

        # ---------------- Follow the Gap ----------------
        self.declare_parameter('fov_deg', 180.0)
        self.declare_parameter('disparity_threshold', 0.3)   # m
        self.declare_parameter('car_half_width', 0.155)      # m
        self.declare_parameter('safety_margin', 0.25)        # m, extra clearance to walls
        self.declare_parameter('max_steer', 0.4189)          # rad
        self.declare_parameter('side_clearance', 0.30)       # m
        self.declare_parameter('v_min', 2.0)                 # m/s
        self.declare_parameter('v_max', 14.0)                # m/s, top speed
        self.declare_parameter('max_decel', 4.0)             # m/s^2, used to plan braking (gentle: hard braking spins the car)
        self.declare_parameter('brake_margin', 0.5)          # m
        self.declare_parameter('max_lat_accel', 12.0)        # m/s^2
        # Speed rate limits, per second of computer clock. The simulator runs
        # slower than real time (~0.7x), so in simulated time they are ~40% higher.
        self.declare_parameter('accel_rate', 4.5)            # m/s^2
        self.declare_parameter('decel_rate', 4.0)            # m/s^2
        # Pure-pursuit steering: aim point at lookahead_min + lookahead_gain * speed
        self.declare_parameter('lookahead_min', 1.0)         # m
        self.declare_parameter('lookahead_gain', 0.3)        # s
        self.declare_parameter('lookahead_max', 4.0)         # m

        # ---------------- Lap pace ----------------
        # The speed cap is adapted so each lap lasts about target_lap_time
        self.declare_parameter('target_lap_time', 60.0)      # s
        self.declare_parameter('max_lap_time', 75.0)         # s, limit shown in the report
        self.declare_parameter('track_length', 488.0)        # m, Sepang lap; re-learned every lap
        self.declare_parameter('pace_initial_cap', 14.0)     # m/s
        self.declare_parameter('pace_kp', 0.05)              # m/s per meter ahead/behind schedule

        # ---------------- Laps ----------------
        self.declare_parameter('target_laps', 10)
        self.declare_parameter('start_x', 0.0)
        self.declare_parameter('start_y', 0.0)
        self.declare_parameter('start_yaw', 0.0)             # rad, driving direction at the line
        self.declare_parameter('start_line_length', 2.5)     # m
        self.declare_parameter('min_lap_distance', 50.0)     # m
        self.declare_parameter('collision_dist', 0.20)       # m, scan reading counted as near-collision
        self.declare_parameter('status_period', 5.0)         # s

        p = lambda name: self.get_parameter(name).value  # noqa: E731
        self.ctrl = GapFollower(
            fov_deg=p('fov_deg'), disparity_threshold=p('disparity_threshold'),
            car_half_width=p('car_half_width'), safety_margin=p('safety_margin'),
            max_steer=p('max_steer'), side_clearance=p('side_clearance'),
            v_min=p('v_min'), max_decel=p('max_decel'),
            brake_margin=p('brake_margin'), max_lat_accel=p('max_lat_accel'),
            accel_rate=p('accel_rate'), decel_rate=p('decel_rate'),
            lookahead_min=p('lookahead_min'), lookahead_gain=p('lookahead_gain'),
            lookahead_max=p('lookahead_max'))
        self.last_scan_time = None
        self.v_max = p('v_max')
        self.v_min = p('v_min')
        self.target_lap_time = p('target_lap_time')
        self.max_lap_time = p('max_lap_time')
        self.track_length = p('track_length')
        self.pace_kp = p('pace_kp')
        self.target_laps = int(p('target_laps'))
        self.start_x, self.start_y = p('start_x'), p('start_y')
        self.start_yaw = p('start_yaw')
        self.start_line_length = p('start_line_length')
        self.min_lap_distance = p('min_lap_distance')
        self.collision_dist = p('collision_dist')

        self.base_cap = min(p('pace_initial_cap'), self.v_max)
        self.speed_cap = self.base_cap

        # ---------------- State ----------------
        self.prev_pos = None
        self.race_start = None
        self.total_distance = 0.0     # m, whole race
        self.prev_along = 0.0
        self.lap_start = None
        self.lap_distance = 0.0
        self.lap_max_speed = 0.0
        self.lap_near_collisions = 0
        self.in_collision = False
        self.current_speed = 0.0
        self.laps = []            # dicts with the data of every completed lap
        self.finished = False

        self.drive_pub = self.create_publisher(AckermannDriveStamped, p('drive_topic'), 10)
        self.create_subscription(LaserScan, p('scan_topic'), self.lidar_callback, 10)
        self.create_subscription(Odometry, p('odom_topic'), self.odom_callback, 10)
        self.create_timer(0.1, self.pace_callback)
        self.create_timer(p('status_period'), self.status_callback)

        self.get_logger().info(
            f'Follow the Gap ready: {self.target_laps} laps, target '
            f'{self.target_lap_time:.0f} s per lap (max {self.max_lap_time:.0f} s).')

    # ---------------------------------------------------------------- time
    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    # ---------------------------------------------------------------- odom
    def odom_callback(self, msg):
        x, y = msg.pose.pose.position.x, msg.pose.pose.position.y
        self.current_speed = msg.twist.twist.linear.x
        dx, dy = x - self.start_x, y - self.start_y
        along = dx * math.cos(self.start_yaw) + dy * math.sin(self.start_yaw)
        cross = -dx * math.sin(self.start_yaw) + dy * math.cos(self.start_yaw)
        t = self.now()

        if self.prev_pos is None:
            self.prev_pos, self.prev_along, self.lap_start = (x, y), along, t
            self.race_start = t
            return
        if self.finished:
            return

        step = math.hypot(x - self.prev_pos[0], y - self.prev_pos[1])
        self.lap_distance += step
        self.total_distance += step
        self.lap_max_speed = max(self.lap_max_speed, self.current_speed)
        self.prev_pos = (x, y)

        crossed = (self.prev_along < 0.0 <= along
                   and abs(cross) <= self.start_line_length / 2.0)
        self.prev_along = along
        if not crossed or self.lap_distance < self.min_lap_distance:
            return

        lap_time = t - self.lap_start
        lap = {
            'lap': len(self.laps) + 1,
            'time': lap_time,
            'distance': self.lap_distance,
            'avg_speed': self.lap_distance / lap_time if lap_time > 0 else 0.0,
            'max_speed': self.lap_max_speed,
            'near_collisions': self.lap_near_collisions,
        }
        self.laps.append(lap)
        self.get_logger().info(
            f"LAP {lap['lap']}/{self.target_laps}: {lap['time']:.2f} s | "
            f"{lap['distance']:.1f} m | avg {lap['avg_speed']:.2f} m/s | "
            f"max {lap['max_speed']:.2f} m/s | near-collisions {lap['near_collisions']}")

        # Lap-to-lap pace correction and learned track length
        self.track_length = self.lap_distance
        self.base_cap = float(np.clip(
            self.base_cap * (1.0 + 0.7 * (lap_time / self.target_lap_time - 1.0)),
            self.v_min, self.v_max))

        self.lap_start = t
        self.lap_distance = 0.0
        self.lap_max_speed = 0.0
        self.lap_near_collisions = 0
        if len(self.laps) >= self.target_laps:
            self.finished = True
            self.publish_drive(0.0, 0.0)
            self.print_summary()

    # ---------------------------------------------------------------- pace
    def pace_callback(self):
        if self.finished or self.lap_start is None or self.target_lap_time <= 0:
            return
        elapsed = self.now() - self.lap_start
        expected = self.track_length * elapsed / self.target_lap_time
        ahead = self.lap_distance - expected          # > 0: ahead of schedule
        self.speed_cap = float(np.clip(self.base_cap - self.pace_kp * ahead,
                                       self.v_min, self.v_max))

    def status_callback(self):
        if self.finished or self.lap_start is None:
            return
        self.get_logger().info(
            f'[Lap {len(self.laps) + 1}/{self.target_laps}] '
            f'lap {self.now() - self.lap_start:5.1f} s / {self.lap_distance:6.1f} m | '
            f'total {self.now() - self.race_start:6.1f} s / {self.total_distance:7.1f} m | '
            f'speed {self.current_speed:4.1f} m/s | cap {self.speed_cap:4.1f} m/s')

    # --------------------------------------------------------------- report
    def print_summary(self):
        best = min(self.laps, key=lambda lap: lap['time'])
        worst = max(self.laps, key=lambda lap: lap['time'])
        n = len(self.laps)
        total_time = sum(lap['time'] for lap in self.laps)
        total_dist = sum(lap['distance'] for lap in self.laps)
        times = np.array([lap['time'] for lap in self.laps])
        top_speed = max(self.laps, key=lambda lap: lap['max_speed'])
        total_near = sum(lap['near_collisions'] for lap in self.laps)
        n_target = sum(1 for lap in self.laps if lap['time'] <= self.target_lap_time)
        n_max = sum(1 for lap in self.laps if lap['time'] <= self.max_lap_time)
        lines = [
            '',
            '=' * 78,
            f'  RACE FINISHED: {len(self.laps)} laps',
            '=' * 78,
            f"  {'Lap':>3} | {'Time (s)':>8} | {'Dist (m)':>8} | {'Avg (m/s)':>9} | "
            f"{'Max (m/s)':>9} | {'Near-coll':>9} | Status",
            '  ' + '-' * 74,
        ]
        for lap in self.laps:
            if lap['time'] <= self.target_lap_time:
                status = 'OK (<= target)'
            elif lap['time'] <= self.max_lap_time:
                status = 'OK (<= max)'
            else:
                status = 'TOO SLOW'
            mark = ''
            if lap is best:
                mark = '  <-- FASTEST'
            elif lap is worst:
                mark = '  <-- SLOWEST'
            lines.append(
                f"  {lap['lap']:>3} | {lap['time']:8.2f} | {lap['distance']:8.1f} | "
                f"{lap['avg_speed']:9.2f} | {lap['max_speed']:9.2f} | "
                f"{lap['near_collisions']:>9} | {status}{mark}")
        lines += [
            '  ' + '-' * 74,
            f'  Total time:          {total_time:.2f} s ({total_time / 60.0:.2f} min)',
            f'  Total distance:      {total_dist:.1f} m ({total_dist / 1000.0:.3f} km)',
            f'  Average speed:       {total_dist / total_time:.2f} m/s',
            f"  Top speed:           {top_speed['max_speed']:.2f} m/s (lap {top_speed['lap']})",
            f'  Average lap:         {times.mean():.2f} s (std dev {times.std():.2f} s) | '
            f'average lap distance: {total_dist / n:.1f} m',
            f'  Laps <= {self.target_lap_time:.0f} s: {n_target}/{n} | '
            f'laps <= {self.max_lap_time:.0f} s: {n_max}/{n}',
            f'  Near-collisions:     {total_near}',
            '  ' + '-' * 74,
            f"  WINNING LAP (fastest):  lap {best['lap']} with {best['time']:.2f} s "
            f"({best['distance']:.1f} m, {best['avg_speed']:.2f} m/s average)",
            f"  SLOWEST LAP (longest):  lap {worst['lap']} with {worst['time']:.2f} s "
            f"({worst['distance']:.1f} m, {worst['avg_speed']:.2f} m/s average)",
            f"  Difference slowest - fastest: {worst['time'] - best['time']:.2f} s",
            '=' * 78,
        ]
        self.get_logger().info('\n'.join(lines))

    # ---------------------------------------------------------------- lidar
    def lidar_callback(self, data):
        r = np.asarray(data.ranges, dtype=np.float64)
        r_min = float(np.min(np.where(np.isfinite(r), r, 30.0)))
        if r_min < self.collision_dist:
            if not self.in_collision and not self.finished:
                self.lap_near_collisions += 1
                self.get_logger().warn(f'Near-collision (min range {r_min:.2f} m)')
            self.in_collision = True
        else:
            self.in_collision = False

        if self.finished:
            self.publish_drive(0.0, 0.0)
            return
        t = self.now()
        dt = 0.01 if self.last_scan_time is None else min(max(t - self.last_scan_time, 0.001), 0.02)
        self.last_scan_time = t
        steer, speed = self.ctrl.compute(r, data.angle_min, data.angle_increment,
                                         self.speed_cap, dt=dt)
        self.publish_drive(steer, speed)

    def publish_drive(self, steer, speed):
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.drive.steering_angle = float(steer)
        msg.drive.speed = float(speed)
        self.drive_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ReactiveFollowGap()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_drive(0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
