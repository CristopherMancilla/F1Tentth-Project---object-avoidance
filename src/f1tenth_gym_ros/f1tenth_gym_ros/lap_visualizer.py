"""Draws the start/finish line (green) and the ego car trajectory (red) in RViz.

The trajectory is cleared every time the car crosses the start line in the
forward direction, so each lap is drawn from scratch.
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray


class LapVisualizer(Node):
    def __init__(self):
        super().__init__('lap_visualizer')

        # Start line pose (defaults match sim.yaml ego start pose)
        self.declare_parameter('sx', 0.0)
        self.declare_parameter('sy', 0.0)
        self.declare_parameter('stheta', 0.0)
        self.declare_parameter('odom_topic', '/ego_racecar/odom')
        self.declare_parameter('frame_id', 'map')
        # Half length of the start line used until the map arrives
        self.declare_parameter('default_half_width', 1.5)
        # Distance the car must travel before a crossing counts as a new lap
        self.declare_parameter('min_lap_distance', 10.0)
        # Minimum spacing between trajectory points
        self.declare_parameter('point_spacing', 0.05)
        # A jump bigger than this between odom samples means the car was reset
        self.declare_parameter('reset_jump', 2.0)

        self.sx = self.get_parameter('sx').value
        self.sy = self.get_parameter('sy').value
        self.stheta = self.get_parameter('stheta').value
        self.frame_id = self.get_parameter('frame_id').value
        self.min_lap_distance = self.get_parameter('min_lap_distance').value
        self.point_spacing = self.get_parameter('point_spacing').value
        self.reset_jump = self.get_parameter('reset_jump').value

        # Forward (along track) and lateral unit vectors of the start line
        self.fwd = (math.cos(self.stheta), math.sin(self.stheta))
        self.lat = (-math.sin(self.stheta), math.cos(self.stheta))

        default_hw = self.get_parameter('default_half_width').value
        self.left_hw = default_hw   # extent along +lat
        self.right_hw = default_hw  # extent along -lat

        self.trajectory = []
        self.prev_pos = None
        self.dist_since_lap = 0.0
        self.lap = 0
        self.lap_start_time = None

        map_qos = QoSProfile(depth=1,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid, '/map', self.map_cb, map_qos)
        self.create_subscription(Odometry, self.get_parameter('odom_topic').value,
                                 self.odom_cb, 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/lap_markers', 10)
        self.create_timer(0.1, self.publish_markers)

    # ------------------------------------------------------------------ map
    def map_cb(self, msg: OccupancyGrid):
        """Size the start line so it spans the track from wall to wall."""
        self.left_hw = self.raycast(msg, self.lat)
        self.right_hw = self.raycast(msg, (-self.lat[0], -self.lat[1]))
        self.get_logger().info(
            f'Start line spans {self.left_hw + self.right_hw:.2f} m across the track')

    def raycast(self, grid: OccupancyGrid, direction, max_range=10.0):
        info = grid.info
        res = info.resolution
        ox, oy = info.origin.position.x, info.origin.position.y
        d = 0.0
        while d < max_range:
            x = self.sx + d * direction[0]
            y = self.sy + d * direction[1]
            col = int((x - ox) / res)
            row = int((y - oy) / res)
            if not (0 <= col < info.width and 0 <= row < info.height):
                break
            val = grid.data[row * info.width + col]
            if val != 0:  # occupied or unknown -> wall
                break
            d += res / 2.0
        return max(d, 0.5)

    # ----------------------------------------------------------------- odom
    def odom_cb(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        now = self.get_clock().now()

        if self.prev_pos is None:
            self.prev_pos = (x, y)
            self.trajectory = [(x, y)]
            self.lap_start_time = now
            return

        px, py = self.prev_pos
        step = math.hypot(x - px, y - py)

        if step > self.reset_jump:
            # Car was teleported (e.g. /initialpose); start over
            self.get_logger().info('Car reset detected, clearing trajectory')
            self.trajectory = [(x, y)]
            self.dist_since_lap = 0.0
            self.lap_start_time = now
            self.prev_pos = (x, y)
            return

        self.dist_since_lap += step

        if self.crossed_start_line((px, py), (x, y)) and \
                self.dist_since_lap > self.min_lap_distance:
            self.lap += 1
            lap_time = (now - self.lap_start_time).nanoseconds * 1e-9
            self.get_logger().info(f'Lap {self.lap} completed in {lap_time:.2f} s')
            self.trajectory = [(x, y)]
            self.dist_since_lap = 0.0
            self.lap_start_time = now
        else:
            lx, ly = self.trajectory[-1]
            if math.hypot(x - lx, y - ly) >= self.point_spacing:
                self.trajectory.append((x, y))

        self.prev_pos = (x, y)

    def local(self, p):
        dx, dy = p[0] - self.sx, p[1] - self.sy
        along = dx * self.fwd[0] + dy * self.fwd[1]
        side = dx * self.lat[0] + dy * self.lat[1]
        return along, side

    def crossed_start_line(self, p0, p1):
        a0, _ = self.local(p0)
        a1, s1 = self.local(p1)
        # Forward crossing only, and within the line's extent
        return a0 < 0.0 <= a1 and -self.right_hw <= s1 <= self.left_hw

    # -------------------------------------------------------------- markers
    def publish_markers(self):
        stamp = self.get_clock().now().to_msg()
        arr = MarkerArray()

        line = Marker()
        line.header.frame_id = self.frame_id
        line.header.stamp = stamp
        line.ns = 'start_line'
        line.id = 0
        line.type = Marker.LINE_STRIP
        line.action = Marker.ADD
        line.pose.orientation.w = 1.0
        line.scale.x = 0.15
        line.color.r, line.color.g, line.color.b, line.color.a = 0.0, 1.0, 0.0, 1.0
        line.points = [
            Point(x=self.sx - self.right_hw * self.lat[0],
                  y=self.sy - self.right_hw * self.lat[1], z=0.02),
            Point(x=self.sx + self.left_hw * self.lat[0],
                  y=self.sy + self.left_hw * self.lat[1], z=0.02),
        ]
        arr.markers.append(line)

        traj = Marker()
        traj.header.frame_id = self.frame_id
        traj.header.stamp = stamp
        traj.ns = 'trajectory'
        traj.id = 1
        traj.pose.orientation.w = 1.0
        if len(self.trajectory) >= 2:
            traj.type = Marker.LINE_STRIP
            traj.action = Marker.ADD
            traj.scale.x = 0.08
            traj.color.r, traj.color.g, traj.color.b, traj.color.a = 1.0, 0.0, 0.0, 1.0
            traj.points = [Point(x=x, y=y, z=0.01) for x, y in self.trajectory]
        else:
            traj.action = Marker.DELETE
        arr.markers.append(traj)

        self.marker_pub.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = LapVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
