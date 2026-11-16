#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class LidarAnalysisNode(Node):
    def __init__(self):
        super().__init__('lidar_analysis_node')
        self.subscription = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)

        self.last_time = None
        self.intervals = []      # últimos intervalos entre mensajes (s)
        self.last_print = 0.0    # para imprimir ~1 vez por segundo

    def scan_callback(self, msg):
        now = self.get_clock().now().nanoseconds * 1e-9

        # --- Frecuencia: intervalo entre mensajes consecutivos ---
        if self.last_time is not None:
            self.intervals.append(now - self.last_time)
            self.intervals = self.intervals[-100:]   # promedio móvil
        self.last_time = now

        # --- Resolución angular ---
        resolucion = math.degrees(msg.angle_increment)

        # --- División de ranges según el ángulo de cada haz ---
        front_ranges = []
        rear_ranges = []
        for i, r in enumerate(msg.ranges):
            angulo = math.degrees(msg.angle_min + i * msg.angle_increment)
            angulo %= 360.0                  # normaliza a [0, 360)
            if 0.0 <= angulo <= 180.0:
                front_ranges.append(r)
            else:
                rear_ranges.append(r)

        # Imprimir solo una vez por segundo para no saturar la consola
        if now - self.last_print < 1.0 or not self.intervals:
            return
        self.last_print = now

        frecuencia = 1.0 / (sum(self.intervals) / len(self.intervals))

        print('-' * 55)
        print(f'Resolución angular del LiDAR: {resolucion:.3f} grados')
        print(f'Frecuencia del LiDAR: {frecuencia:.3f} Hz')
        print(f'Cantidad de mediciones frontales (0° a 180°): {len(front_ranges)}')
        print(f'Cantidad de mediciones traseras (180° a 360°): {len(rear_ranges)}')
        print(f'Primeros valores frontales: {[round(v, 2) for v in front_ranges[:5]]}')
        print(f'Primeros valores traseros: {[round(v, 2) for v in rear_ranges[:5]]}')


def main(args=None):
    rclpy.init(args=args)
    node = LidarAnalysisNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
