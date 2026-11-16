#!/usr/bin/env python3
"""
square_driver.py - F1TENTH: recorre el pasillo rectangular del mapa LevineB usando SOLO odometria.

Suscribe : nav_msgs/Odometry                    (odom_topic, por defecto /ego_racecar/odom)
Publica  : ackermann_msgs/AckermannDriveStamped (drive_topic, por defecto /drive)

Medidas de LevineB (leidas de levineB.png / levineB.yaml: resolucion 0.05 m, origen (-25, -12.75)):
    paredes exteriores : x = -14.70 .. 10.40   y = -0.90 .. 9.65
    paredes interiores : x = -13.00 ..  8.70   y =  0.85 .. 7.90
    ancho del pasillo  : ~1.7 m
    linea central      : x = -13.85 / 9.55      y = 0.0 / 8.78
El robot parte en (0, 0) con yaw 0 (sim.yaml: sx=0, sy=0, stheta=0), en el pasillo inferior.

Logica por lado:
  1. Recta: mantiene la linea central (error de rumbo + error lateral, control tipo Stanley).
  2. Cuando la distancia que falta a la esquina (calculada con la odometria) es <= R_giro,
     inicia el giro con steering controlado hacia la izquierda.
  3. El giro termina cuando el yaw alcanza el rumbo del siguiente lado.
  4. Repite hasta completar la vuelta y se detiene en el punto de partida.
"""
import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def clip(x, lo, hi):
    return max(lo, min(hi, x))


class SquareController:
    """Controlador puro (sin ROS): step(x, y, yaw) -> (velocidad, steering)."""
    STRAIGHT, TURN, DONE = 0, 1, 2

    def __init__(self, points, speed=1.0, turn_speed=0.8, wheelbase=0.3302, turn_steer=0.38,
                 pre_turn_margin=0.0, k_heading=1.0, k_cross=1.5, k_turn=3.0, yaw_tol=0.10,
                 decel=1.5, max_steer=0.4189, log=None):
        self.pts = points
        self.v, self.v_turn = speed, turn_speed
        self.turn_steer = turn_steer
        self.max_steer = max_steer
        self.k_h, self.k_c, self.k_turn = k_heading, k_cross, k_turn
        self.yaw_tol = yaw_tol
        self.decel = decel
        self.log = log or (lambda m: None)
        # Radio de giro del centro del auto con steering constante
        r_rear = wheelbase / math.tan(turn_steer)
        self.r_turn = math.sqrt(r_rear ** 2 + (wheelbase / 2.0) ** 2)
        self.d_pre = self.r_turn + pre_turn_margin
        # lados: heading de cada tramo
        self.headings = [math.atan2(b[1] - a[1], b[0] - a[0]) for a, b in zip(points[:-1], points[1:])]
        self.leg = 0
        self.state = self.STRAIGHT
        self.log(f'Radio de giro = {self.r_turn:.2f} m -> el giro inicia {self.d_pre:.2f} m antes de cada esquina')

    def step(self, x, y, yaw):
        if self.state == self.DONE:
            return 0.0, 0.0

        end = self.pts[self.leg + 1]
        th = self.headings[self.leg]
        last = self.leg == len(self.headings) - 1
        remaining = math.cos(th) * (end[0] - x) + math.sin(th) * (end[1] - y)  # distancia al final del lado
        cross = -math.sin(th) * (x - end[0]) + math.cos(th) * (y - end[1])     # + = a la izquierda de la linea

        if self.state == self.STRAIGHT:
            steer = self.k_h * wrap(th - yaw) - math.atan2(self.k_c * cross, max(self.v, 0.3))
            stop_at = 0.0 if last else self.d_pre
            tgt = 0.05 if last else self.v_turn
            # frenado suave hacia la esquina (o hacia el punto final)
            speed = clip(math.sqrt(tgt ** 2 + 2.0 * self.decel * max(remaining - stop_at, 0.0)), tgt, self.v)

            if last:
                if remaining <= 0.15:
                    self.state = self.DONE
                    self.log('Vuelta completa. Deteniendo el robot.')
                    return 0.0, 0.0
            elif remaining <= self.d_pre:
                self.state = self.TURN
                self.log(f'Lado {self.leg + 1}: inicia giro a {remaining:.2f} m de la esquina '
                         f'({end[0]:.2f}, {end[1]:.2f})')
            return speed, steer

        # ---- giro ----
        nxt = self.headings[self.leg + 1]
        err = wrap(nxt - yaw)
        sign = 1.0 if wrap(nxt - th) > 0 else -1.0
        steer = sign * min(self.turn_steer, self.k_turn * abs(err))
        if abs(err) < self.yaw_tol:
            self.leg += 1
            self.state = self.STRAIGHT
            self.log(f'Giro terminado, iniciando lado {self.leg + 1}')
            return self.v_turn, self.k_h * wrap(nxt - yaw)
        return self.v_turn, steer


class SquareDriver(Node):
    def __init__(self):
        super().__init__('square_driver')
        d = self.declare_parameter
        d('odom_topic', '/ego_racecar/odom')
        d('drive_topic', '/drive')
        # linea central del pasillo de LevineB (medidas del mapa)
        d('x_right', 9.55)
        d('x_left', -13.85)
        d('y_bottom', 0.0)
        d('y_top', 8.78)
        d('start_x', 0.0)
        d('start_y', 0.0)
        d('speed', 1.0)
        d('turn_speed', 0.8)
        d('wheelbase', 0.3302)
        d('turn_steer', 0.38)
        d('pre_turn_margin', 0.0)
        d('k_heading', 1.0)
        d('k_cross', 1.5)
        d('k_turn', 3.0)
        d('yaw_tol', 0.10)
        d('laps', 1)
        d('rate', 50.0)
        g = lambda n: self.get_parameter(n).value

        xr, xl, yb, yt = g('x_right'), g('x_left'), g('y_bottom'), g('y_top')
        start = (g('start_x'), g('start_y'))
        corners = [(xr, yb), (xr, yt), (xl, yt), (xl, yb)]          # antihorario
        points = [start] + corners * int(g('laps')) + [start]

        self.ctrl = SquareController(
            points, speed=g('speed'), turn_speed=g('turn_speed'), wheelbase=g('wheelbase'),
            turn_steer=g('turn_steer'), pre_turn_margin=g('pre_turn_margin'),
            k_heading=g('k_heading'), k_cross=g('k_cross'), k_turn=g('k_turn'),
            yaw_tol=g('yaw_tol'), log=self.get_logger().info)

        self.pose = None
        self.create_subscription(Odometry, g('odom_topic'), self.odom_cb, 10)
        self.pub = self.create_publisher(AckermannDriveStamped, g('drive_topic'), 10)
        self.create_timer(1.0 / g('rate'), self.loop)

    def odom_cb(self, msg):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        first = self.pose is None
        self.pose = (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)
        if first:
            self.get_logger().info(
                f'Pose inicial: x={self.pose[0]:.2f} y={self.pose[1]:.2f} yaw={math.degrees(yaw):.1f} deg')

    def publish(self, speed, steer):
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.drive.speed = float(speed)
        msg.drive.steering_angle = float(clip(steer, -self.ctrl.max_steer, self.ctrl.max_steer))
        self.pub.publish(msg)

    def loop(self):
        if self.pose is None:
            return
        v, steer = self.ctrl.step(*self.pose)
        self.publish(v, steer)


def main():
    rclpy.init()
    node = SquareDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish(0.0, 0.0)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
