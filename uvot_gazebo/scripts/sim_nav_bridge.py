#!/usr/bin/env python3
"""
Puente de topics para navegar en simulación con la misma configuración de
Nav2 que el robot real.

  Nav2 / robot real                 Simulación (ros2_control)
  ---------------------------------------------------------------------------
  /<ns>/cmd_vel (Twist)        ->   skid/diff: /<ns>/<ctrl>/cmd_vel  (TwistStamped)
                                    omni:      /<ns>/mecanum_controller/reference
  /<ns>/odom                   <-   skid/diff: /<ns>/<ctrl>/odom
                                    omni:      /<ns>/mecanum_controller/odometry
  /tf  (odom -> base_link)     <-   omni:      /<ns>/mecanum_controller/tf_odometry
                                    (skid/diff ya publican en /tf)

Los frames que lleguen sin el prefijo '<ns>/' se prefijan, para que encajen con
el árbol del URDF (robot1/odom, robot1/base_link).
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage

CONTROLLERS = {
    'skid': ('skid_controller', 'cmd_vel', 'odom', None),
    'diff': ('diff_controller', 'cmd_vel', 'odom', None),
    'omni': ('mecanum_controller', 'reference', 'odometry', 'tf_odometry'),
}


class SimNavBridge(Node):

    def __init__(self):
        super().__init__('sim_nav_bridge')
        self.ns = self.declare_parameter('robot_ns', 'robot1').value.strip('/')
        rtype = self.declare_parameter('robot_type', 'skid').value
        if rtype not in CONTROLLERS:
            raise RuntimeError(f"robot_type '{rtype}' no soportado (omni, skid, diff)")
        ctrl, cmd, odom, tf = CONTROLLERS[rtype]
        base = f'/{self.ns}/{ctrl}'
        self.base_frame = f'{self.ns}/base_link'

        self.cmd_pub = self.create_publisher(TwistStamped, f'{base}/{cmd}', 10)
        self.create_subscription(Twist, f'/{self.ns}/cmd_vel', self.on_cmd, 10)

        self.odom_pub = self.create_publisher(Odometry, f'/{self.ns}/odom', 10)
        self.create_subscription(Odometry, f'{base}/{odom}', self.on_odom, 10)

        if tf:
            tf_qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE,
                                durability=DurabilityPolicy.VOLATILE)
            self.tf_pub = self.create_publisher(TFMessage, '/tf', tf_qos)
            self.create_subscription(TFMessage, f'{base}/{tf}', self.on_tf, 10)

        self.get_logger().info(
            f'[{rtype}] /{self.ns}/cmd_vel -> {base}/{cmd} ; '
            f'{base}/{odom} -> /{self.ns}/odom'
            + (f' ; {base}/{tf} -> /tf' if tf else ''))

    def fix(self, frame):
        frame = frame.lstrip('/')
        return frame if frame.startswith(self.ns + '/') else f'{self.ns}/{frame}'

    def on_cmd(self, msg):
        out = TwistStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self.base_frame
        out.twist = msg
        self.cmd_pub.publish(out)

    def on_odom(self, msg):
        msg.header.frame_id = self.fix(msg.header.frame_id)
        msg.child_frame_id = self.fix(msg.child_frame_id)
        self.odom_pub.publish(msg)

    def on_tf(self, msg):
        for t in msg.transforms:
            t.header.frame_id = self.fix(t.header.frame_id)
            t.child_frame_id = self.fix(t.child_frame_id)
        self.tf_pub.publish(msg)


def main():
    rclpy.init()
    node = SimNavBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
