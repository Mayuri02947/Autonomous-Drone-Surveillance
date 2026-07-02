
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty, Bool, Int8
from sensor_msgs.msg import NavSatFix
import time


class WaypointNavigator(Node):
    def __init__(self):
        super().__init__('waypoint_navigator')

        self.cmd_pub = self.create_publisher(Twist, '/simple_drone/cmd_vel', 10)
        self.takeoff_pub = self.create_publisher(Empty, '/simple_drone/takeoff', 10)
        self.land_pub = self.create_publisher(Empty, '/simple_drone/land', 10)
        self.posctrl_pub = self.create_publisher(Bool, '/simple_drone/posctrl', 10)

        self.gps_sub = self.create_subscription(NavSatFix, '/simple_drone/gps/nav', self.gps_cb, 10)
        self.state_sub = self.create_subscription(Int8, '/simple_drone/state', self.state_cb, 10)
        self.current_gps = None
        self.drone_state = 0

        self.waypoints = [
            (5.0, 0.0, 5.0, 0.0),
            (5.0, 5.0, 5.0, 1.57),
            (0.0, 5.0, 5.0, 3.14),
            (0.0, 0.0, 5.0, -1.57),
        ]
        self.wp_index = 0
        self.last_wp_change = time.time()
        self.wp_hold_seconds = 6.0

        self.get_logger().info('Waiting for connection to drone plugin...')
        while self.cmd_pub.get_subscription_count() == 0:
            rclpy.spin_once(self, timeout_sec=0.5)
        self.get_logger().info('Connected. Enabling position control and taking off...')

        self.enable_posctrl()
        time.sleep(1.0)
        self.takeoff()

        wait_start = time.time()
        while self.drone_state == 0 and (time.time() - wait_start) < 10.0:
            rclpy.spin_once(self, timeout_sec=0.5)
            if self.drone_state == 0:
                self.takeoff_pub.publish(Empty())

        if self.drone_state == 0:
            self.get_logger().warn('Takeoff did not confirm after 10s, continuing anyway.')
        else:
            self.get_logger().info('Takeoff confirmed, state={}'.format(self.drone_state))

        self.timer = self.create_timer(0.2, self.publish_current_target)

    def gps_cb(self, msg):
        self.current_gps = msg

    def state_cb(self, msg):
        self.drone_state = msg.data

    def enable_posctrl(self):
        msg = Bool()
        msg.data = True
        self.posctrl_pub.publish(msg)
        self.get_logger().info('Position control mode enabled.')

    def takeoff(self):
        self.takeoff_pub.publish(Empty())
        self.get_logger().info('Takeoff command sent.')

    def land(self):
        self.land_pub.publish(Empty())
        self.get_logger().info('Land command sent.')

    def publish_current_target(self):
        now = time.time()
        if now - self.last_wp_change > self.wp_hold_seconds:
            self.wp_index = (self.wp_index + 1) % len(self.waypoints)
            self.last_wp_change = now

            gps_str = 'unknown'
            if self.current_gps is not None:
                gps_str = 'lat={:.6f}, lon={:.6f}, alt={:.2f}'.format(
                    self.current_gps.latitude, self.current_gps.longitude, self.current_gps.altitude)
            x, y, z, yaw = self.waypoints[self.wp_index]
            self.get_logger().info(
                'Moving to waypoint {}/{} local=({}, {}, {}) | GPS: {} | state={}'.format(
                    self.wp_index + 1, len(self.waypoints), x, y, z, gps_str, self.drone_state)
            )

        x, y, z, yaw = self.waypoints[self.wp_index]
        cmd = Twist()
        cmd.linear.x = x
        cmd.linear.y = y
        cmd.linear.z = z
        cmd.angular.z = yaw
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = WaypointNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.land()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
