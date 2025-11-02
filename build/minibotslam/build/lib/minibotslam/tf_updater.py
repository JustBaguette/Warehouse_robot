import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from rclpy.qos import QoSProfile, ReliabilityPolicy


class FourWheelDiffOdom(Node):
    def __init__(self):
        super().__init__('odom_publisher_4wd')

        # --- Robot geometry constants ---
        self.WHEEL_BASE = 0.154     # distance between left & right wheels [m]
        self.WHEEL_RADIUS = 0.044   # wheel radius [m]
        self.CRITICAL_FREQ = 10.0  # skip bad timing if lower than 10 Hz

        # --- Robot state ---
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.prev_time = None
        self.v = 0.0
        self.omega = 0.0

        # --- QoS and publishers ---
        qos_rel = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        qos_best = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.odom_pub = self.create_publisher(Odometry, '/odom', qos_rel)
        self.tf_broadcaster = TransformBroadcaster(self)

        # --- Subscription to encoder topic ---
        # Expect geometry_msgs/Twist containing:
        # linear.x = front_left, linear.y = rear_left,
        # linear.z = front_right, angular.x = rear_right
        self.encoder_sub = self.create_subscription(
            Twist, '/encoder_data', self.encoder_callback, qos_best
        )

        # --- Timer for periodic publishing ---
        self.create_timer(0.1, self.publish_odometry)

    def encoder_callback(self, msg: Twist):
        current_time = self.get_clock().now()

        # Decode wheel speeds [rad/s]
        fl = msg.linear.x
        rl = msg.linear.y
        fr = msg.linear.z
        rr = msg.angular.x

        # Average per side
        left_speed = (fl + rl) / 2.0
        right_speed = (fr + rr) / 2.0

        # Linear & angular velocity (differential drive equations)
        v = (self.WHEEL_RADIUS / 2.0) * (right_speed + left_speed)
        omega = (self.WHEEL_RADIUS / self.WHEEL_BASE) * (right_speed - left_speed)

        if self.prev_time is None:
            self.prev_time = current_time
            return

        dt = (current_time - self.prev_time).nanoseconds * 1e-9
        if dt <= 0.0 or dt > 1.0 / self.CRITICAL_FREQ:
            self.prev_time = current_time
            return
        self.prev_time = current_time

        # Integrate motion
        dx = v * math.cos(self.yaw) * dt
        dy = v * math.sin(self.yaw) * dt
        dtheta = omega * dt

        self.x += dx
        self.y += dy
        self.yaw = (self.yaw + dtheta + math.pi) % (2 * math.pi) - math.pi

        # Cache velocities
        self.v = v
        self.omega = omega

    def publish_odometry(self):
        now = self.get_clock().now().to_msg()

        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.z = math.sin(self.yaw / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.yaw / 2.0)

        odom.twist.twist.linear.x = self.v
        odom.twist.twist.angular.z = self.omega

        # Broadcast TF
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.z = math.sin(self.yaw / 2.0)
        t.transform.rotation.w = math.cos(self.yaw / 2.0)
        self.tf_broadcaster.sendTransform(t)

        self.odom_pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = FourWheelDiffOdom()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
