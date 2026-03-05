from math import sin, cos, pi
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster, TransformStamped

# ---- helpers ----
def euler_to_quaternion(roll, pitch, yaw):
    qx = sin(roll/2) * cos(pitch/2) * cos(yaw/2) - cos(roll/2) * sin(pitch/2) * sin(yaw/2)
    qy = cos(roll/2) * sin(pitch/2) * cos(yaw/2) + sin(roll/2) * cos(pitch/2) * sin(yaw/2)
    qz = cos(roll/2) * cos(pitch/2) * sin(yaw/2) - sin(roll/2) * sin(pitch/2) * cos(yaw/2)
    qw = cos(roll/2) * cos(pitch/2) * cos(yaw/2) + sin(roll/2) * sin(pitch/2) * sin(yaw/2)
    return Quaternion(x=qx, y=qy, z=qz, w=qw)

class StatePublisher(Node):
    """
    Publishes:
      - /joint_states with names from the URDF:
        ['joint_one','joint_two','joint_three','joint_four','joint_five']
      - TF odom -> base_link (moving in a circle)
    """
    def __init__(self):
        super().__init__('state_publisher')

        qos_profile = QoSProfile(depth=10)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', qos_profile)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.get_logger().info(f"{self.get_name()} started (30 Hz)")

        # Angles (rad) & motion parameters
        self.degree = pi / 180.0
        self.t = 0.0               # time/phase for nice sinusoid motions
        self.ang_base = 0.0        # for odom->base_link yaw

        # URDF limits
        self.j1_min, self.j1_max = -pi, pi                       # joint_one (free)
        self.j2_min, self.j2_max = -0.45, 0.65                   # joint_two
        self.j3_min, self.j3_max = -2.1815, 0.95                 # joint_three
        self.j4_min, self.j4_max = -1.65, 1.9198                 # joint_four
        self.j5_min, self.j5_max = -pi, pi                       # joint_five (free)

        # 30 Hz timer
        self.timer = self.create_timer(1.0/30.0, self._on_timer)

    def _on_timer(self):
        now = self.get_clock().now().to_msg()

        # Generate smooth positions within limits using sin()
        def mid_span(lo, hi):
            return (lo + hi) * 0.5, (hi - lo) * 0.5

        m2, a2 = mid_span(self.j2_min, self.j2_max)
        m3, a3 = mid_span(self.j3_min, self.j3_max)
        m4, a4 = mid_span(self.j4_min, self.j4_max)

        # Sweep speeds (rad/s equivalent via phase increment)
        self.t += 0.02                   # tune sweep speed
        self.ang_base += self.degree/4   # base yaw for TF path

        j1 = (self.t * 0.7) % (2*pi)     # free rotate
        # wrap to [-pi, pi] for aesthetics (optional)
        if j1 > pi:
            j1 -= 2*pi

        j2 = m2 + a2 * sin(self.t * 0.9)
        j3 = m3 + a3 * sin(self.t * 0.6 + 1.1)
        j4 = m4 + a4 * sin(self.t * 0.8 + 2.0)
        j5 = (self.t * 1.1) % (2*pi)
        if j5 > pi:
            j5 -= 2*pi

        # -------- publish /joint_states --------
        js = JointState()
        js.header.stamp = now
        js.name = ['joint_one', 'joint_two', 'joint_three', 'joint_four', 'joint_five']
        js.position = [j1, j2, j3, j4, j5]
        self.joint_pub.publish(js)

        # -------- broadcast TF: odom -> base_link --------
        tfm = TransformStamped()
        tfm.header.stamp = now
        tfm.header.frame_id = 'odom'
        tfm.child_frame_id = 'base_link'   # matches URDF root
        radius = 2.0
        tfm.transform.translation.x = cos(self.ang_base) * radius
        tfm.transform.translation.y = sin(self.ang_base) * radius
        tfm.transform.translation.z = 0.7
        tfm.transform.rotation = euler_to_quaternion(0.0, 0.0, self.ang_base + pi/2.0)
        self.tf_broadcaster.sendTransform(tfm)

def main():
    rclpy.init()
    node = StatePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
