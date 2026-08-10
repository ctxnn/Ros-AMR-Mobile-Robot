"""MuJoCo physics backend for supermarketbot, in place of Gazebo Sim.

Steps the MuJoCo model in world/world.xml and publishes what
robot_state_publisher + slam_toolbox need: /joint_states, /odom, TF
(odom->base_link), and /scan (from the 360-ray lidar fan defined in
world.xml). Subscribes /supermarketbot/cmd_vel and converts (v, w) to
wheel velocities using the same kinematics as the old Gazebo DiffDrive
plugin (wheel_separation=0.205, wheel_radius=0.035).

Opens the MuJoCo viewer by default so you can watch the robot and drive it
with the arrow keys while the map builds in RViz. Set the `viewer` parameter
false to run headless and drive over /supermarketbot/cmd_vel instead.

Usage:
    ros2 run supermarketbot mujoco_bridge
    ros2 run supermarketbot mujoco_bridge --ros-args -p viewer:=false
"""
import math
import os

import mujoco
import mujoco.viewer
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState, LaserScan
from tf2_ros import TransformBroadcaster

WHEEL_SEPARATION = 0.205
WHEEL_RADIUS = 0.035
SIM_HZ = 500.0
PUBLISH_HZ = 20.0
VIEWER_HZ = 30.0

# Arrow-key teleop speeds. Slow enough that slam_toolbox gets overlapping
# scans to match against; bump if mapping the whole store drags.
KEY_LINEAR = 0.25   # m/s
KEY_ANGULAR = 0.6   # rad/s

# GLFW keycodes: Up=265, Down=264, Left=263, Right=262, Space=32
KEY_CMD = {
    265: (KEY_LINEAR, 0.0),
    264: (-KEY_LINEAR, 0.0),
    263: (0.0, KEY_ANGULAR),
    262: (0.0, -KEY_ANGULAR),
    32: (0.0, 0.0),
}


class MujocoBridge(Node):
    def __init__(self):
        super().__init__('mujoco_bridge')

        world_path = os.path.join(get_package_share_directory('supermarketbot'), 'world', 'world.xml')
        self.model = mujoco.MjModel.from_xml_path(world_path)
        self.data = mujoco.MjData(self.model)

        self.left_joint = self.model.joint('left_wheel_joint').id
        self.right_joint = self.model.joint('right_wheel_joint').id

        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.create_subscription(Twist, '/supermarketbot/cmd_vel', self.on_cmd_vel, 10)

        self.create_timer(1.0 / SIM_HZ, self.step_sim)
        self.create_timer(1.0 / PUBLISH_HZ, self.publish_state)

        self.declare_parameter('viewer', True)
        self.viewer = None
        if self.get_parameter('viewer').value:
            self.viewer = mujoco.viewer.launch_passive(
                self.model, self.data, key_callback=self.on_key,
                show_left_ui=False, show_right_ui=False,
            )
            self.create_timer(1.0 / VIEWER_HZ, self.sync_viewer)
            self.get_logger().info('viewer open — arrows drive, space stops')

    def on_key(self, keycode):
        if keycode in KEY_CMD:
            self.set_wheels(*KEY_CMD[keycode])

    def sync_viewer(self):
        if not self.viewer.is_running():
            raise SystemExit
        self.viewer.sync()

    def set_wheels(self, v, w):
        self.data.ctrl[0] = (v - w * WHEEL_SEPARATION / 2.0) / WHEEL_RADIUS
        self.data.ctrl[1] = (v + w * WHEEL_SEPARATION / 2.0) / WHEEL_RADIUS

    def on_cmd_vel(self, msg: Twist):
        self.set_wheels(msg.linear.x, msg.angular.z)

    def step_sim(self):
        mujoco.mj_step(self.model, self.data)

    def publish_state(self):
        now = self.get_clock().now().to_msg()
        qpos = self.data.qpos

        # base_link freejoint: qpos[0:3]=xyz, qpos[3:7]=wxyz quat
        x, y, z = qpos[0], qpos[1], qpos[2]
        qw, qx, qy, qz = qpos[3], qpos[4], qpos[5], qpos[6]

        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = z
        t.transform.rotation.w = qw
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        self.tf_broadcaster.sendTransform(t)

        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = z
        odom.pose.pose.orientation.w = qw
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.twist.twist.linear.x = self.data.qvel[0]
        odom.twist.twist.angular.z = self.data.qvel[5]
        self.odom_pub.publish(odom)

        js = JointState()
        js.header.stamp = now
        js.name = ['left_wheel_link_to_chassis_link_joint', 'right_wheel_link_to_chassis_link_joint']
        js.position = [
            float(self.data.qpos[self.model.jnt_qposadr[self.left_joint]]),
            float(self.data.qpos[self.model.jnt_qposadr[self.right_joint]]),
        ]
        js.velocity = [
            float(self.data.qvel[self.model.jnt_dofadr[self.left_joint]]),
            float(self.data.qvel[self.model.jnt_dofadr[self.right_joint]]),
        ]
        self.joint_pub.publish(js)

        n = self.model.nsensor
        scan = LaserScan()
        scan.header.stamp = now
        scan.header.frame_id = 'lidar_link'
        scan.angle_min = -math.pi
        scan.angle_max = math.pi - (2 * math.pi / n)
        scan.angle_increment = 2 * math.pi / n
        scan.range_min = 0.1
        scan.range_max = 10.0
        scan.time_increment = 0.0
        scan.scan_time = 1.0 / PUBLISH_HZ
        ranges = self.data.sensordata.copy()
        ranges[ranges < 0] = float('inf')  # no-hit rays
        scan.ranges = ranges.tolist()
        self.scan_pub.publish(scan)


def main(args=None):
    rclpy.init(args=args)
    node = MujocoBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        if node.viewer is not None:
            node.viewer.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
