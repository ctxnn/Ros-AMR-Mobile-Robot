"""
Autonomous supermarket explorer — Nav2 FollowWaypoints client.

Sends the aisle waypoints to Nav2's FollowWaypoints action instead of
driving the robot directly. Nav2's costmap-based DWB controller and the
recovery behaviors configured in config/nav2_params.yaml (spin, backup,
drive_on_heading, wait) handle obstacle avoidance and getting unstuck —
a hand-rolled lidar-arc + P-controller kept driving the robot straight
into walls/corners and corrupting the SLAM map on contact.

Requires the Nav2 stack + SLAM Toolbox already running:
    ros2 launch supermarketbot explore.launch.py

Usage:
    ros2 run supermarketbot auto_explorer

    # Save the map when done:
    ros2 run nav2_map_server map_saver_cli -f ~/s_ws/src/supermarketbot/maps/world_map
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowWaypoints


# Snake through all 4 aisles of the 16x12 m supermarket. Robot spawns near (0, 1).
WAYPOINTS = [
    (1.0, 1.5),
    (1.0, 3.0), (-1.0, 3.0), (-3.5, 3.0),
    (-3.5, 1.5),
    (-3.5, 0.5), (0.0, 0.5), (3.5, 0.5),
    (3.5, -1.0),
    (3.5, -2.0), (0.0, -2.0), (-3.5, -2.0),
    (-3.5, -3.5),
    (-3.5, -4.5), (0.0, -4.5), (3.5, -4.5),
    (1.0, 0.0), (1.0, 1.0),
]


def make_pose(x: float, y: float) -> PoseStamped:
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.orientation.w = 1.0
    return pose


class AutoExplorer(Node):

    def __init__(self):
        super().__init__('auto_explorer')
        self._client = ActionClient(self, FollowWaypoints, 'follow_waypoints')

    def run(self):
        self.get_logger().info('Waiting for Nav2 follow_waypoints action server...')
        self._client.wait_for_server()

        goal = FollowWaypoints.Goal()
        goal.poses = [make_pose(x, y) for x, y in WAYPOINTS]

        self.get_logger().info(f'Sending {len(goal.poses)} waypoints to Nav2...')
        send_goal_future = self._client.send_goal_async(
            goal, feedback_callback=self._feedback_cb
        )
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()

        if not goal_handle.accepted:
            self.get_logger().error('Nav2 rejected the waypoint goal.')
            return

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result

        if result.missed_waypoints:
            self.get_logger().warn(
                f'Finished with {len(result.missed_waypoints)} missed waypoint(s): '
                f'{list(result.missed_waypoints)}'
            )
        else:
            self.get_logger().info('All waypoints reached! Save the map now with:')
            self.get_logger().info(
                '  ros2 run nav2_map_server map_saver_cli '
                '-f ~/s_ws/src/supermarketbot/maps/world_map'
            )

    def _feedback_cb(self, feedback_msg):
        current = feedback_msg.feedback.current_waypoint
        self.get_logger().info(f'Heading to waypoint {current + 1}/{len(WAYPOINTS)}')


def main(args=None):
    rclpy.init(args=args)
    node = AutoExplorer()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
