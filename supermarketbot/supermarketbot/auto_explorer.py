"""
Autonomous supermarket explorer — direct velocity control.

Navigates the robot through predefined waypoints using simple
proportional control on odometry, with LiDAR-based obstacle avoidance.
Does NOT require Nav2 — works directly with slam.launch.py.

Fixes vs original:
  - Narrowed front-arc scan to ±20° (avoids false triggers from side shelves)
  - Reduced OBSTACLE_DIST to 0.25 m (allows driving through aisles)
  - Reduced SLOW_DIST to 0.50 m (less aggressive slow-down)
  - Added stuck detector: if robot hasn't moved in STUCK_TIMEOUT s, skip waypoint
  - Added back-up recovery: reverse for BACKUP_DURATION s before rotating
  - Increased WP_TOLERANCE to 0.50 m (skip unreachable points faster)

Usage:
    # Terminal 1 — launch SLAM
    ros2 launch supermarketbot slam.launch.py
    # OR headless (faster on WSL2):
    ros2 launch supermarketbot slam_headless.launch.py

    # Terminal 2 — start autonomous exploration
    source ~/s_ws/install/setup.bash
    ros2 run supermarketbot auto_explorer

    # Terminal 3 — save the map when done
    ros2 run nav2_map_server map_saver_cli -f ~/s_ws/src/supermarketbot/maps/world_map
"""

import math
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan


# ── Tuning constants ──────────────────────────────────────────────────
LINEAR_SPEED    = 0.25     # m/s — cruise speed (bumped up slightly)
ANGULAR_SPEED   = 0.6      # rad/s — max turning rate
WP_TOLERANCE    = 0.50     # m — how close counts as "reached" (was 0.35)
SELF_HIT_RANGE  = 0.12     # m — ignore LiDAR below this (robot body)
OBSTACLE_DIST   = 0.25     # m — hard stop distance (was 0.35 — too aggressive in aisles)
SLOW_DIST       = 0.50     # m — slow-down zone (was 0.70)
KP_LINEAR       = 0.6      # proportional gain for forward speed
KP_ANGULAR      = 1.5      # proportional gain for turning
HEADING_THRESH  = 0.5      # rad — rotate-in-place if heading error exceeds this

# Stuck detection
STUCK_TIMEOUT   = 2.0      # s — seconds without meaningful movement → skip waypoint
STUCK_MIN_DIST  = 0.03    # m — movement below this counts as "stuck"

# Back-up recovery
BACKUP_DURATION = 2.0      # s — reverse for this long when stuck against obstacle
BACKUP_SPEED    = -0.15    # m/s — slow reverse
BACKUP_TURN_Z   = 0.4      # rotate while backing up to escape corners

# Front arc for obstacle check (±degrees from forward)
FRONT_ARC_DEG   = 25      # narrowed from 30° to reduce false positives from side shelves

# ── Waypoints ─────────────────────────────────────────────────────────
# Snake through all 4 aisles of the 16×12 m supermarket.
# Robot spawns at (1.0, 1.0).
# Waypoints are placed in the AISLE CORRIDORS, not inside shelves.
WAYPOINTS = [
    # ── Start: move away from spawn, face the store ──
    ( 1.0,  1.5),

    # ── Aisle 1 (Beverages, y ≈ 3.0) — left to right ──
    ( 1.0,  3.0),
    (-1.0,  3.0),
    (-3.5,  3.0),

    # ── Cross-aisle transition at left wall ──
    (-3.5,  1.5),

    # ── Aisle 2 (Snacks, y ≈ 0.5) — left to right ──
    (-3.5,  0.5),
    ( 0.0,  0.5),
    ( 3.5,  0.5),

    # ── Cross-aisle transition at right wall ──
    ( 3.5, -1.0),

    # ── Aisle 3 (Dairy, y ≈ −2.0) — right to left ──
    ( 3.5, -2.0),
    ( 0.0, -2.0),
    (-3.5, -2.0),

    # ── Cross-aisle transition at left wall ──
    (-3.5, -3.5),

    # ── Aisle 4 (Household, y ≈ −4.5) — left to right ──
    (-3.5, -4.5),
    ( 0.0, -4.5),
    ( 3.5, -4.5),

    # ── Return toward center/spawn ──
    ( 1.0,  0.0),
    ( 1.0,  1.0),    # back near spawn
]


def normalize_angle(angle: float) -> float:
    """Wrap angle to [−π, π]."""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def yaw_from_quaternion(q) -> float:
    """Extract yaw from a quaternion."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class AutoExplorer(Node):
    """Waypoint explorer with stuck detection + back-up recovery."""

    def __init__(self):
        super().__init__('auto_explorer')

        # Publishers / Subscribers
        self.cmd_pub = self.create_publisher(Twist, '/supermarketbot/cmd_vel', 10)

        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self._odom_cb, 10
        )
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self._scan_cb, 10
        )

        # Pose state
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.odom_received = False
        self.min_front_dist = float('inf')  # min obstacle dist in front arc

        # Waypoint state
        self.wp_index = 0
        self.finished = False

        # Stuck detection
        self._last_check_x = 0.0
        self._last_check_y = 0.0
        self._last_check_time = time.time()
        self._stuck_count = 0          # consecutive stuck checks

        # Back-up recovery state
        self._backing_up = False
        self._backup_end_time = 0.0

        # Control loop at 10 Hz
        self.timer = self.create_timer(0.1, self._control_loop)

        self.get_logger().info(
            f'🛒 Auto-Explorer started — {len(WAYPOINTS)} waypoints.\n'
            f'   OBSTACLE_DIST={OBSTACLE_DIST}m  WP_TOLERANCE={WP_TOLERANCE}m  '
            f'FRONT_ARC=±{FRONT_ARC_DEG}°\n'
            f'   Stuck timeout: {STUCK_TIMEOUT}s → skip waypoint.'
        )

    # ── Callbacks ─────────────────────────────────────────────────────

    def _odom_cb(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        
        # Track commanded vs actual velocity to detect wall contact
        self.actual_linear_vel = msg.twist.twist.linear.x
        self.odom_received = True

    def _scan_cb(self, msg: LaserScan):
        n = len(msg.ranges)
        if n == 0:
            return

        arc_indices = int(n * FRONT_ARC_DEG / 360.0)
        front_ranges = []
        for i in list(range(0, arc_indices)) + list(range(n - arc_indices, n)):
            r = msg.ranges[i]
            # KEY FIX: treat near-zero as a VALID wall hit, not a self-hit
            if math.isfinite(r) and r > SELF_HIT_RANGE:
                front_ranges.append(r)
            elif r <= SELF_HIT_RANGE and r > 0.01:  # not NaN/zero — it's a real wall!
                front_ranges.append(SELF_HIT_RANGE)  # clamp but don't ignore

        self.min_front_dist = min(front_ranges) if front_ranges else float('inf')
    # ── Stuck detection ───────────────────────────────────────────────

    def _check_if_stuck(self) -> bool:
        """Returns True if robot hasn't moved meaningfully since last check."""
        now = time.time()
        if now - self._last_check_time < STUCK_TIMEOUT:
            return False

        dist_moved = math.sqrt(
            (self.x - self._last_check_x) ** 2 +
            (self.y - self._last_check_y) ** 2
        )
        self._last_check_time = now
        self._last_check_x = self.x
        self._last_check_y = self.y

        if dist_moved < STUCK_MIN_DIST:
            self._stuck_count += 1
            return True

        self._stuck_count = 0
        return False

    # ── Control loop ──────────────────────────────────────────────────

    def _control_loop(self):

        if self.finished:
            return

        if not self.odom_received:
            return  # wait for first odom

        if (hasattr(self, '_last_cmd_vel') and 
            self._last_cmd_vel > 0.05 and 
            abs(self.actual_linear_vel) < 0.02):
            self._wall_contact_count = getattr(self, '_wall_contact_count', 0) + 1
            if self._wall_contact_count > 5:
                self.get_logger().warn('🧱 Wall contact detected via velocity mismatch!')
                self._wall_contact_count = 0
                self._backing_up = True
                self._backup_end_time = time.time() + BACKUP_DURATION
                return
        else:
            self._wall_contact_count = 0

        if self.wp_index >= len(WAYPOINTS):
            self._stop()
            self.finished = True
            self.get_logger().info(
                '🎉 Exploration COMPLETE! All waypoints reached.\n'
                '   Save the map now with:\n'
                '   ros2 run nav2_map_server map_saver_cli '
                '-f ~/s_ws/src/supermarketbot/maps/world_map'
            )
            return

        # ── Back-up recovery (active) ──────────────────────────────
        if self._backing_up:
            if time.time() < self._backup_end_time:
                cmd = Twist()
                cmd.linear.x = BACKUP_SPEED
                cmd.angular.z = BACKUP_TURN_Z    # rotate while backing up
                self.cmd_pub.publish(cmd)
                return
            else:
                self._backing_up = False
                self.get_logger().info('🔄 Backup done — resuming navigation.')

        target_x, target_y = WAYPOINTS[self.wp_index]
        dx = target_x - self.x
        dy = target_y - self.y
        distance = math.sqrt(dx * dx + dy * dy)

        # ── Reached waypoint? ──────────────────────────────────────
        if distance < WP_TOLERANCE:
            self.get_logger().info(
                f'✅ Waypoint {self.wp_index + 1}/{len(WAYPOINTS)} reached '
                f'({target_x:.1f}, {target_y:.1f})'
            )
            self.wp_index += 1
            self._last_check_time = time.time()
            self._last_check_x = self.x
            self._last_check_y = self.y
            self._stuck_count = 0
            return

        # ── Stuck check ────────────────────────────────────────────
        if self._check_if_stuck():
            self.get_logger().warn(
                f'⏭️  STUCK at waypoint {self.wp_index + 1} '
                f'({target_x:.1f}, {target_y:.1f}) — skipping! '
                f'(stuck_count={self._stuck_count})'
            )
            self.wp_index += 1
            return

        # ── Heading computation ────────────────────────────────────
        desired_yaw = math.atan2(dy, dx)
        heading_error = normalize_angle(desired_yaw - self.yaw)

        cmd = Twist()

        # ── Obstacle avoidance ─────────────────────────────────────
        if self.min_front_dist < OBSTACLE_DIST:
            # Hard stop — initiate back-up then rotate
            self._stop()
            self.get_logger().warn(
                f'🚧 Obstacle at {self.min_front_dist:.2f} m — backing up!'
            )
            self._backing_up = True
            self._backup_end_time = time.time() + BACKUP_DURATION
            return

        elif abs(heading_error) > HEADING_THRESH:
            # Rotate in place to align with waypoint
            cmd.linear.x = 0.0
            cmd.angular.z = KP_ANGULAR * heading_error
            cmd.angular.z = max(-ANGULAR_SPEED, min(ANGULAR_SPEED, cmd.angular.z))

        else:
            # Drive toward waypoint
            speed = KP_LINEAR * distance
            speed = min(speed, LINEAR_SPEED)

            # Slow down near obstacles
            if self.min_front_dist < SLOW_DIST:
                speed *= (self.min_front_dist / SLOW_DIST)
                speed = max(speed, 0.05)  # ensure minimum creep speed

            cmd.linear.x = speed
            cmd.angular.z = KP_ANGULAR * heading_error
            cmd.angular.z = max(-ANGULAR_SPEED, min(ANGULAR_SPEED, cmd.angular.z))

        self._last_cmd_vel = cmd.linear.x
        self.cmd_pub.publish(cmd)

    def _stop(self):
        """Publish zero velocity."""
        self.cmd_pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = AutoExplorer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
