#!/usr/bin/env python3
"""Camera-only ArUco docking: align at 30 cm, advance to 10 cm.

Undock reverses straight until camera-to-marker z reaches 30 cm. Marker loss stops movement.
Use independent collision protection and a physically verified E-stop.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import PoseStamped, Twist
from std_srvs.srv import SetBool
from std_msgs.msg import Bool, String
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
    QoSDurabilityPolicy,
)


class ArucoDockerNode(Node):
    STATE_STANDBY = 'STANDBY'
    STATE_ALIGNING = 'ALIGNING'
    STATE_FINAL_APPROACH = 'FINAL_APPROACH'
    STATE_DOCKED = 'DOCKED'
    STATE_UNDOCKING = 'UNDOCKING'

    def __init__(self):
        super().__init__('aruco_docker_node')

        self.state = self.STATE_STANDBY
        self.is_docking_active = False
        self.is_searching_marker = False

        self.last_pose_time = self.get_clock().now()
        self.search_start_time = None
        self.marker_missing_stopped = False

        self.last_marker_z = None
        self.last_marker_x = None
        self.last_marker_yaw = None
        self.realign_backoff_started = None
        self.realign_backoff_blocked = False
        self.final_started = None
        self.last_control_log = self.get_clock().now()
        self.progress_z_start = None
        self.progress_start_time = None
        self.undock_started = None

        # ---------------------------
        # ArUco alignment target
        # ---------------------------
        self.target_z = float(self.declare_parameter('target_z', 0.30).value)
        self.target_x = float(self.declare_parameter('target_x', 0.0).value)
        self.target_yaw = float(self.declare_parameter('target_yaw', 0.0).value)

        self.tol_z = float(self.declare_parameter('tol_z', 0.04).value)
        self.tol_x = float(self.declare_parameter('tol_x', 0.05).value)
        self.tol_yaw = float(self.declare_parameter('tol_yaw', 0.05).value)

        self.k_linear = float(self.declare_parameter('k_linear', 0.3).value)
        self.k_angular = float(self.declare_parameter('k_angular', 0.2).value)
        self.k_yaw = float(self.declare_parameter('k_yaw', 0.6).value)
        # +1 keeps legacy yaw direction; -1 reverses ONLY yaw correction.
        self.yaw_control_sign = float(self.declare_parameter('yaw_control_sign', 1.0).value)

        self.min_linear_vel = float(self.declare_parameter('min_linear_vel', 0.05).value)
        self.max_linear_vel = float(self.declare_parameter('max_linear_vel', 0.20).value)
        self.min_angular_vel = float(self.declare_parameter('min_angular_vel', 0.05).value)
        self.max_angular_vel = float(self.declare_parameter('max_angular_vel', 0.40).value)

        self.linear_stop_band = float(self.declare_parameter('linear_stop_band', 0.04).value)
        self.angular_stop_band = float(self.declare_parameter('angular_stop_band', 0.05).value)
        self.approach_yaw_gate = float(self.declare_parameter('approach_yaw_gate', 0.25).value)
        self.max_linear_angular_mix = float(
            self.declare_parameter('max_linear_angular_mix', 0.20).value
        )

        # Marker-distance software floor.
        # If closer than this, forward is blocked, but REVERSE is still allowed.
        self.min_safe_marker_z = float(
            self.declare_parameter('min_safe_marker_z', 0.10).value
        )

        # Camera-only final distance to the detected ArUco marker (meters).
        self.dock_camera_distance = float(self.declare_parameter('dock_camera_distance', 0.10).value)
        self.undock_camera_distance = float(self.declare_parameter('undock_camera_distance', 0.30).value)
        self.final_approach_speed = float(self.declare_parameter('final_approach_speed', 0.10).value)
        self.undock_speed = float(self.declare_parameter('undock_speed', 0.10).value)
        self.undock_max_sec = float(self.declare_parameter('undock_max_sec', 10.0).value)
        self.recovery_speed = float(self.declare_parameter('recovery_speed', 0.10).value)
        self.realign_backoff_max_sec = float(self.declare_parameter('realign_backoff_max_sec', 100000.0).value)
        self.x_control_sign = float(self.declare_parameter('x_control_sign', 1.0).value)
        self.final_max_sec = float(self.declare_parameter('final_max_sec', 30.0).value)
        self.progress_timeout_sec = float(self.declare_parameter('progress_timeout_sec', 4.0).value)
        self.progress_min_delta = float(self.declare_parameter('progress_min_delta', 0.015).value)
        # ---------------------------
        # Marker watchdog/search
        # ---------------------------
        self.marker_stale_sec = float(
            self.declare_parameter('marker_stale_sec', 0.50).value
        )
        self.marker_lost_start_sec = float(
            self.declare_parameter('marker_lost_start_sec', 1.50).value
        )
        self.enable_marker_search = bool(
            self.declare_parameter('enable_marker_search', True).value
        )
        self.search_angular_vel = float(
            self.declare_parameter('search_angular_vel', 0.20).value
        )
        self.search_timeout_sec = float(
            self.declare_parameter('search_timeout_sec', 20.0).value
        )

        if not (
            0.0 < self.min_linear_vel <= self.max_linear_vel
            and 0.0 < self.min_angular_vel <= self.max_angular_vel
        ):
            raise ValueError('min velocities must be > 0 and <= max velocities')

        if self.linear_stop_band < self.tol_z:
            raise ValueError('linear_stop_band must be >= tol_z')

        if self.angular_stop_band < self.tol_yaw:
            raise ValueError('angular_stop_band must be >= tol_yaw')

        if self.x_control_sign not in (-1.0, 1.0):
            raise ValueError('x_control_sign must be +1 or -1')
        if self.yaw_control_sign not in (-1.0, 1.0):
            raise ValueError('yaw_control_sign must be +1 or -1')
        if not (0 < self.dock_camera_distance < self.target_z <= self.undock_camera_distance):
            raise ValueError('Require dock_camera_distance < target_z <= undock_camera_distance')
        if self.undock_max_sec <= 0.0:
            raise ValueError('undock_max_sec must be positive')

        # ---------------------------
        # ROS I/O
        # ---------------------------
        self.pose_sub = self.create_subscription(
            PoseStamped, '/aruco_pose', self.pose_callback, 10
        )

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.srv = self.create_service(
            SetBool, '/toggle_docking', self.toggle_callback
        )

        self.status_pub = self.create_publisher(String, '/docking_status', 10)
        self.complete_pub = self.create_publisher(Bool, '/docking_complete', 10)

        detector_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            depth=1,
        )
        self.detector_enable_pub = self.create_publisher(
            Bool, '/aruco_detector_enable', detector_qos
        )

        self.watchdog_timer = self.create_timer(0.10, self.control_timer)

        self.publish_detector_enable(False)
        self.publish_docking_status(self.STATE_STANDBY, False)

        self.get_logger().info(
            'ArUco camera-only: align@0.30m -> camera@0.10m -> '
            'undock camera@0.30m'
        )

    # ------------------------------------------------------------------
    # Common helpers
    # ------------------------------------------------------------------
    def stop(self):
        self.cmd_pub.publish(Twist())

    def publish_docking_status(self, status, completed=False):
        msg = String()
        msg.data = str(status)
        self.status_pub.publish(msg)

        done = Bool()
        done.data = bool(completed)
        self.complete_pub.publish(done)

    def set_state(self, state, completed=False):
        if self.state != state:
            self.get_logger().info(f'[STATE] {self.state} -> {state}')
        self.state = state
        self.publish_docking_status(state, completed)

    def publish_detector_enable(self, enable):
        msg = Bool()
        msg.data = bool(enable)
        self.detector_enable_pub.publish(msg)

    @staticmethod
    def yaw_from_quaternion(q):
        return math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )

    @staticmethod
    def limited_nonzero(v, low, high):
        if v == 0.0:
            return 0.0
        return math.copysign(min(max(abs(v), low), high), v)

    # ------------------------------------------------------------------
    # Camera pose freshness / bounded reverse
    # ------------------------------------------------------------------
    def marker_is_fresh(self):
        return (self.last_marker_z is not None and
                (self.get_clock().now() - self.last_pose_time).nanoseconds / 1e9
                <= self.marker_stale_sec)

    def reverse_command(self, speed):
        # Without a rear range sensor the reverse clearance is NOT measured here.
        # Require an independent rear collision interlock in the robot.
        cmd = Twist()
        cmd.linear.x = -abs(speed)
        self.cmd_pub.publish(cmd)
        return True

    # ------------------------------------------------------------------
    # Service
    # ------------------------------------------------------------------
    def toggle_callback(self, request, response):
        if request.data:
            self.stop()

            self.is_docking_active = True
            self.is_searching_marker = False
            self.marker_missing_stopped = False
            self.search_start_time = None
            self.realign_backoff_started = None
            self.realign_backoff_blocked = False
            self.last_marker_z = None
            self.final_started = None
            self.undock_started = None
            self.progress_z_start = None
            self.progress_start_time = None
            self.last_pose_time = self.get_clock().now()

            self.publish_detector_enable(True)
            self.set_state(self.STATE_ALIGNING, False)

            response.success = True
            response.message = (
                'Docking activated: ArUco alignment to 0.30 m'
            )
            return response

        # OFF: straight camera-guided retreat to 30 cm. Keep detector active.
        self.stop()
        self.is_docking_active = False
        self.is_searching_marker = False
        self.search_start_time = None
        self.publish_detector_enable(True)
        if self.marker_is_fresh() and self.last_marker_z >= self.undock_camera_distance:
            self.publish_detector_enable(False)
            self.set_state(self.STATE_STANDBY, False)
            response.success = True
            response.message = 'Already at camera clearance >= 0.30 m'
            return response
        self.undock_started = self.get_clock().now()
        self.set_state(self.STATE_UNDOCKING, False)
        response.success = True
        response.message = 'Undocking: straight reverse until camera marker_z >= 0.30 m; stale marker stops'
        return response

    # ------------------------------------------------------------------
    # Main state controller
    # ------------------------------------------------------------------
    def control_timer(self):
        if self.state == self.STATE_ALIGNING:
            self.check_marker_timeout()
        elif self.state == self.STATE_FINAL_APPROACH:
            self.run_final_approach()
        elif self.state == self.STATE_UNDOCKING:
            self.run_undocking()

    def run_final_approach(self):
        """After 30 cm alignment: forward only, NO turning or re-alignment."""
        now = self.get_clock().now()
        if not self.marker_is_fresh():
            self.handle_marker_loss()
            return
        if self.final_started is None:
            self.final_started = now
        if (now - self.final_started).nanoseconds / 1e9 > self.final_max_sec:
            self.stop()
            self.is_docking_active = False
            self.set_state('FINAL_APPROACH_TIMEOUT', False)
            return

        # Camera z is distance to the marker, not a bumper/charger clearance.
        if self.last_marker_z <= self.dock_camera_distance:
            self.stop()
            self.is_docking_active = False
            self.publish_detector_enable(False)
            self.set_state(self.STATE_DOCKED, True)
            self.get_logger().info(
                f'[DOCKED] camera marker_z={self.last_marker_z:.3f}m; '
                'no electrical-contact verification')
            return

        # Never turn here: x/yaw errors must not trigger realign/backoff/rotation.
        # Stop if the measured camera distance is not decreasing while commanded.
        if self.progress_start_time is None:
            self.progress_start_time = now
            self.progress_z_start = self.last_marker_z
        elif self.progress_z_start - self.last_marker_z >= self.progress_min_delta:
            self.progress_start_time = now
            self.progress_z_start = self.last_marker_z
        elif (now - self.progress_start_time).nanoseconds / 1e9 >= self.progress_timeout_sec:
            self.stop()
            self.is_docking_active = False
            self.set_state('FINAL_NO_FORWARD_PROGRESS', False)
            return

        cmd = Twist()
        cmd.linear.x = abs(self.final_approach_speed)
        cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)
        if (now - self.last_control_log).nanoseconds / 1e9 >= 0.5:
            self.get_logger().info(
                f'[FINAL_STRAIGHT] marker_z={self.last_marker_z:.3f}m '
                f'vx={cmd.linear.x:+.3f} wz=0.000')
            self.last_control_log = now

    def run_undocking(self):
        now = self.get_clock().now()
        if not self.marker_is_fresh():
            self.stop()
            self.publish_docking_status('UNDOCK_WAIT_MARKER', False)
            return
        if self.last_marker_z >= self.undock_camera_distance:
            self.stop()
            self.publish_detector_enable(False)
            self.set_state(self.STATE_STANDBY, False)
            self.get_logger().info(
                f'Undocking complete: camera marker_z={self.last_marker_z:.3f}m')
            return
        if (now - self.undock_started).nanoseconds / 1e9 > self.undock_max_sec:
            self.stop()
            self.publish_docking_status('UNDOCK_TIMEOUT', False)
            return
        self.reverse_command(self.undock_speed)

    def handle_marker_loss(self):
        self.stop()
        if not self.marker_missing_stopped:
            self.marker_missing_stopped = True
            self.get_logger().warn(
                'Marker lost; stopping camera-only docking (no blind forward/turn)')

        self.publish_docking_status('MARKER_LOST_STOPPED', False)

    def check_marker_timeout(self):
        if not self.is_docking_active:
            return
        missing = (self.get_clock().now() - self.last_pose_time).nanoseconds / 1e9
        if missing > self.marker_stale_sec:
            self.handle_marker_loss()

    def pose_callback(self, msg):
        if self.state not in (
            self.STATE_ALIGNING,
            self.STATE_FINAL_APPROACH,
            self.STATE_UNDOCKING,
        ):
            return
        if not self.is_docking_active and self.state != self.STATE_UNDOCKING:
            return
        p, q = msg.pose.position, msg.pose.orientation
        if not all(math.isfinite(v) for v in (p.x, p.z, q.x, q.y, q.z, q.w)):
            self.stop()
            return
        self.last_pose_time = self.get_clock().now()
        if p.z <= 0.0:
            self.stop()
            return
        self.last_marker_z = p.z
        self.last_marker_x = p.x
        self.last_marker_yaw = self.yaw_from_quaternion(q)
        self.marker_missing_stopped = False
        # During straight final approach and undock, update camera distance only.
        # No alignment/rotation command is issued by this callback.
        if self.state in (
            self.STATE_FINAL_APPROACH,
            self.STATE_UNDOCKING,
        ):
            return

        error_z = p.z - self.target_z
        error_x = p.x - self.target_x
        error_yaw = math.atan2(
            math.sin(self.last_marker_yaw - self.target_yaw),
            math.cos(self.last_marker_yaw - self.target_yaw))

        if (abs(error_z) <= self.tol_z and abs(error_x) <= self.tol_x
                and abs(error_yaw) <= self.tol_yaw):
            self.stop()
            # Keep detector enabled throughout camera-only approach.
            self.realign_backoff_started = None
            self.realign_backoff_blocked = False
            self.final_started = None
            self.progress_z_start = None
            self.progress_start_time = None
            self.set_state(self.STATE_FINAL_APPROACH, False)
            self.get_logger().info('30 cm alignment reached; straight-only approach, angular.z=0')
            return

        cmd = Twist()
        if abs(error_z) > self.linear_stop_band and abs(error_yaw) <= self.approach_yaw_gate:
            requested = self.k_linear * error_z
            if p.z <= self.min_safe_marker_z and requested > 0.0:
                requested = 0.0
            cmd.linear.x = self.limited_nonzero(
                requested, self.min_linear_vel, self.max_linear_vel)
        if abs(error_yaw) > self.angular_stop_band or abs(error_x) > self.tol_x:
            cmd.angular.z = self.limited_nonzero(
                -self.x_control_sign * self.k_angular * error_x -self.yaw_control_sign * self.k_yaw * error_yaw,
                self.min_angular_vel, self.max_angular_vel)
        # Too close: never rotate next to station; reverse to regain clearance.
        if p.z < self.target_z and cmd.angular.z != 0.0:
            cmd.angular.z = 0.0
            # Reverse bounded by time, with a stop if clearance does not recover.
            if self.realign_backoff_blocked:
                cmd.linear.x = 0.0
            else:
                if self.realign_backoff_started is None:
                    self.realign_backoff_started = self.get_clock().now()
                elapsed = (self.get_clock().now() - self.realign_backoff_started).nanoseconds / 1e9
                if elapsed >= self.realign_backoff_max_sec:
                    self.realign_backoff_blocked = True
                    cmd.linear.x = 0.0
                else:
                    cmd.linear.x = -abs(self.recovery_speed)
        else:
            self.realign_backoff_started = None
            self.realign_backoff_blocked = False
            if abs(error_yaw) > self.approach_yaw_gate or abs(cmd.angular.z) > self.max_linear_angular_mix:
                cmd.linear.x = 0.0
        if cmd.linear.x == 0 and cmd.angular.z == 0:
            self.stop()
            self.publish_docking_status('ALIGNMENT_LIMIT', False)
            return
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDockerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.publish_detector_enable(False)
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()