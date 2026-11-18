#!/usr/bin/env python3
"""ArUco + LiDAR docking controller.

Sequence
1) ArUco pose alignment at 0.30 m.
   - Forward AND reverse corrections are allowed.
2) Keep ArUco corrections while visible; optional verified LiDAR-only terminal approach to 0.16 m.
3) When /toggle_docking false is requested, reverse until front LiDAR distance >= 0.45 m.

Test carefully with downstream E-stop / velocity limits enabled.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import LaserScan
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
    STATE_REALIGN_BACKOFF = 'REALIGN_BACKOFF'
    STATE_REALIGN_TURN = 'REALIGN_TURN'
    STATE_LIDAR_CLOSE = 'LIDAR_CLOSE_APPROACH'

    def __init__(self):
        super().__init__('aruco_docker_node')

        self.state = self.STATE_STANDBY
        self.is_docking_active = False
        self.is_searching_marker = False

        self.last_pose_time = self.get_clock().now()
        self.last_scan_time = self.get_clock().now()
        self.search_start_time = None
        self.marker_missing_stopped = False

        self.front_distance = None
        self.rear_distance = None
        self.last_marker_z = None
        self.last_marker_x = None
        self.last_marker_yaw = None
        self.recovery_started = None
        self.recovery_attempted = False
        self.realign_backoff_started = None
        self.realign_backoff_blocked = False
        self.last_guidance_log = self.get_clock().now()
        self.final_started = None
        self.final_start_front = None
        self.last_control_log = self.get_clock().now()
        self.previous_final_cmd = Twist()
        self.realign_started = None
        self.realign_attempts = 0
        self.realign_heading_target = None
        self.realign_best_z = None
        self.realign_lost_at = None
        self.progress_z_start = None
        self.progress_start_time = None
        self.realign_guidance_until = None
        self.close_start_time = None
        self.close_start_front = None
        self.close_last_front = None
        self.close_last_progress_time = None
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
        self.k_angular = float(self.declare_parameter('k_angular', 0.4).value)
        self.k_yaw = float(self.declare_parameter('k_yaw', 0.8).value)
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
            self.declare_parameter('min_safe_marker_z', 0.18).value
        )

        # ---------------------------
        # LiDAR final approach / retreat
        # ---------------------------
        self.scan_topic = str(self.declare_parameter('scan_topic', '/scan').value)
        self.front_sector_deg = float(
            self.declare_parameter('front_sector_deg', 10.0).value
        )

        self.dock_lidar_distance = float(
            self.declare_parameter('dock_lidar_distance', 0.16).value
        )
        self.undock_lidar_distance = float(
            self.declare_parameter('undock_lidar_distance', 0.45).value
        )

        self.final_approach_speed = float(
            self.declare_parameter('final_approach_speed', 0.10).value
        )
        self.undock_speed = float(
            self.declare_parameter('undock_speed', 0.10).value
        )

        self.rear_clearance = float(self.declare_parameter('rear_clearance', 0.35).value)
        self.recovery_speed = float(self.declare_parameter('recovery_speed', 0.10).value)
        self.recovery_max_sec = float(self.declare_parameter('recovery_max_sec', 1.0).value)
        # Rear LiDAR requirement disabled as requested. Reverse still requires fresh front LiDAR.
        # IMPORTANT: physical rear collision protection must be verified separately.
        self.require_rear_scan = bool(self.declare_parameter('require_rear_scan', False).value)
        self.final_yaw_gate = float(self.declare_parameter('final_yaw_gate', 0.07).value)
        self.final_x_gate = float(self.declare_parameter('final_x_gate', 0.05).value)
        # Begin gradual steering well before the hard re-alignment threshold.
        self.final_x_soft = float(self.declare_parameter('final_x_soft', 0.015).value)
        self.final_yaw_soft = float(self.declare_parameter('final_yaw_soft', 0.025).value)
        self.final_steer_x_gain = float(self.declare_parameter('final_steer_x_gain', 0.5).value)
        self.final_steer_yaw_gain = float(self.declare_parameter('final_steer_yaw_gain', 0.8).value)
        self.final_steer_max = float(self.declare_parameter('final_steer_max', 0.08).value)
        self.final_steer_min = float(self.declare_parameter('final_steer_min', 0.0).value)
        # Expose x sign separately; do not assume the observed error proves sign.
        self.x_control_sign = float(self.declare_parameter('x_control_sign', 1.0).value)
        self.final_max_sec = float(self.declare_parameter('final_max_sec', 30.0).value)
        # Prevent camera-to-marker clearance being exhausted while LiDAR sees another object.
        self.final_marker_floor = float(self.declare_parameter('final_marker_floor', 0.05).value)
        # One calibration point (LiDAR 0.28m ~= physical 0.15m) is NOT enough
        # to certify the 0.16m endpoint. Require deliberate opt-in after physical
        # verification of sensor geometry, braking distance, and collision safety.
        self.enable_close_approach = bool(
            self.declare_parameter('enable_close_approach', False).value)
        self.close_entry_lidar = float(
            self.declare_parameter('close_entry_lidar', 0.30).value)
        self.close_linear_speed = float(
            self.declare_parameter('close_linear_speed', 0.04).value)
        self.close_max_sec = float(
            self.declare_parameter('close_max_sec', 8.0).value)
        self.close_max_travel = float(
            self.declare_parameter('close_max_travel', 0.14).value)
        self.close_progress_sec = float(
            self.declare_parameter('close_progress_sec', 2.5).value)
        self.close_progress_delta = float(
            self.declare_parameter('close_progress_delta', 0.008).value)

        # Clearance for turning is measured using ArUco camera-to-marker z, NOT LiDAR.
        self.realign_turn_marker_z = float(self.declare_parameter('realign_turn_marker_z', 0.36).value)
        self.realign_backoff_max_sec = float(self.declare_parameter('realign_backoff_max_sec', 10000.0).value)
        # Commanded-distance cap is an additional limit, NOT proof of physical travel.
        self.realign_backoff_max_commanded_m = float(self.declare_parameter('realign_backoff_max_commanded_m', 0.45).value)
        self.realign_max_attempts = int(self.declare_parameter('realign_max_attempts', 10).value)
        self.realign_heading_gain = float(self.declare_parameter('realign_heading_gain', 0.35).value)
        # Flip only the lateral-correction heading in REALIGN_TURN; do not invert yaw feedback.
        self.realign_heading_sign = float(self.declare_parameter('realign_heading_sign', -1.0).value)
        self.realign_heading_max_deg = float(self.declare_parameter('realign_heading_max_deg', 8.0).value)
        self.realign_turn_timeout = float(self.declare_parameter('realign_turn_timeout', 2.5).value)
        self.realign_turn_speed = float(self.declare_parameter('realign_turn_speed', 0.05).value)
        self.realign_turn_yaw_tol_deg = float(self.declare_parameter('realign_turn_yaw_tol_deg', 2.5).value)
        self.progress_timeout_sec = float(self.declare_parameter('progress_timeout_sec', 4.0).value)
        self.progress_min_delta = float(self.declare_parameter('progress_min_delta', 0.015).value)
        # No rear scan in the user-approved clear test zone; bounded reverse ONLY.
        self.require_rear_scan = False
        self.scan_stale_sec = float(
            self.declare_parameter('scan_stale_sec', 0.50).value
        )

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
        if self.realign_heading_sign not in (-1.0, 1.0):
            raise ValueError('realign_heading_sign must be +1 or -1')
        if not 0 <= self.final_steer_min <= self.final_steer_max:
            raise ValueError('final steering speed limits invalid')
        if not 0 < self.final_marker_floor < self.target_z:
            raise ValueError('final_marker_floor must be between 0 and target_z')
        if self.realign_backoff_max_sec <= 0 or self.realign_backoff_max_commanded_m <= 0:
            raise ValueError('realign backoff limits must be positive')
        if self.realign_turn_marker_z <= self.target_z:
            raise ValueError('realign_turn_marker_z must exceed target_z')
        if self.dock_lidar_distance <= 0.0:
            raise ValueError('dock_lidar_distance must be > 0')
        if not (self.dock_lidar_distance < self.close_entry_lidar <= 0.35):
            raise ValueError('Require dock_lidar_distance < close_entry_lidar <= 0.35m')
        if not (0 < self.close_linear_speed <= 0.10):
            raise ValueError('close_linear_speed must be in (0, 0.10]m/s')
        if self.close_max_sec <= 0 or self.close_max_travel <= 0:
            raise ValueError('close limits must be positive')

        if self.undock_lidar_distance <= self.dock_lidar_distance:
            raise ValueError(
                'undock_lidar_distance must be greater than dock_lidar_distance'
            )

        # ---------------------------
        # ROS I/O
        # ---------------------------
        self.pose_sub = self.create_subscription(
            PoseStamped, '/aruco_pose', self.pose_callback, 10
        )

        # SensorDataQoS-compatible profile for common /scan publishers.
        scan_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self.scan_sub = self.create_subscription(
            LaserScan, self.scan_topic, self.scan_callback, scan_qos
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
            'ArUco docker ready: align@0.30m -> LiDAR 0.15m -> '
            'undock reverse to LiDAR 0.45m'
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
    # LiDAR
    # ------------------------------------------------------------------
    def scan_callback(self, msg):
        values = []
        rear_values = []
        half_sector = math.radians(self.front_sector_deg)

        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r):
                continue

            angle = msg.angle_min + i * msg.angle_increment

            # Normalize to [-pi, pi]
            angle = math.atan2(math.sin(angle), math.cos(angle))

            if msg.range_min <= r <= msg.range_max:
                if abs(angle) <= half_sector:
                    values.append(r)
                if abs(abs(angle) - math.pi) <= half_sector:
                    rear_values.append(r)

        self.rear_distance = min(rear_values) if rear_values else None
        if not values:
            self.front_distance = None
            return

        # Use minimum distance in the front sector for conservative stopping.
        self.front_distance = min(values)
        self.last_scan_time = self.get_clock().now()

    def lidar_is_fresh(self):
        if self.front_distance is None:
            return False
        age = (
            self.get_clock().now() - self.last_scan_time
        ).nanoseconds / 1e9
        return age <= self.scan_stale_sec

    def reverse_clear(self):
        if not self.lidar_is_fresh():
            return False
        if not self.require_rear_scan:
            return True  # Only use with independent rear obstacle protection.
        return self.rear_distance is not None and self.rear_distance > self.rear_clearance

    def reverse_command(self, speed):
        if not self.reverse_clear():
            self.stop()
            self.publish_docking_status('REVERSE_BLOCKED_REAR', False)
            return False
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
            self.recovery_started = None
            self.recovery_attempted = False
            self.realign_backoff_started = None
            self.realign_backoff_blocked = False
            self.last_marker_z = None
            self.final_started = None
            self.final_start_front = None
            self.close_start_time = None
            self.close_start_front = None
            self.close_last_front = None
            self.close_last_progress_time = None
            self.undock_started = None
            self.realign_started = None
            self.realign_attempts = 0
            self.realign_heading_target = None
            self.realign_guidance_until = None
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

        # OFF means undock/retreat, not immediate deactivate.
        self.stop()
        self.is_docking_active = False
        self.is_searching_marker = False
        self.search_start_time = None
        self.publish_detector_enable(False)

        if not self.lidar_is_fresh():
            self.set_state('UNDOCK_WAIT_LIDAR', False)
            response.success = False
            response.message = (
                'Docking OFF requested, but no fresh front LiDAR data is available'
            )
            return response

        # Already farther than 45 cm: no need to move.
        if self.front_distance >= self.undock_lidar_distance:
            self.set_state(self.STATE_STANDBY, False)
            response.success = True
            response.message = (
                f'Already clear: front LiDAR {self.front_distance:.3f} m '
                f'>= {self.undock_lidar_distance:.3f} m'
            )
            return response

        self.undock_started = self.get_clock().now()
        self.set_state(self.STATE_UNDOCKING, False)
        response.success = True
        response.message = (
            f'Undocking: reverse until front LiDAR >= '
            f'{self.undock_lidar_distance:.2f} m'
        )
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
        elif self.state == self.STATE_REALIGN_BACKOFF:
            self.run_realign_backoff()
        elif self.state == self.STATE_REALIGN_TURN:
            self.run_realign_turn()
        elif self.state == self.STATE_LIDAR_CLOSE:
            self.run_lidar_close()

    def run_final_approach(self):
        """Bounded approach: continuous small corrections, safe retreat for big errors."""
        now = self.get_clock().now()
        if not self.lidar_is_fresh():
            self.stop()
            self.publish_docking_status('FINAL_APPROACH_WAIT_LIDAR', False)
            return

        missing = (now - self.last_pose_time).nanoseconds / 1e9
        if missing > self.marker_stale_sec:
            self.handle_marker_loss()
            return

        if (self.last_marker_z is None or self.last_marker_x is None
                or self.last_marker_yaw is None):
            self.stop()
            self.publish_docking_status('WAIT_MARKER_POSE', False)
            return

        if self.final_started is None:
            self.final_started = now
            self.final_start_front = self.front_distance
        if (now - self.final_started).nanoseconds / 1e9 > self.final_max_sec:
            self.stop()
            self.publish_docking_status('FINAL_APPROACH_TIMEOUT', False)
            return

        error_x = self.last_marker_x - self.target_x
        error_yaw = math.atan2(
            math.sin(self.last_marker_yaw - self.target_yaw),
            math.cos(self.last_marker_yaw - self.target_yaw))
        misaligned = abs(error_x) > self.final_x_gate or abs(error_yaw) > self.final_yaw_gate

        if (now - self.last_guidance_log).nanoseconds / 1e9 >= 0.5:
            self.get_logger().info(
                f'[GUIDANCE] phase=FINAL marker_z={self.last_marker_z:.3f} '
                f'front={self.front_distance:.3f} x_err={error_x:+.3f} '
                f'yaw_err_deg={math.degrees(error_yaw):+.1f} '
                f'xsign={self.x_control_sign:+.0f} yawsign={self.yaw_control_sign:+.0f} '
                f'misaligned={misaligned}')
            self.last_guidance_log = now

        # A LiDAR range threshold is not proof that docking is aligned.
        # Never report completion while the ArUco correction is out of tolerance.
        if self.front_distance <= self.dock_lidar_distance:
            self.stop()
            if self.last_marker_z <= self.final_marker_floor and not self.enable_close_approach:
                self.publish_docking_status('CLOSE_APPROACH_NOT_VERIFIED', False)
                return
            if misaligned:
                self.publish_docking_status('DOCK_DISTANCE_BUT_MISALIGNED', False)
                return
            self.is_docking_active = False
            self.publish_detector_enable(False)
            self.set_state(self.STATE_DOCKED, True)
            self.get_logger().info(
                f'Docking distance reached, LiDAR={self.front_distance:.3f} m; '
                'this does not confirm electrical charging/contact')
            return

        # Camera distance floor is approaching: near the station the marker
        # can leave the camera FOV. Only switch to short straight LiDAR-only
        # approach AFTER alignment was verified, and only when opt-in enabled.
        if self.last_marker_z <= self.final_marker_floor:
            self.stop()
            if not misaligned and self.front_distance <= self.close_entry_lidar:
                if not self.enable_close_approach:
                    self.publish_docking_status('CLOSE_APPROACH_NOT_VERIFIED', False)
                    self.get_logger().warn(
                        '[CLOSE] Calibration not independently verified. STOP; '
                        'do not enable close approach until physically checked',
                        throttle_duration_sec=2.0)
                    return
                self.close_start_time = now
                self.close_start_front = self.front_distance
                self.close_last_front = self.front_distance
                self.close_last_progress_time = now
                self.set_state(self.STATE_LIDAR_CLOSE, False)
                self.get_logger().warn(
                    f'[CLOSE] camera floor {self.last_marker_z:.3f}m; '
                    f'LiDAR-only straight approach from {self.front_distance:.3f}m '
                    f'to {self.dock_lidar_distance:.3f}m; speed '
                    f'{self.close_linear_speed:.3f}m/s')
                return
            self.publish_docking_status('FINAL_MARKER_CLEARANCE_LIMIT', False)
            return

        # The logs show LiDAR front range can *increase* as marker_z decreases.
        # Do not continue a blind approach when the camera-marker clearance runs out.

        # After backing out and turning, allow a SHORT guided approach to
        # translate lateral x error. A stationary turn cannot correct x.
        guidance_active = (self.realign_guidance_until is not None and
                           (now - self.realign_guidance_until).nanoseconds < 0)
        too_close_to_correct = self.last_marker_z < self.target_z + 0.01
        if misaligned and (too_close_to_correct or not guidance_active):
            self.stop()
            self.begin_realign('final x/yaw error')
            return

        # Motion watchdog: observed 7s of vx=0.10 with marker_z unchanged.
        # Without wheel odometry, marker_z is a conservative progress proxy.
        if self.progress_start_time is None:
            self.progress_start_time = now
            self.progress_z_start = self.last_marker_z
        elif (self.progress_z_start - self.last_marker_z) >= self.progress_min_delta:
            self.progress_start_time = now
            self.progress_z_start = self.last_marker_z
        elif (now - self.progress_start_time).nanoseconds / 1e9 >= self.progress_timeout_sec:
            self.stop()
            self.is_docking_active = False
            self.set_state('FINAL_NO_FORWARD_PROGRESS', False)
            self.get_logger().error(
                '[FINAL] commanded forward but marker_z did not decrease '
                f'by {self.progress_min_delta:.3f}m within '
                f'{self.progress_timeout_sec:.1f}s; STOP')
            return

        # Small errors: combine low-gain steering with forward travel rather
        # than driving with angular.z=0 until x error exceeds 5 cm.
        self.realign_backoff_started = None
        self.realign_backoff_blocked = False
        cmd = Twist()
        cmd.linear.x = abs(self.final_approach_speed)
        x_term = 0.0 if abs(error_x) <= self.final_x_soft else error_x
        yaw_term = 0.0 if abs(error_yaw) <= self.final_yaw_soft else error_yaw
        raw_wz = (-self.x_control_sign * self.final_steer_x_gain * x_term
                  -self.yaw_control_sign * self.final_steer_yaw_gain * yaw_term)
        cmd.angular.z = max(-self.final_steer_max,
                            min(self.final_steer_max, raw_wz))
        if (abs(error_x) > 0.03 or abs(error_yaw) > math.radians(2.0)) and 0 < abs(cmd.angular.z) < self.final_steer_min:
            cmd.angular.z = math.copysign(self.final_steer_min, cmd.angular.z)
        self.cmd_pub.publish(cmd)
        if (now - self.last_control_log).nanoseconds / 1e9 >= 0.5:
            self.get_logger().info(
                f'[FINAL_CONTROL] marker_z={self.last_marker_z:.3f} '
                f'front={self.front_distance:.3f} '
                f'x={error_x:+.3f} yaw={math.degrees(error_yaw):+.1f}deg '
                f'vx={cmd.linear.x:+.3f} wz={cmd.angular.z:+.3f}')
            self.last_control_log = now

    def run_lidar_close(self):
        """Short, straight terminal approach; no rotation with lost marker FOV.

        This is only allowed with explicit physical calibration/safety approval.
        There is no attempt to infer charger contact from range alone.
        """
        now = self.get_clock().now()
        if not self.enable_close_approach:
            self.stop()
            self.set_state('CLOSE_APPROACH_DISABLED', False)
            self.is_docking_active = False
            return
        if not self.lidar_is_fresh():
            self.stop()
            self.set_state('CLOSE_LIDAR_STALE_STOP', False)
            self.is_docking_active = False
            return
        if self.front_distance <= self.dock_lidar_distance:
            self.stop()
            self.is_docking_active = False
            self.publish_detector_enable(False)
            self.set_state(self.STATE_DOCKED, True)
            self.get_logger().info(
                f'[CLOSE] target reached: lidar={self.front_distance:.3f}m; '
                'range only, no electrical contact proof')
            return
        elapsed = (now - self.close_start_time).nanoseconds / 1e9
        travel = max(0.0, self.close_start_front - self.front_distance)
        if elapsed > self.close_max_sec or travel >= self.close_max_travel:
            self.stop()
            self.set_state('CLOSE_TRAVEL_OR_TIME_LIMIT', False)
            self.is_docking_active = False
            return
        if self.close_last_front - self.front_distance >= self.close_progress_delta:
            self.close_last_front = self.front_distance
            self.close_last_progress_time = now
        if (now - self.close_last_progress_time).nanoseconds / 1e9 > self.close_progress_sec:
            self.stop()
            self.set_state('CLOSE_NO_LIDAR_PROGRESS', False)
            self.is_docking_active = False
            return
        cmd = Twist()
        cmd.linear.x = self.close_linear_speed
        cmd.angular.z = 0.0   # no blind rotation while marker may be out of view
        self.cmd_pub.publish(cmd)
        self.get_logger().info(
            f'[CLOSE] front={self.front_distance:.3f}m '
            f'target={self.dock_lidar_distance:.3f}m '
            f'delta={travel:.3f}m vx={cmd.linear.x:.3f} wz=0',
            throttle_duration_sec=0.5)

    def begin_realign(self, why):
        """Stop first; do not rotate near the station."""
        self.stop()
        if self.realign_attempts >= self.realign_max_attempts:
            self.set_state('REALIGN_ATTEMPT_LIMIT', False)
            self.is_docking_active = False
            self.get_logger().error('[REALIGN] attempt limit, stopped')
            return
        self.realign_attempts += 1
        self.realign_started = self.get_clock().now()
        self.realign_heading_target = None
        self.progress_z_start = None
        self.progress_start_time = None
        self.set_state(self.STATE_REALIGN_BACKOFF, False)
        self.get_logger().warn(
            f'[REALIGN] {why}; reverse straight toward marker_z '
            f'{self.realign_turn_marker_z:.2f}m, '
            f'max {self.realign_backoff_max_sec:.1f}s, attempt '
            f'{self.realign_attempts}/{self.realign_max_attempts}')

    def run_realign_backoff(self):
        """Reverse straight; never turn while near the charger."""
        now = self.get_clock().now()
        elapsed = (now - self.realign_started).nanoseconds / 1e9
        marker_fresh = (now - self.last_pose_time).nanoseconds / 1e9 <= self.marker_stale_sec
        if not self.lidar_is_fresh():
            self.stop()
            self.set_state('REALIGN_WAIT_LIDAR', False)
            self.is_docking_active = False
            return
        if marker_fresh and self.last_marker_z is not None and self.last_marker_z >= self.realign_turn_marker_z:
            self.stop()
            self.realign_started = now
            self.set_state(self.STATE_REALIGN_TURN, False)
            return
        # Never allow longer unverified reverse just because marker data is lost.
        # The commanded-distance bound is independent of marker recognition.
        commanded_m = abs(self.recovery_speed) * elapsed
        if elapsed >= self.realign_backoff_max_sec or commanded_m >= self.realign_backoff_max_commanded_m:
            self.stop()
            # Don't turn with uncertain clearance, especially if the camera lost the marker.
            self.get_logger().error(
                f'[REALIGN] backoff limit (time={elapsed:.2f}s, commanded={commanded_m:.3f}m); marker_z={self.last_marker_z}; '
                'stopped instead of rotating blindly')
            self.set_state('REALIGN_BACKOFF_TIMEOUT', False)
            self.is_docking_active = False
            return
        self.reverse_command(self.recovery_speed)
        self.get_logger().info(
            f'[REALIGN_BACKOFF] t={elapsed:.1f}s commanded={commanded_m:.3f}m marker_z={self.last_marker_z} '
            f'fresh={marker_fresh} pose_age={(now - self.last_pose_time).nanoseconds / 1e9:.2f}s '
            f'cmd_vx=-{self.recovery_speed:.2f}',
            throttle_duration_sec=0.5)

    def run_realign_turn(self):
        """Make a small heading correction at clearance, then drive to reduce x.

        A stationary turn alone cannot remove lateral x error. The heading
        offset is followed by a short forward guidance segment.
        """
        now = self.get_clock().now()
        age = (now - self.last_pose_time).nanoseconds / 1e9
        if age > self.marker_stale_sec:
            self.stop()
            # A fresh marker is mandatory for autonomous rotation.
            self.set_state('REALIGN_MARKER_LOST_STOP', False)
            self.is_docking_active = False
            return
        if self.last_marker_z is None or self.last_marker_z < self.realign_turn_marker_z - 0.025:
            self.stop()
            self.set_state('REALIGN_CLEARANCE_LOST_STOP', False)
            self.is_docking_active = False
            return
        elapsed = (now - self.realign_started).nanoseconds / 1e9
        if elapsed > self.realign_turn_timeout:
            self.stop()
            self.set_state('REALIGN_TURN_TIMEOUT', False)
            self.is_docking_active = False
            return
        x_error = self.last_marker_x - self.target_x
        yaw_error = math.atan2(math.sin(self.last_marker_yaw - self.target_yaw),
                               math.cos(self.last_marker_yaw - self.target_yaw))
        # Relative heading offset: useful for translating lateral x while
        # advancing. Sign follows the existing x-control convention.
        target_offset = max(-math.radians(self.realign_heading_max_deg),
                            min(math.radians(self.realign_heading_max_deg),
                                self.realign_heading_sign * self.x_control_sign *
                                self.realign_heading_gain *
                                math.atan2(x_error, max(self.last_marker_z, 0.10))))
        target_yaw = self.target_yaw + target_offset
        steer_error = math.atan2(math.sin(self.last_marker_yaw - target_yaw),
                                math.cos(self.last_marker_yaw - target_yaw))
        if abs(steer_error) <= math.radians(self.realign_turn_yaw_tol_deg):
            self.stop()
            self.realign_guidance_until = now + Duration(seconds=2.0)
            self.progress_z_start = None
            self.progress_start_time = None
            self.set_state(self.STATE_FINAL_APPROACH, False)
            self.get_logger().info(
                f'[REALIGN] heading established; x={x_error:+.3f} '
                f'yaw={math.degrees(yaw_error):+.1f}deg; resume guided approach')
            return
        cmd = Twist()
        cmd.angular.z = self.limited_nonzero(
            -self.yaw_control_sign * self.k_yaw * steer_error,
            self.min_angular_vel, self.realign_turn_speed)
        self.cmd_pub.publish(cmd)
        self.get_logger().info(
            f'[REALIGN_TURN] x={x_error:+.3f} '
            f'yaw={math.degrees(yaw_error):+.1f}deg '
            f'target_offset={math.degrees(target_offset):+.1f}deg '
            f'heading_sign={self.realign_heading_sign:+.0f} '
            f'wz={cmd.angular.z:+.3f}',
            throttle_duration_sec=0.5)

    def run_undocking(self):
        if not self.lidar_is_fresh():
            self.stop()
            self.publish_docking_status('UNDOCK_WAIT_LIDAR', False)
            return
        if self.front_distance >= self.undock_lidar_distance:
            self.stop()
            self.set_state(self.STATE_STANDBY, False)
            self.get_logger().info(f'Undocking completed by LiDAR: {self.front_distance:.3f} m')
            return
        self.reverse_command(self.undock_speed)

    def handle_marker_loss(self):
        self.stop()
        if not self.marker_missing_stopped:
            self.marker_missing_stopped = True
            self.get_logger().warn(
                f'Marker lost, front LiDAR={self.front_distance}; no blind rotation')

        # Near the charger, back away in a bounded one-shot recovery attempt.
        # More than one attempt without a new marker detection is forbidden.
        near = (self.last_marker_z is not None and
                self.last_marker_z < self.realign_turn_marker_z)
        if not near:
            self.publish_docking_status('MARKER_LOST_STOPPED', False)
            return
        if self.state == self.STATE_FINAL_APPROACH:
            self.begin_realign('marker lost close to station')
            return
        if self.recovery_attempted:
            self.publish_docking_status('MARKER_LOST_RECOVERY_FAILED', False)
            return
        if self.recovery_started is None:
            self.recovery_started = self.get_clock().now()
        elapsed = (self.get_clock().now() - self.recovery_started).nanoseconds / 1e9
        if elapsed >= self.recovery_max_sec:
            self.stop()
            self.recovery_attempted = True
            self.publish_docking_status('MARKER_LOST_RECOVERY_FAILED', False)
            return
        self.publish_docking_status('MARKER_LOST_REVERSING', False)
        self.reverse_command(self.recovery_speed)

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
            self.STATE_REALIGN_BACKOFF,
            self.STATE_REALIGN_TURN,
            self.STATE_LIDAR_CLOSE,
        ):
            return
        if not self.is_docking_active:
            return
        p, q = msg.pose.position, msg.pose.orientation
        if not all(math.isfinite(v) for v in (p.x, p.z, q.x, q.y, q.z, q.w)):
            self.stop()
            return
        self.last_pose_time = self.get_clock().now()
        self.last_marker_z = p.z
        self.last_marker_x = p.x
        self.last_marker_yaw = self.yaw_from_quaternion(q)
        self.marker_missing_stopped = False
        # Backoff and turn MUST refresh marker pose, but must not issue ALIGNING
        # commands from the callback. Control runs in the 10 Hz timer.
        if self.state in (
            self.STATE_FINAL_APPROACH,
            self.STATE_REALIGN_BACKOFF,
            self.STATE_REALIGN_TURN,
            self.STATE_LIDAR_CLOSE,
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
            # Crucial: keep detector enabled during LiDAR approach.
            self.realign_backoff_started = None
            self.realign_backoff_blocked = False
            self.final_started = None
            self.final_start_front = None
            self.progress_z_start = None
            self.progress_start_time = None
            self.set_state(self.STATE_FINAL_APPROACH, False)
            self.get_logger().info('30 cm alignment reached; continuous ArUco guidance enabled')
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
        if cmd.linear.x < 0 and not self.reverse_clear():
            self.stop()
            self.publish_docking_status('REVERSE_BLOCKED_REAR', False)
            return
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