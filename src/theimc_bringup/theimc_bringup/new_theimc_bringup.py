#!/usr/bin/env python3

import math
import serial
import sys

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import BatteryState, Imu, LaserScan
from std_msgs.msg import Bool, Float32, Float32MultiArray, String, UInt32


def quaternion_from_euler(roll, pitch, yaw):
    """Convert roll/pitch/yaw [rad] to quaternion [x, y, z, w]."""
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    return [
        cy * cp * sr - sy * sp * cr,
        sy * cp * sr + cy * sp * cr,
        sy * cp * cr - cy * sp * sr,
        cy * cp * cr + sy * sp * sr,
    ]


class BringUp(Node):
    """
    ROS/STM bringup after Beckhoff migration.

    STM32 role:
      - IMU / TOF receive
      - TABOS battery BMS CAN values -> serial -> ROS
      - TABOS WPT RX CAN values -> serial -> ROS
      - ROS /charger/command -> serial -> STM32 -> WPT CAN2.0A

    Base drive:
      /cmd_vel -> agv_ads_driver directly
      (this node does NOT subscribe to /cmd_vel)

    Rail:
      /rail_command (FORWARD/BACK/STOP)
          -> safety/state logic in this node
          -> /rail/cmd_speed at 10 Hz
          -> agv_ads_driver
          -> Beckhoff ADS

      /rail/speed_feedback
          <- agv_ads_driver
          -> /rail_speed + /rail/odom
    """

    def __init__(self):
        super().__init__('bring_up')

        # ------------------------------------------------------------
        # Parameters
        # ------------------------------------------------------------
        self.declare_parameter('serial_port', '/dev/stm32_link')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('serial_timeout', 0.1)

        self.declare_parameter('rail_forward_speed', 0.2)
        self.declare_parameter('rail_back_speed', 0.2)
        self.declare_parameter('rail_command_rate_hz', 10.0)

        # Direction reversal interlock:
        # FORWARD -> STOP(confirmed by feedback) -> BACK
        # BACK    -> STOP(confirmed by feedback) -> FORWARD
        self.declare_parameter('rail_reverse_stop_threshold_mps', 0.01)

        self.declare_parameter('rail_max_distance_m', 20.0)
        self.declare_parameter('rail_end_release_margin_m', 0.10)

        self.declare_parameter('rail_obstacle_distance', 0.5)
        self.declare_parameter('rail_obstacle_fov_deg', 45.0)

        self.serial_port = str(self.get_parameter('serial_port').value)
        self.baud_rate = int(self.get_parameter('baud_rate').value)
        self.serial_timeout = float(self.get_parameter('serial_timeout').value)

        self.rail_forward_speed = abs(
            float(self.get_parameter('rail_forward_speed').value)
        )
        self.rail_back_speed = abs(
            float(self.get_parameter('rail_back_speed').value)
        )
        self.rail_command_rate_hz = max(
            1.0,
            float(self.get_parameter('rail_command_rate_hz').value),
        )
        self.rail_reverse_stop_threshold_mps = abs(
            float(
                self.get_parameter(
                    'rail_reverse_stop_threshold_mps'
                ).value
            )
        )

        self.rail_max_distance_m = float(
            self.get_parameter('rail_max_distance_m').value
        )
        self.rail_end_release_margin_m = abs(
            float(self.get_parameter('rail_end_release_margin_m').value)
        )

        self.rail_obstacle_distance = float(
            self.get_parameter('rail_obstacle_distance').value
        )
        self.rail_obstacle_fov_deg = float(
            self.get_parameter('rail_obstacle_fov_deg').value
        )

        # ------------------------------------------------------------
        # STM32 serial: sensors only
        # ------------------------------------------------------------
        try:
            self.stm_serial = serial.Serial(
                port=self.serial_port,
                baudrate=self.baud_rate,
                timeout=self.serial_timeout,
            )
            self.stm_serial.reset_input_buffer()
            self.get_logger().info(
                f'Connected to STM32 sensor link '
                f'{self.serial_port} @ {self.baud_rate}'
            )
        except serial.SerialException as error:
            self.get_logger().error(f'Failed to open STM32 serial: {error}')
            sys.exit(1)

        self.max_serial_lines_per_update = 64

        # ------------------------------------------------------------
        # Rail state / command
        # ------------------------------------------------------------
        self.rail_state = 'OUT_RAIL'

        # Command requested by BT/operator.  This value is preserved while
        # temporarily blocked by an obstacle so motion can resume automatically
        # after the obstacle clears.
        self.requested_rail_command = 'STOP'

        # Command actually being emitted to agv_ads_driver.
        self.effective_rail_command = 'STOP'
        self.effective_rail_speed = 0.0

        self.last_rail_motion_direction = 'STOP'

        # Direction reversal safety.
        # A reverse command is held here until ADS feedback confirms
        # that the rail motor has actually stopped.
        self.pending_reverse_command = None
        self.last_measured_rail_velocity = 0.0
        self.variable_rail_speed_mode = False
        self.variable_rail_speed_setpoint = 0.0

        # Rail motor is allowed only while physically on the rail.
        # ENTERING is intentionally excluded: entry motion should not
        # accidentally start the rail motor before ON_RAIL.
        self.rail_motor_allowed_states = {
            'ON_RAIL',
            'WORKING',
            'EXITING',
        }

        # ------------------------------------------------------------
        # Rail odometry / safety
        # ------------------------------------------------------------
        self.rail_odom_distance = 0.0
        self.rail_odom_last_time = None
        self.rail_end_reached = False

        self.rail_obstacle = False
        self.front_rail_obstacle = False
        self.rear_rail_obstacle = False
        self.obstacle_stop_latched = False

        # ------------------------------------------------------------
        # QoS
        # ------------------------------------------------------------
        qos_profile = QoSProfile(
            depth=20,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )

        # ------------------------------------------------------------
        # Subscribers
        # ------------------------------------------------------------
        self.sub_rail_cmd = self.create_subscription(
            String,
            '/rail_command',
            self.cb_rail_cmd_msg,
            qos_profile,
        )

        self.sub_rail_speed_setpoint = self.create_subscription(
            Float32,
            '/rail_speed_setpoint',
            self.cb_rail_speed_setpoint,
            qos_profile,
        )

        self.sub_rail_state = self.create_subscription(
            String,
            '/rail_state',
            self.cb_rail_state_msg,
            qos_profile,
        )

        self.sub_scan = self.create_subscription(
            LaserScan,
            '/scan',
            self.cb_scan,
            qos_profile_sensor_data,
        )

        self.sub_rail_speed_feedback = self.create_subscription(
            Float32MultiArray,
            '/rail/speed_feedback',
            self.cb_rail_speed_feedback,
            qos_profile,
        )

        # WPT charger command from ROS -> STM32 -> CAN2.0A charger
        # Accepted strings: ON, OFF, RESET
        self.sub_charger_command = self.create_subscription(
            String,
            '/charger/command',
            self.cb_charger_command,
            qos_profile,
        )

        # ------------------------------------------------------------
        # Publishers
        # ------------------------------------------------------------
        self.pub_rail_cmd_speed = self.create_publisher(
            Float32,
            '/rail/cmd_speed',
            qos_profile,
        )

        self.pub_imu = self.create_publisher(
            Imu,
            '/imu',
            qos_profile,
        )

        self.pub_tof = self.create_publisher(
            Float32,
            '/tof_distance',
            qos_profile,
        )

        self.pub_rail_speed = self.create_publisher(
            Float32MultiArray,
            '/rail_speed',
            qos_profile,
        )

        self.pub_rail_command_state = self.create_publisher(
            String,
            '/rail_command_state',
            qos_profile,
        )

        self.pub_rail_odom = self.create_publisher(
            Odometry,
            '/rail/odom',
            qos_profile,
        )

        self.pub_rail_obstacle = self.create_publisher(
            Bool,
            '/rail_obstacle',
            10,
        )
        self.pub_rail_obstacle_front = self.create_publisher(
            Bool,
            '/rail_obstacle_front',
            10,
        )
        self.pub_rail_obstacle_rear = self.create_publisher(
            Bool,
            '/rail_obstacle_rear',
            10,
        )
        self.pub_rail_end_reached = self.create_publisher(
            Bool,
            '/rail_end_reached',
            10,
        )

        # ------------------------------------------------------------
        # Battery / charger publishers
        # ------------------------------------------------------------
        self.pub_battery_state = self.create_publisher(
            BatteryState,
            '/battery/state',
            qos_profile,
        )
        self.pub_battery_soh = self.create_publisher(
            Float32,
            '/battery/soh',
            qos_profile,
        )
        self.pub_battery_remaining_capacity = self.create_publisher(
            Float32,
            '/battery/remaining_capacity',
            qos_profile,
        )
        self.pub_battery_available_energy = self.create_publisher(
            Float32,
            '/battery/available_energy',
            qos_profile,
        )
        self.pub_battery_cycle_count = self.create_publisher(
            UInt32,
            '/battery/cycle_count',
            qos_profile,
        )
        self.pub_battery_error_code = self.create_publisher(
            String,
            '/battery/error_code',
            qos_profile,
        )
        self.pub_battery_status = self.create_publisher(
            String,
            '/battery/status',
            qos_profile,
        )

        self.pub_charger_state = self.create_publisher(
            String,
            '/charger/state',
            qos_profile,
        )
        self.pub_charger_voltage = self.create_publisher(
            Float32,
            '/charger/voltage',
            qos_profile,
        )
        self.pub_charger_current = self.create_publisher(
            Float32,
            '/charger/current',
            qos_profile,
        )
        self.pub_charger_set_voltage = self.create_publisher(
            Float32,
            '/charger/set_voltage',
            qos_profile,
        )
        self.pub_charger_set_current = self.create_publisher(
            Float32,
            '/charger/set_current',
            qos_profile,
        )
        self.pub_charger_ready = self.create_publisher(
            Bool,
            '/charger/ready',
            qos_profile,
        )
        self.pub_charger_contact = self.create_publisher(
            Bool,
            '/charger/contact',
            qos_profile,
        )
        self.pub_charger_charging = self.create_publisher(
            Bool,
            '/charger/charging',
            qos_profile,
        )
        self.pub_charger_completed = self.create_publisher(
            Bool,
            '/charger/completed',
            qos_profile,
        )
        self.pub_charger_error = self.create_publisher(
            Bool,
            '/charger/error',
            qos_profile,
        )
        self.pub_charger_error_code = self.create_publisher(
            String,
            '/charger/error_code',
            qos_profile,
        )
        self.pub_charger_tx_coil_temp = self.create_publisher(
            Float32,
            '/charger/tx_coil_temp',
            qos_profile,
        )
        self.pub_charger_rx_coil_temp = self.create_publisher(
            Float32,
            '/charger/rx_coil_temp',
            qos_profile,
        )
        self.pub_charger_ack = self.create_publisher(
            String,
            '/charger/ack',
            qos_profile,
        )

        # ------------------------------------------------------------
        # Timers
        # ------------------------------------------------------------
        # STM sensor receive loop: 20 Hz
        self.create_timer(0.05, self.update_stm_sensors)

        # Rail command heartbeat.  The ADS driver watchdog depends on this.
        self.create_timer(
            1.0 / self.rail_command_rate_hz,
            self.publish_rail_command_heartbeat,
        )

        # Initial state
        self.publish_rail_end_reached(False)
        self._set_effective_rail_command('STOP', log=False)
        self.publish_rail_command_heartbeat()

        self.get_logger().info(
            'BringUp initialized: '
            'STM32=[IMU, TOF, BMS, WPT], '
            '/charger/command->[STM UART]->WPT CAN2.0A, '
            '/cmd_vel->agv_ads_driver, '
            '/rail_command->/rail/cmd_speed->agv_ads_driver'
        )

    # ============================================================
    # WPT charger command: ROS -> STM32 USART3 -> CAN2.0A
    # ============================================================
    def cb_charger_command(self, msg):
        command = msg.data.strip().upper()

        aliases = {
            'ON': 'ON',
            'START': 'ON',
            'CHARGE_ON': 'ON',
            'OFF': 'OFF',
            'STOP': 'OFF',
            'CHARGE_OFF': 'OFF',
            'RESET': 'RESET',
            'ERROR_RESET': 'RESET',
        }

        if command not in aliases:
            self.get_logger().warning(
                f'Unsupported charger command: {command} '
                f'(use ON/OFF/RESET)'
            )
            return

        command = aliases[command]
        serial_command = f'WPT_CMD,{command}\n'

        try:
            self.stm_serial.write(serial_command.encode('ascii'))
            self.stm_serial.flush()
            self.get_logger().info(
                f'[CHARGER CMD] {command} -> STM32'
            )
        except Exception as error:
            self.get_logger().error(
                f'Failed to send charger command to STM32: {error}'
            )

    # ============================================================
    # Rail command handling
    # ============================================================
    def cb_rail_cmd_msg(self, msg):
        command = msg.data.strip().upper()
        self.variable_rail_speed_mode = False
        self.variable_rail_speed_setpoint = 0.0

        if command not in ('FORWARD', 'BACK', 'STOP'):
            self.get_logger().warning(
                f'Unsupported rail command: {command}'
            )
            return

        # STOP always has the highest priority and also cancels any
        # pending direction reversal.
        if command == 'STOP':
            self.pending_reverse_command = None
            self.requested_rail_command = 'STOP'
            self._set_effective_rail_command(
                'STOP',
                reason='STOP command',
            )
            return

        self.requested_rail_command = command

        # Determine the direction in which the rail is physically moving.
        measured_direction = 'STOP'
        if (
            self.last_measured_rail_velocity >
            self.rail_reverse_stop_threshold_mps
        ):
            measured_direction = 'FORWARD'
        elif (
            self.last_measured_rail_velocity <
            -self.rail_reverse_stop_threshold_mps
        ):
            measured_direction = 'BACK'

        # If feedback is essentially zero, also consider the currently
        # commanded direction. This catches an immediate FORWARD->BACK
        # command before the first feedback sample arrives.
        current_direction = measured_direction
        if (
            current_direction == 'STOP' and
            self.effective_rail_command in ('FORWARD', 'BACK')
        ):
            current_direction = self.effective_rail_command

        # Vendor requirement: direction change must be
        #   FORWARD -> STOP -> BACK
        #   BACK    -> STOP -> FORWARD
        #
        # Do not start the opposite direction until feedback confirms
        # the motor velocity is approximately zero.
        if (
            current_direction in ('FORWARD', 'BACK') and
            current_direction != command
        ):
            self.pending_reverse_command = command
            self.get_logger().info(
                f'[RAIL REVERSE] {current_direction} -> {command}: '
                f'STOP first, waiting for speed <= '
                f'{self.rail_reverse_stop_threshold_mps:.3f} m/s'
            )
            self._set_effective_rail_command(
                'STOP',
                reason=f'reverse interlock before {command}',
            )
            return

        self.pending_reverse_command = None
        self.last_rail_motion_direction = command
        self._reevaluate_rail_command(reason='new command')

    def cb_rail_speed_setpoint(self, msg):
        speed = float(msg.data)

        if not math.isfinite(speed):
            self.get_logger().warning(
                f'Invalid rail speed setpoint: {speed}'
            )
            return

        speed = max(
            -self.rail_back_speed if self.rail_back_speed > 0.0 else -0.5,
            min(
                self.rail_forward_speed if self.rail_forward_speed > 0.0 else 0.5,
                speed,
            ),
        )

        # Allow teleop step control up to +/-0.5 m/s independently of
        # the fixed FORWARD/BACK parameter values.
        speed = max(-0.5, min(0.5, float(msg.data)))

        self.variable_rail_speed_mode = True
        self.variable_rail_speed_setpoint = speed

        if abs(speed) < 1.0e-6:
            self.pending_reverse_command = None
            self.requested_rail_command = 'STOP'
            self._set_effective_rail_speed(
                0.0,
                reason='rail speed setpoint STOP',
            )
            return

        command = 'FORWARD' if speed > 0.0 else 'BACK'
        self.requested_rail_command = command

        # Respect the same direction-reversal safety rule.
        current_direction = 'STOP'
        if self.last_measured_rail_velocity > self.rail_reverse_stop_threshold_mps:
            current_direction = 'FORWARD'
        elif self.last_measured_rail_velocity < -self.rail_reverse_stop_threshold_mps:
            current_direction = 'BACK'
        elif self.effective_rail_command in ('FORWARD', 'BACK'):
            current_direction = self.effective_rail_command

        if current_direction in ('FORWARD', 'BACK') and current_direction != command:
            self.pending_reverse_command = command
            self._set_effective_rail_speed(
                0.0,
                reason=f'reverse interlock before {command}',
            )
            return

        self.pending_reverse_command = None
        self.last_rail_motion_direction = command

        if self.rail_state not in self.rail_motor_allowed_states:
            self._set_effective_rail_speed(
                0.0,
                reason=f'{command} blocked in rail_state={self.rail_state}',
            )
            return

        if command == 'FORWARD' and self.rail_end_reached:
            self._set_effective_rail_speed(
                0.0,
                reason='FORWARD blocked by rail end',
            )
            return

        if self._obstacle_for_direction(command):
            self.obstacle_stop_latched = True
            self._set_effective_rail_speed(
                0.0,
                reason=f'{command} blocked by obstacle',
            )
            return

        self.obstacle_stop_latched = False
        self._set_effective_rail_speed(
            speed,
            reason='rail speed setpoint',
        )

    def _set_effective_rail_speed(self, speed, reason=''):
        speed = float(speed)

        if speed > 0.0:
            command = 'FORWARD'
        elif speed < 0.0:
            command = 'BACK'
        else:
            command = 'STOP'
            speed = 0.0

        changed = (
            command != self.effective_rail_command or
            abs(speed - self.effective_rail_speed) > 1.0e-9
        )

        self.effective_rail_command = command
        self.effective_rail_speed = speed

        if changed:
            self.publish_rail_command_state(command)
            self._publish_rail_speed_command(speed)
            suffix = f' ({reason})' if reason else ''
            self.get_logger().info(
                f'[RAIL SPEED] {command} -> {speed:+.3f} m/s{suffix}'
            )

    def cb_rail_state_msg(self, msg):
        rail_state = msg.data.strip().upper()

        valid_states = {
            'MOVING_TO_RAIL',
            'RAIL_APPROACH',
            'ENTERING',
            'ON_RAIL',
            'WORKING',
            'EXITING',
            'OUT_RAIL',
        }

        if rail_state not in valid_states:
            self.get_logger().warning(
                f'Unknown rail state: {rail_state}'
            )
            return

        if rail_state == self.rail_state:
            return

        previous_state = self.rail_state
        self.rail_state = rail_state

        self.get_logger().info(
            f'Rail state: {previous_state} -> {rail_state}'
        )

        if rail_state == 'OUT_RAIL':
            # Immediate rail motor stop + rail odometry reset.
            self.variable_rail_speed_mode = False
            self.variable_rail_speed_setpoint = 0.0
            self.pending_reverse_command = None
            self.requested_rail_command = 'STOP'
            self._set_effective_rail_command(
                'STOP',
                reason='OUT_RAIL',
            )

            if previous_state != 'OUT_RAIL':
                self.reset_rail_odometry(publish=True)
        else:
            self._reevaluate_rail_command(reason='rail state changed')

    def _reevaluate_rail_command(self, reason=''):
        command = self.requested_rail_command

        # While a direction reversal is pending, stay at STOP until
        # /rail/speed_feedback confirms that the motor is stationary.
        if (
            self.pending_reverse_command is not None and
            command == self.pending_reverse_command
        ):
            self._set_effective_rail_command(
                'STOP',
                reason='waiting for rail motor to stop before reverse',
            )
            return

        if command == 'STOP':
            self._set_effective_rail_command(
                'STOP',
                reason=reason,
            )
            return

        # Rail motor must never run outside the allowed rail states.
        if self.rail_state not in self.rail_motor_allowed_states:
            self._set_effective_rail_command(
                'STOP',
                reason=(
                    f'{command} blocked in rail_state={self.rail_state}'
                ),
            )
            return

        # Rail end blocks only FORWARD. BACK remains available to retreat.
        if command == 'FORWARD' and self.rail_end_reached:
            self._set_effective_rail_command(
                'STOP',
                reason='FORWARD blocked by rail end',
            )
            return

        # Directional obstacle check.
        if self._obstacle_for_direction(command):
            self.obstacle_stop_latched = True
            self._set_effective_rail_command(
                'STOP',
                reason=f'{command} blocked by obstacle',
            )
            return

        # Safe to execute requested motion.
        self.obstacle_stop_latched = False
        self._set_effective_rail_command(
            command,
            reason=reason,
        )

    def _set_effective_rail_command(
        self,
        command,
        reason='',
        log=True,
    ):
        command = command.strip().upper()

        if command == 'FORWARD':
            speed = self.rail_forward_speed
        elif command == 'BACK':
            speed = -self.rail_back_speed
        else:
            command = 'STOP'
            speed = 0.0

        changed = (
            command != self.effective_rail_command or
            abs(speed - self.effective_rail_speed) > 1.0e-9
        )

        self.effective_rail_command = command
        self.effective_rail_speed = speed

        if changed:
            self.publish_rail_command_state(command)
            self._publish_rail_speed_command(speed)

            if log:
                suffix = f' ({reason})' if reason else ''
                self.get_logger().info(
                    f'[RAIL CMD] {command} -> '
                    f'{speed:+.3f} m/s{suffix}'
                )

    def publish_rail_command_heartbeat(self):
        """
        Publish the current effective speed periodically.

        This is intentionally a heartbeat: agv_ads_driver updates its watchdog
        time on every message but sends a Beckhoff MoveVelocity command only
        when the requested speed actually changes.
        """
        self._publish_rail_speed_command(
            self.effective_rail_speed
        )

    def _publish_rail_speed_command(self, speed):
        msg = Float32()
        msg.data = float(speed)
        self.pub_rail_cmd_speed.publish(msg)

    def publish_rail_command_state(self, command):
        msg = String()
        msg.data = str(command).strip().upper()
        self.pub_rail_command_state.publish(msg)

    # ============================================================
    # Obstacle handling
    # ============================================================
    def _obstacle_for_direction(self, direction):
        if direction == 'FORWARD':
            return self.front_rail_obstacle
        if direction == 'BACK':
            return self.rear_rail_obstacle
        return False

    def _publish_directional_obstacles(self):
        front = Bool()
        front.data = bool(self.front_rail_obstacle)
        self.pub_rail_obstacle_front.publish(front)

        rear = Bool()
        rear.data = bool(self.rear_rail_obstacle)
        self.pub_rail_obstacle_rear.publish(rear)

    def cb_scan(self, scan_msg):
        # Obstacle detection is relevant only while rail motor can be active.
        if self.rail_state not in self.rail_motor_allowed_states:
            changed = (
                self.front_rail_obstacle or
                self.rear_rail_obstacle or
                self.rail_obstacle
            )

            self.front_rail_obstacle = False
            self.rear_rail_obstacle = False
            self.rail_obstacle = False
            self.obstacle_stop_latched = False

            self._publish_directional_obstacles()
            self._publish_rail_obstacle(False)

            if changed:
                self._reevaluate_rail_command(
                    reason='obstacle monitoring disabled'
                )
            return

        half_fov_rad = math.radians(
            self.rail_obstacle_fov_deg
        )

        front_detected = False
        rear_detected = False
        front_min = float('inf')
        rear_min = float('inf')

        for index, distance in enumerate(scan_msg.ranges):
            if not math.isfinite(distance):
                continue
            if (
                distance < scan_msg.range_min or
                distance > scan_msg.range_max or
                distance > self.rail_obstacle_distance
            ):
                continue

            angle = (
                scan_msg.angle_min +
                index * scan_msg.angle_increment
            )
            angle = math.atan2(
                math.sin(angle),
                math.cos(angle),
            )

            if abs(angle) <= half_fov_rad:
                front_detected = True
                front_min = min(front_min, distance)

            rear_error = abs(abs(angle) - math.pi)
            if rear_error <= half_fov_rad:
                rear_detected = True
                rear_min = min(rear_min, distance)

        old_front = self.front_rail_obstacle
        old_rear = self.rear_rail_obstacle

        self.front_rail_obstacle = front_detected
        self.rear_rail_obstacle = rear_detected
        self._publish_directional_obstacles()

        if self.requested_rail_command == 'FORWARD':
            detected = front_detected
            min_distance = front_min
        elif self.requested_rail_command == 'BACK':
            detected = rear_detected
            min_distance = rear_min
        elif self.last_rail_motion_direction == 'FORWARD':
            detected = front_detected
            min_distance = front_min
        elif self.last_rail_motion_direction == 'BACK':
            detected = rear_detected
            min_distance = rear_min
        else:
            detected = False
            min_distance = float('inf')

        previous_combined = self.rail_obstacle
        self.rail_obstacle = bool(detected)
        self._publish_rail_obstacle(self.rail_obstacle)

        directional_changed = (
            old_front != self.front_rail_obstacle or
            old_rear != self.rear_rail_obstacle
        )

        if self.rail_obstacle and not previous_combined:
            self.get_logger().warning(
                f'Rail obstacle detected at '
                f'{min_distance:.2f} m. Rail STOP.'
            )

        if not self.rail_obstacle and previous_combined:
            self.get_logger().info(
                'Rail obstacle cleared. '
                'Requested rail command will resume automatically.'
            )

        if directional_changed or (
            previous_combined != self.rail_obstacle
        ):
            self._reevaluate_rail_command(
                reason='obstacle state changed'
            )

    def _publish_rail_obstacle(self, detected):
        msg = Bool()
        msg.data = bool(detected)
        self.pub_rail_obstacle.publish(msg)

    # ============================================================
    # ADS rail feedback -> rail speed / odometry
    # ============================================================
    def cb_rail_speed_feedback(self, msg):
        if not msg.data:
            return

        values = [
            float(value)
            for value in msg.data
            if math.isfinite(float(value))
        ]

        if not values:
            return

        # Current Axis-3 assumption yields one feedback value.
        # Averaging also keeps compatibility if the driver later publishes
        # multiple rail motor values.
        measured_velocity = sum(values) / len(values)

        if abs(measured_velocity) < 0.001:
            measured_velocity = 0.0

        self.last_measured_rail_velocity = measured_velocity

        # Complete a pending direction reversal only after the actual
        # ADS feedback confirms that the rail motor has stopped.
        if (
            self.pending_reverse_command is not None and
            abs(measured_velocity) <=
            self.rail_reverse_stop_threshold_mps
        ):
            reverse_command = self.pending_reverse_command
            self.pending_reverse_command = None
            self.last_rail_motion_direction = reverse_command

            self.get_logger().info(
                f'[RAIL REVERSE] STOP confirmed '
                f'({measured_velocity:+.3f} m/s) -> '
                f'{reverse_command}'
            )

            self._reevaluate_rail_command(
                reason='STOP confirmed before direction reversal'
            )

        speed_msg = Float32MultiArray()
        speed_msg.data = values
        self.pub_rail_speed.publish(speed_msg)

        timestamp_now = self.get_clock().now()

        rail_odom_active = (
            self.rail_state in self.rail_motor_allowed_states
        )

        if self.rail_odom_last_time is not None:
            dt = (
                timestamp_now -
                self.rail_odom_last_time
            ).nanoseconds * 1e-9

            if rail_odom_active and dt > 0.0:
                self.rail_odom_distance += (
                    measured_velocity * dt
                )

        self.rail_odom_last_time = timestamp_now

        if (
            rail_odom_active and
            self.rail_odom_distance >=
            self.rail_max_distance_m
        ):
            if not self.rail_end_reached:
                self.set_rail_end_reached(True)

        elif (
            self.rail_end_reached and
            self.rail_odom_distance <=
            (
                self.rail_max_distance_m -
                self.rail_end_release_margin_m
            )
        ):
            self.set_rail_end_reached(False)

        # A rail-end / obstacle update may block or resume motion.
        # In variable-speed mode, preserve the exact signed speed requested
        # by t/b/g instead of remapping BACK/FORWARD to the fixed 0.2 m/s.
        if self.variable_rail_speed_mode:
            variable_speed = self.variable_rail_speed_setpoint

            if abs(variable_speed) < 1.0e-6:
                self._set_effective_rail_speed(
                    0.0,
                    reason='rail feedback update / variable STOP',
                )
            else:
                variable_direction = (
                    'FORWARD' if variable_speed > 0.0 else 'BACK'
                )

                blocked = False

                if self.rail_state not in self.rail_motor_allowed_states:
                    blocked = True
                elif variable_direction == 'FORWARD' and self.rail_end_reached:
                    blocked = True
                elif self._obstacle_for_direction(variable_direction):
                    blocked = True

                if blocked:
                    self._set_effective_rail_speed(
                        0.0,
                        reason='rail feedback update / variable speed blocked',
                    )
                else:
                    self._set_effective_rail_speed(
                        variable_speed,
                        reason='rail feedback update / keep variable speed',
                    )
        else:
            self._reevaluate_rail_command(
                reason='rail feedback update'
            )

        odom_velocity = (
            measured_velocity
            if rail_odom_active
            else 0.0
        )

        self.publish_rail_odometry(
            timestamp_now,
            self.rail_odom_distance,
            odom_velocity,
        )

    def publish_rail_odometry(
        self,
        timestamp_now,
        distance,
        linear_velocity,
    ):
        msg = Odometry()
        msg.header.stamp = timestamp_now.to_msg()
        msg.header.frame_id = 'rail_odom'
        msg.child_frame_id = 'base_footprint'

        msg.pose.pose.position.x = float(distance)
        msg.pose.pose.orientation.w = 1.0
        msg.twist.twist.linear.x = float(linear_velocity)

        pose_covariance = [0.0] * 36
        pose_covariance[0] = 0.02
        pose_covariance[7] = 1.0e6
        pose_covariance[14] = 1.0e6
        pose_covariance[21] = 1.0e6
        pose_covariance[28] = 1.0e6
        pose_covariance[35] = 1.0e6
        msg.pose.covariance = pose_covariance

        twist_covariance = [0.0] * 36
        twist_covariance[0] = 0.02
        twist_covariance[7] = 1.0e6
        twist_covariance[14] = 1.0e6
        twist_covariance[21] = 1.0e6
        twist_covariance[28] = 1.0e6
        twist_covariance[35] = 1.0e6
        msg.twist.covariance = twist_covariance

        self.pub_rail_odom.publish(msg)

    def reset_rail_odometry(self, publish=False):
        self.rail_odom_distance = 0.0
        self.rail_odom_last_time = self.get_clock().now()
        self.set_rail_end_reached(False)

        if publish:
            self.publish_rail_odometry(
                self.rail_odom_last_time,
                0.0,
                0.0,
            )

        self.get_logger().info(
            'Rail odometry reset: 0.000 m'
        )

    def set_rail_end_reached(self, reached):
        reached = bool(reached)

        if reached == self.rail_end_reached:
            return

        self.rail_end_reached = reached
        self.publish_rail_end_reached(reached)

        if reached:
            self.get_logger().warning(
                f'[RAIL LIMIT] End reached at '
                f'{self.rail_odom_distance:.3f} m '
                f'(limit={self.rail_max_distance_m:.3f} m).'
            )
        else:
            self.get_logger().info(
                '[RAIL LIMIT] End cleared.'
            )

    def publish_rail_end_reached(self, reached):
        msg = Bool()
        msg.data = bool(reached)
        self.pub_rail_end_reached.publish(msg)

    # ============================================================
    # STM32 serial: IMU / TOF / Battery BMS / WPT charger
    # ============================================================
    def update_stm_sensors(self):
        if self.stm_serial.in_waiting <= 0:
            return

        try:
            latest_imu_line = None
            latest_tof_line = None
            latest_bms_line = None
            latest_wpt_line = None
            latest_wpt_cmd_line = None

            lines_read = 0

            while (
                self.stm_serial.in_waiting > 0 and
                lines_read < self.max_serial_lines_per_update
            ):
                line = (
                    self.stm_serial.readline()
                    .decode('utf-8', errors='ignore')
                    .strip()
                )
                lines_read += 1

                if not line:
                    continue

                if line.startswith('IMU'):
                    latest_imu_line = line
                elif line.startswith('TOF'):
                    latest_tof_line = line
                elif line.startswith('BMS'):
                    latest_bms_line = line
                elif line.startswith('WPTCMD'):
                    latest_wpt_cmd_line = line
                elif line.startswith('WPT'):
                    latest_wpt_line = line
                elif (
                    line.startswith('WHEEL') or
                    line.startswith('RAIL') or
                    line.startswith('ADS')
                ):
                    # Intentionally ignored after Beckhoff migration.
                    pass
                else:
                    self.get_logger().debug(
                        f'[STM UNKNOWN] {line}'
                    )

            timestamp_now = self.get_clock().now()

            if latest_imu_line is not None:
                self.publish_imu_from_line(
                    timestamp_now,
                    latest_imu_line,
                )

            if latest_tof_line is not None:
                self.publish_tof_from_line(
                    latest_tof_line
                )

            if latest_bms_line is not None:
                self.publish_battery_from_line(
                    timestamp_now,
                    latest_bms_line,
                )

            if latest_wpt_line is not None:
                self.publish_charger_from_line(
                    latest_wpt_line
                )

            if latest_wpt_cmd_line is not None:
                msg = String()
                msg.data = latest_wpt_cmd_line
                self.pub_charger_ack.publish(msg)

        except Exception as error:
            self.get_logger().warning(
                f'STM receive error: {error}'
            )

    @staticmethod
    def _parse_key_value_line(line):
        values = {}
        for token in line.split(',')[1:]:
            if '=' not in token:
                continue
            key, value = token.split('=', 1)
            values[key.strip().upper()] = value.strip()
        return values

    def publish_battery_from_line(self, timestamp_now, bms_line):
        status_msg = String()
        status_msg.data = bms_line
        self.pub_battery_status.publish(status_msg)

        if bms_line == 'BMS,TIMEOUT':
            return

        values = self._parse_key_value_line(bms_line)

        try:
            voltage = float(values['V'])
            current = float(values['I'])
            soc_percent = float(values['SOC'])
            soh_percent = float(values['SOH'])
            temperature = float(values['T'])
            remaining_capacity = float(values['RC'])
            available_energy = float(values['AE'])
            cycle_count = int(values['CYC'], 0)
            status_flags = int(values['STAT'], 0)
        except (KeyError, ValueError) as error:
            self.get_logger().warning(
                f'[STM BMS] Invalid line: {bms_line} ({error})'
            )
            return

        battery = BatteryState()
        battery.header.stamp = timestamp_now.to_msg()
        battery.header.frame_id = 'battery'

        battery.voltage = voltage
        battery.current = current
        battery.charge = remaining_capacity
        battery.capacity = float('nan')
        battery.design_capacity = float('nan')
        battery.percentage = max(0.0, min(1.0, soc_percent / 100.0))
        battery.temperature = temperature
        battery.present = True

        if current > 0.10:
            battery.power_supply_status = (
                BatteryState.POWER_SUPPLY_STATUS_CHARGING
            )
        elif current < -0.10:
            battery.power_supply_status = (
                BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
            )
        else:
            battery.power_supply_status = (
                BatteryState.POWER_SUPPLY_STATUS_NOT_CHARGING
            )

        battery.power_supply_health = (
            BatteryState.POWER_SUPPLY_HEALTH_GOOD
            if status_flags == 0
            else BatteryState.POWER_SUPPLY_HEALTH_UNKNOWN
        )
        battery.power_supply_technology = (
            BatteryState.POWER_SUPPLY_TECHNOLOGY_UNKNOWN
        )

        self.pub_battery_state.publish(battery)

        msg = Float32()
        msg.data = soh_percent
        self.pub_battery_soh.publish(msg)

        msg = Float32()
        msg.data = remaining_capacity
        self.pub_battery_remaining_capacity.publish(msg)

        msg = Float32()
        msg.data = available_energy
        self.pub_battery_available_energy.publish(msg)

        cycle_msg = UInt32()
        cycle_msg.data = max(0, cycle_count)
        self.pub_battery_cycle_count.publish(cycle_msg)

        error_code_msg = String()
        error_code_msg.data = f'0x{status_flags:04X}'
        self.pub_battery_error_code.publish(error_code_msg)

    def publish_charger_from_line(self, wpt_line):
        if wpt_line == 'WPT,TIMEOUT':
            state_msg = String()
            state_msg.data = 'TIMEOUT'
            self.pub_charger_state.publish(state_msg)

            bool_msg = Bool()
            bool_msg.data = False
            self.pub_charger_ready.publish(bool_msg)
            self.pub_charger_charging.publish(bool_msg)
            self.pub_charger_completed.publish(bool_msg)

            error_msg = Bool()
            error_msg.data = True
            self.pub_charger_error.publish(error_msg)

            code_msg = String()
            code_msg.data = 'TIMEOUT'
            self.pub_charger_error_code.publish(code_msg)
            return

        values = self._parse_key_value_line(wpt_line)

        try:
            voltage = float(values['V'])
            current = float(values['I'])
            set_voltage = float(values['SETV'])
            set_current = float(values['SETI'])
            status_mode = int(values['STATUS'], 0)
            contact = int(values['CONTACT'], 0)
            ready = int(values['READY'], 0)
            error_flags = int(values['ERR'], 0)
            tx_temp = float(values['TXT'])
            rx_temp = float(values['RXT'])

            ack_command_str, ack_result_str = values.get(
                'ACK', '0x00/0x00'
            ).split('/', 1)
            ack_command = int(ack_command_str, 0)
            ack_result = int(ack_result_str, 0)
        except (KeyError, ValueError) as error:
            self.get_logger().warning(
                f'[STM WPT] Invalid line: {wpt_line} ({error})'
            )
            return

        state_names = {
            0: 'STOP',
            1: 'READY',
            4: 'CHARGING',
            5: 'COMPLETED',
            7: 'WARNING',
            8: 'ERROR',
        }
        state = state_names.get(
            status_mode,
            f'UNKNOWN_{status_mode}'
        )

        state_msg = String()
        state_msg.data = state
        self.pub_charger_state.publish(state_msg)

        for publisher, value in (
            (self.pub_charger_voltage, voltage),
            (self.pub_charger_current, current),
            (self.pub_charger_set_voltage, set_voltage),
            (self.pub_charger_set_current, set_current),
            (self.pub_charger_tx_coil_temp, tx_temp),
            (self.pub_charger_rx_coil_temp, rx_temp),
        ):
            msg = Float32()
            msg.data = float(value)
            publisher.publish(msg)

        msg = Bool()
        msg.data = bool(ready)
        self.pub_charger_ready.publish(msg)

        msg = Bool()
        msg.data = bool(contact)
        self.pub_charger_contact.publish(msg)

        msg = Bool()
        msg.data = (status_mode == 4)
        self.pub_charger_charging.publish(msg)

        msg = Bool()
        msg.data = (status_mode == 5)
        self.pub_charger_completed.publish(msg)

        msg = Bool()
        msg.data = (error_flags != 0 or status_mode == 8)
        self.pub_charger_error.publish(msg)

        code_msg = String()
        code_msg.data = f'0x{error_flags:04X}'
        self.pub_charger_error_code.publish(code_msg)

        ack_name = {
            0xA1: 'ON',
            0xA7: 'OFF',
            0xC1: 'RESET',
        }.get(ack_command, f'0x{ack_command:02X}')

        ack_result_name = {
            0x06: 'OK',
            0x21: 'FAIL',
            0x00: 'NONE',
        }.get(ack_result, f'0x{ack_result:02X}')

        ack_msg = String()
        ack_msg.data = f'{ack_name}:{ack_result_name}'
        self.pub_charger_ack.publish(ack_msg)

    def publish_tof_from_line(self, tof_line):
        try:
            # STM32 format: "TOF: 176.00 mm"
            distance_mm = float(
                tof_line
                .replace('TOF:', '')
                .replace('mm', '')
                .strip()
            )
        except ValueError:
            self.get_logger().warning(
                f'[STM TOF] Invalid line: {tof_line}'
            )
            return

        if not math.isfinite(distance_mm):
            return

        msg = Float32()
        msg.data = distance_mm
        self.pub_tof.publish(msg)

    def publish_imu_from_line(
        self,
        timestamp_now,
        imu_line,
    ):
        parts = imu_line.split(',')

        if len(parts) < 4:
            return

        try:
            roll = float(parts[1])
            pitch = float(parts[2])
            yaw = float(parts[3])
        except ValueError:
            return

        if not all(
            math.isfinite(value)
            for value in (roll, pitch, yaw)
        ):
            return

        quaternion = quaternion_from_euler(
            roll,
            pitch,
            yaw,
        )

        message = Imu()
        message.header.stamp = timestamp_now.to_msg()
        message.header.frame_id = 'imu_link'

        message.orientation.x = quaternion[0]
        message.orientation.y = quaternion[1]
        message.orientation.z = quaternion[2]
        message.orientation.w = quaternion[3]

        message.orientation_covariance = [
            0.10, 0.0, 0.0,
            0.0, 0.10, 0.0,
            0.0, 0.0, 0.05,
        ]

        message.angular_velocity_covariance[0] = -1.0
        message.linear_acceleration_covariance[0] = -1.0

        self.pub_imu.publish(message)

    # ============================================================
    # Shutdown
    # ============================================================
    def shutdown_safely(self):
        # Stop the rail through ROS -> agv_ads_driver.
        self.pending_reverse_command = None
        self.requested_rail_command = 'STOP'
        self._set_effective_rail_command(
            'STOP',
            reason='bringup shutdown',
        )

        # Send a few STOP heartbeats before this node disappears.
        for _ in range(3):
            self.publish_rail_command_heartbeat()

        try:
            if self.stm_serial.is_open:
                self.stm_serial.close()
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = BringUp()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown_safely()
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()