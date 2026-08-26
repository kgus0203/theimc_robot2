import math
import serial
import sys

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Imu, JointState, LaserScan
from std_msgs.msg import Bool, Float32, Float32MultiArray, String


def quaternion_from_euler(roll, pitch, yaw):
    """Convert roll/pitch/yaw in radians to quaternion [x, y, z, w]."""
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


class OdomPose:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0


class Joint:
    def __init__(self):
        self.joint_name = [
            'wheel_left_joint',
            'wheel_right_joint',
        ]
        self.joint_pos = [0.0, 0.0]
        self.joint_vel = [0.0, 0.0]


class BringUp(Node):
    def __init__(self):
        super().__init__('bring_up')

        # STM32 serial link
        self.serial_port = '/dev/stm32_link'
        self.baud_rate = 230400
        self.serial_timeout = 0.1

        try:
            self.stm_serial = serial.Serial(
                port=self.serial_port,
                baudrate=self.baud_rate,
                timeout=self.serial_timeout,
            )
            self.stm_serial.reset_input_buffer()
            self.stm_serial.reset_output_buffer()
            self.get_logger().info(f'Connected to {self.serial_port} @ {self.baud_rate}')
        except serial.SerialException as error:
            self.get_logger().error(f'Failed to open serial: {error}')
            sys.exit(1)

        # Robot parameters
        self.wheel_separation = 1.0
        self.wheel_radius = 0.085
        self.max_lin_vel_x = 1.0
        self.max_ang_vel_z = 1.0
        self.max_rail_vel = 0.5

        self.odom_pose = OdomPose()
        self.joint = Joint()
        self.timestamp_previous = self.get_clock().now()

        # Rail odometry
        # Signed 1-D distance along the rail.
        # + : forward, - : backward
        self.rail_odom_distance = 0.0
        self.rail_odom_last_time = None

        # Rail state is now decided by ROS2 BT, not STM32.
        self.rail_state = 'OUT_RAIL'
        self.is_on_rail = False
        self.requested_rail_command = 'STOP'

        # Obstacle state
        self.rail_obstacle = False
        self.front_rail_obstacle = False
        self.rear_rail_obstacle = False
        self.obstacle_stop_latched = False
        self.last_rail_motion_direction = 'STOP'
        self.rail_obstacle_distance = 0.5

        # Interpreted as +/- degrees from the front/rear center line.
        # 45.0 means front sector -45..+45 deg and rear sector 135..225 deg.
        self.rail_obstacle_fov_deg = 45.0

        # Prevent one 20 Hz timer callback from staying forever inside the
        # serial-drain loop when STM32 is continuously streaming data.
        self.max_serial_lines_per_update = 64

        qos_profile = QoSProfile(
            depth=20,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )

        # Command subscriptions
        self.sub_cmd_vel = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cb_cmd_vel_msg,
            qos_profile,
        )
        self.sub_cmd_rail = self.create_subscription(
            Twist,
            'cmd_rail',
            self.cb_cmd_rail_msg,
            qos_profile,
        )
        self.sub_rail_cmd = self.create_subscription(
            String,
            'rail_command',
            self.cb_rail_cmd_msg,
            qos_profile,
        )

        # /rail_state is published by the BT layer.
        self.sub_rail_state = self.create_subscription(
            String,
            '/rail_state',
            self.cb_rail_state_msg,
            qos_profile,
        )

        self.sub_scan = self.create_subscription(
            LaserScan,
            'scan',
            self.cb_scan,
            qos_profile_sensor_data,
        )

        # Sensor publishers
        self.pub_joint_states = self.create_publisher(
            JointState,
            'joint_states',
            qos_profile,
        )
        self.pub_odom = self.create_publisher(
            Odometry,
            '/wheel/odom',
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

        # 20 Hz serial receive/update loop
        self.create_timer(0.05, self.update_robot)

    # ------------------------------------------------------------------
    # ROS2 -> STM32 commands
    # ------------------------------------------------------------------
    def _write_serial_line(self, line):
        try:
            self.stm_serial.write((line + '\n').encode('utf-8'))
            return True
        except serial.SerialException as error:
            self.get_logger().error(f'Serial write failed: {error}')
            return False

    def send_base_command(self, linear_x, angular_z):
        self._write_serial_line(f'CMD,{linear_x:.3f},{angular_z:.3f}')

    def send_rail_velocity(self, rail_velocity):
        # STM32 RAIL command accepts one velocity value.
        self._write_serial_line(f'RAIL,{rail_velocity:.3f}')

    def publish_rail_command_state(self, command):
        msg = String()
        msg.data = str(command).strip().upper()
        self.pub_rail_command_state.publish(msg)

    def cb_cmd_vel_msg(self, msg):
        vx = 0.0 if abs(msg.linear.x) < 0.01 else msg.linear.x
        wz = 0.0 if abs(msg.angular.z) < 0.01 else msg.angular.z

        vx = max(-self.max_lin_vel_x, min(self.max_lin_vel_x, vx))
        wz = max(-self.max_ang_vel_z, min(self.max_ang_vel_z, wz))

        self.send_base_command(vx, wz)

    def cb_cmd_rail_msg(self, msg):
        # One common rail velocity is used by the STM32 rail controller.
        rail_velocity = msg.linear.x
        rail_velocity = 0.0 if abs(rail_velocity) < 0.01 else rail_velocity
        rail_velocity = max(-self.max_rail_vel, min(self.max_rail_vel, rail_velocity))

        if rail_velocity > 0.0:
            requested = 'FORWARD'
        elif rail_velocity < 0.0:
            requested = 'BACK'
        else:
            requested = 'STOP'

        if requested in ('FORWARD', 'BACK'):
            self.last_rail_motion_direction = requested

            if self._obstacle_for_direction(requested):
                self.get_logger().warning(
                    f'Blocked rail velocity command [{requested}] due to obstacle.'
                )
                self.send_rail_velocity(0.0)
                self.publish_rail_command_state('STOP')
                self.requested_rail_command = requested
                self.obstacle_stop_latched = True
                return

        self.requested_rail_command = requested
        self.send_rail_velocity(rail_velocity)
        self.publish_rail_command_state(requested)

    def cb_rail_cmd_msg(self, msg):
        """Compatibility path for existing GUI/BT string commands."""
        command = msg.data.strip().upper()

        if command not in ('FORWARD', 'BACK', 'STOP'):
            self.get_logger().warning(f'Unsupported rail command: {command}')
            return

        if command in ('FORWARD', 'BACK'):
            self.last_rail_motion_direction = command

            if self._obstacle_for_direction(command):
                self.get_logger().warning(
                    f'Cannot execute [{command}] due to detected obstacle.'
                )
                self._write_serial_line('STOP')
                self.publish_rail_command_state('STOP')

                # Keep the requested direction so scan monitoring remains active.
                self.requested_rail_command = command
                self.obstacle_stop_latched = True
                return

        # A user/BT STOP is accepted. If an obstacle is currently latched,
        # last_rail_motion_direction is preserved so the display can remain
        # DETECTED until the physical obstacle is actually gone.
        if self._write_serial_line(command):
            self.requested_rail_command = command
            self.publish_rail_command_state(command)
            self.get_logger().info(f'Sent [{command}] command to STM32.')

    # ------------------------------------------------------------------
    # BT rail state -> BringUp
    # ------------------------------------------------------------------
    def cb_rail_state_msg(self, msg):
        rail_state = msg.data.strip().upper()

        if rail_state not in ('MOVING_TO_RAIL', 'RAIL_APPROACH', 'ENTERING', 'ON_RAIL', 'WORKING', 'EXITING', 'OUT_RAIL'):
            self.get_logger().warning(f'Unknown rail state: {rail_state}')
            return

        if rail_state == self.rail_state:
            return

        previous_state = self.rail_state
        self.rail_state = rail_state

        # Main-wheel odometry should be frozen while the robot is physically
        # engaging with / riding on / leaving the rail.
        self.is_on_rail = rail_state in (
            'ENTERING',
            'ON_RAIL',
            'WORKING',
            'EXITING',
        )

        # /rail/odom is a per-ride coordinate. It is cleared only when the
        # robot has completely left the rail.
        if rail_state == 'OUT_RAIL' and previous_state != 'OUT_RAIL':
            self.reset_rail_odometry(publish=True)

        self.get_logger().info(f'Rail state => [{rail_state}]')

    # ------------------------------------------------------------------
    # Rail obstacle detection
    # ------------------------------------------------------------------
    def _obstacle_for_direction(self, direction):
        if direction == 'FORWARD':
            return self.front_rail_obstacle
        if direction == 'BACK':
            return self.rear_rail_obstacle
        return False

    def _publish_directional_obstacles(self):
        msg_front = Bool()
        msg_front.data = self.front_rail_obstacle
        self.pub_rail_obstacle_front.publish(msg_front)

        msg_rear = Bool()
        msg_rear.data = self.rear_rail_obstacle
        self.pub_rail_obstacle_rear.publish(msg_rear)

    def cb_scan(self, scan_msg):
        # Rail obstacle sensing is meaningful while riding or leaving the rail.
        if self.rail_state not in ('ON_RAIL', 'WORKING', 'EXITING'):
            self.front_rail_obstacle = False
            self.rear_rail_obstacle = False
            self._publish_directional_obstacles()
            self.set_rail_obstacle(False)
            return

        half_fov_rad = math.radians(self.rail_obstacle_fov_deg)

        front_detected = False
        rear_detected = False
        front_min = float('inf')
        rear_min = float('inf')

        for index, distance in enumerate(scan_msg.ranges):
            if not math.isfinite(distance):
                continue
            if distance < scan_msg.range_min or distance > scan_msg.range_max:
                continue
            if distance > self.rail_obstacle_distance:
                continue

            angle = scan_msg.angle_min + index * scan_msg.angle_increment

            # Normalize to [-pi, +pi].
            angle = math.atan2(math.sin(angle), math.cos(angle))

            if abs(angle) <= half_fov_rad:
                front_detected = True
                front_min = min(front_min, distance)

            rear_error = abs(abs(angle) - math.pi)
            if rear_error <= half_fov_rad:
                rear_detected = True
                rear_min = min(rear_min, distance)

        self.front_rail_obstacle = front_detected
        self.rear_rail_obstacle = rear_detected
        self._publish_directional_obstacles()

        # The legacy /rail_obstacle topic remains direction-aware.
        # If a STOP was issued because of an obstacle, continue monitoring the
        # last motion direction until that obstacle physically disappears.
        if self.requested_rail_command in ('FORWARD', 'BACK'):
            monitor_direction = self.requested_rail_command
        elif self.obstacle_stop_latched:
            monitor_direction = self.last_rail_motion_direction
        else:
            monitor_direction = 'STOP'

        if monitor_direction == 'FORWARD':
            detected = front_detected
            min_distance = front_min
        elif monitor_direction == 'BACK':
            detected = rear_detected
            min_distance = rear_min
        else:
            detected = False
            min_distance = float('inf')

        self.set_rail_obstacle(detected, min_distance)

    def set_rail_obstacle(self, detected, distance=float('inf')):
        previous = self.rail_obstacle
        self.rail_obstacle = bool(detected)

        msg = Bool()
        msg.data = self.rail_obstacle
        self.pub_rail_obstacle.publish(msg)

        if self.rail_obstacle:
            if not self.obstacle_stop_latched:
                direction = (
                    self.requested_rail_command
                    if self.requested_rail_command in ('FORWARD', 'BACK')
                    else self.last_rail_motion_direction
                )
                self.get_logger().warning(
                    f'Rail obstacle [{direction}] at {distance:.2f} m. '
                    'Stopping rail motor.'
                )
                self._write_serial_line('STOP')
                self.publish_rail_command_state('STOP')
                self.obstacle_stop_latched = True
        else:
            if previous:
                self.get_logger().info('Rail obstacle cleared.')

            self.obstacle_stop_latched = False

    # ------------------------------------------------------------------
    # STM32 -> ROS2 sensor parsing
    # STM output format:
    #   WHEEL,<left_mps>,<right_mps>
    #   RAIL,<rm1_mps>,<rm2_mps>
    #   IMU,<roll_rad>,<pitch_rad>,<yaw_rad>
    #   TOF,<distance_mm>
    # ------------------------------------------------------------------
    def update_robot(self):
        timestamp_now = self.get_clock().now()
        dt = (timestamp_now - self.timestamp_previous).nanoseconds * 1e-9

        if self.stm_serial.in_waiting <= 0:
            return

        try:
            latest_wheel_line = None
            latest_rail_line = None
            latest_imu_line = None
            latest_tof_line = None

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

                if line.startswith('WHEEL,'):
                    latest_wheel_line = line
                elif line.startswith('RAIL,'):
                    latest_rail_line = line
                elif line.startswith('IMU,'):
                    latest_imu_line = line
                elif line.startswith('TOF,'):
                    latest_tof_line = line
                else:
                    self.get_logger().debug(f'Unknown STM32 message: {line}')

            if latest_imu_line is not None:
                self.publish_imu_from_line(timestamp_now, latest_imu_line)

            if latest_rail_line is not None:
                self.publish_rail_from_line(timestamp_now, latest_rail_line)

            if latest_tof_line is not None:
                self.publish_tof_from_line(latest_tof_line)

            # While physically on the rail, keep wheel odometry frozen.
            if self.is_on_rail:
                self.publish_zero_odometry(timestamp_now, dt)
            elif latest_wheel_line is not None:
                self.publish_wheel_odom_from_line(
                    timestamp_now,
                    latest_wheel_line,
                    dt,
                )

        except Exception as error:
            self.get_logger().warning(f'Receive error: {error}')

    def publish_rail_from_line(self, timestamp_now, rail_line):
        parts = rail_line.split(',')
        if len(parts) != 3:
            return

        try:
            rm1_velocity = float(parts[1])
            rm2_velocity = float(parts[2])
        except ValueError:
            return

        if not math.isfinite(rm1_velocity) or not math.isfinite(rm2_velocity):
            return

        # Remove very small encoder noise while stopped.
        if abs(rm1_velocity) < 0.01:
            rm1_velocity = 0.0
        if abs(rm2_velocity) < 0.01:
            rm2_velocity = 0.0

        # 1) Raw rail motor speed topic
        speed_msg = Float32MultiArray()
        speed_msg.data = [rm1_velocity, rm2_velocity]
        self.pub_rail_speed.publish(speed_msg)

        # 2) 1-D rail odometry
        # Raw rail motors can spin even while the robot is not actually on the
        # rail. Only integrate distance in states that represent real rail
        # travel. ENTERING is intentionally excluded, so ON_RAIL starts at 0 m.
        measured_rail_velocity = (rm1_velocity + rm2_velocity) / 2.0
        rail_odom_active = self.rail_state in (
            'ON_RAIL',
            'WORKING',
            'EXITING',
        )

        if self.rail_odom_last_time is not None:
            dt = (
                timestamp_now - self.rail_odom_last_time
            ).nanoseconds * 1e-9

            if rail_odom_active and dt > 0.0:
                self.rail_odom_distance += measured_rail_velocity * dt

        # Always refresh the timestamp. Otherwise time spent OUT_RAIL/ENTERING
        # would be integrated as one large jump when ON_RAIL begins.
        self.rail_odom_last_time = timestamp_now

        odom_velocity = measured_rail_velocity if rail_odom_active else 0.0

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

        # Rail motion is modeled as one-dimensional motion along x.
        msg.pose.pose.position.x = distance
        msg.pose.pose.position.y = 0.0
        msg.pose.pose.position.z = 0.0

        # No rail heading estimation is needed.
        msg.pose.pose.orientation.w = 1.0

        msg.twist.twist.linear.x = linear_velocity
        msg.twist.twist.linear.y = 0.0
        msg.twist.twist.angular.z = 0.0

        # x is the meaningful dimension. Other DOFs are intentionally uncertain.
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

        if publish:
            self.publish_rail_odometry(
                self.rail_odom_last_time,
                0.0,
                0.0,
            )

        self.get_logger().info(
            'Rail odometry reset: distance = 0.000 m'
        )

    def publish_tof_from_line(self, tof_line):
        parts = tof_line.split(',')
        if len(parts) != 2:
            return

        try:
            distance_mm = float(parts[1])
        except ValueError:
            return

        msg = Float32()
        msg.data = distance_mm
        self.pub_tof.publish(msg)

    def publish_imu_from_line(self, timestamp_now, imu_line):
        parts = imu_line.split(',')
        if len(parts) != 4:
            return

        try:
            roll = float(parts[1])
            pitch = float(parts[2])
            yaw = float(parts[3])
        except ValueError:
            return

        quaternion = quaternion_from_euler(roll, pitch, yaw)

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

        # STM currently sends orientation only.
        message.angular_velocity_covariance[0] = -1.0
        message.linear_acceleration_covariance[0] = -1.0

        self.pub_imu.publish(message)

    def publish_wheel_odom_from_line(self, timestamp_now, wheel_line, dt):
        parts = wheel_line.split(',')
        if len(parts) != 3:
            return

        try:
            left_velocity = float(parts[1])
            right_velocity = float(parts[2])
        except ValueError:
            return

        if abs(left_velocity) < 0.01:
            left_velocity = 0.0
        if abs(right_velocity) < 0.01:
            right_velocity = 0.0

        if dt <= 0.0:
            dt = 0.001

        linear_velocity = (right_velocity + left_velocity) / 2.0
        angular_velocity = (
            right_velocity - left_velocity
        ) / self.wheel_separation

        delta_theta = angular_velocity * dt
        theta_mid = self.odom_pose.theta + delta_theta * 0.5

        self.odom_pose.x += linear_velocity * dt * math.cos(theta_mid)
        self.odom_pose.y += linear_velocity * dt * math.sin(theta_mid)
        self.odom_pose.theta += delta_theta
        self.odom_pose.theta = math.atan2(
            math.sin(self.odom_pose.theta),
            math.cos(self.odom_pose.theta),
        )

        self.publish_odometry(
            timestamp_now,
            linear_velocity,
            angular_velocity,
        )
        self.update_joint_states(
            timestamp_now,
            linear_velocity,
            angular_velocity,
            dt,
        )

        self.timestamp_previous = timestamp_now

    def publish_zero_odometry(self, timestamp_now, dt):
        if dt <= 0.0:
            dt = 0.001

        self.publish_odometry(timestamp_now, 0.0, 0.0)
        self.update_joint_states(timestamp_now, 0.0, 0.0, dt)
        self.timestamp_previous = timestamp_now

    def publish_odometry(self, timestamp_now, linear_velocity, angular_velocity):
        quaternion = quaternion_from_euler(0.0, 0.0, self.odom_pose.theta)

        odom = Odometry()
        odom.header.stamp = timestamp_now.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'

        odom.pose.pose.position.x = self.odom_pose.x
        odom.pose.pose.position.y = self.odom_pose.y
        odom.pose.pose.position.z = 0.0

        odom.pose.pose.orientation.x = quaternion[0]
        odom.pose.pose.orientation.y = quaternion[1]
        odom.pose.pose.orientation.z = quaternion[2]
        odom.pose.pose.orientation.w = quaternion[3]

        odom.twist.twist.linear.x = linear_velocity
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = angular_velocity

        pose_covariance = [0.0] * 36
        pose_covariance[0] = 0.10
        pose_covariance[7] = 0.10
        pose_covariance[14] = 1.0e6
        pose_covariance[21] = 1.0e6
        pose_covariance[28] = 1.0e6
        pose_covariance[35] = 0.20
        odom.pose.covariance = pose_covariance

        twist_covariance = [0.0] * 36
        twist_covariance[0] = 0.02
        twist_covariance[7] = 0.01
        twist_covariance[14] = 1.0e6
        twist_covariance[21] = 1.0e6
        twist_covariance[28] = 1.0e6
        twist_covariance[35] = 0.05
        odom.twist.covariance = twist_covariance

        self.pub_odom.publish(odom)


    def update_joint_states(
        self,
        timestamp_now,
        linear_velocity,
        angular_velocity,
        dt,
    ):
        left_linear_velocity = (
            linear_velocity
            - angular_velocity * self.wheel_separation / 2.0
        )
        right_linear_velocity = (
            linear_velocity
            + angular_velocity * self.wheel_separation / 2.0
        )

        left_angular_velocity = left_linear_velocity / self.wheel_radius
        right_angular_velocity = right_linear_velocity / self.wheel_radius

        self.joint.joint_pos[0] += left_angular_velocity * dt
        self.joint.joint_pos[1] += right_angular_velocity * dt
        self.joint.joint_vel = [
            left_angular_velocity,
            right_angular_velocity,
        ]

        message = JointState()
        message.header.stamp = timestamp_now.to_msg()
        message.header.frame_id = 'base_link'
        message.name = self.joint.joint_name
        message.position = self.joint.joint_pos
        message.velocity = self.joint.joint_vel

        self.pub_joint_states.publish(message)

    def _send_stop_commands(self):
        self.send_base_command(0.0, 0.0)
        self.send_rail_velocity(0.0)
        self._write_serial_line('STOP')


def main(args=None):
    rclpy.init(args=args)
    node = BringUp()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._send_stop_commands()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()