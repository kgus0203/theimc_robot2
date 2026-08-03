import math
import serial
import sys
import time

import rclpy
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import String, Float32
from tf2_ros import TransformBroadcaster

def quaternion_from_euler(roll, pitch, yaw):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = [0.0] * 4
    q[0] = cy * cp * sr - sy * sp * cr
    q[1] = sy * cp * sr + cy * sp * cr
    q[2] = sy * cp * cr - cy * sp * sr
    q[3] = cy * cp * cr + sy * sp * sr
    return q


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

        self.declare_parameter('use_stm_odom', True)
        self.declare_parameter('use_stm_imu', True)
        self.declare_parameter('use_stm_wheel', True)

        self.use_stm_odom = self.get_parameter(
            'use_stm_odom'
        ).get_parameter_value().bool_value
        self.use_stm_imu = self.get_parameter(
            'use_stm_imu'
        ).get_parameter_value().bool_value
        self.use_stm_wheel = self.get_parameter(
            'use_stm_wheel'
        ).get_parameter_value().bool_value
        

        # Serial port
        self.serial_port = '/dev/stm32_link'
        self.baud_rate = 230400
        self.serial_timeout = 0.1

        try:
            self.pico_serial = serial.Serial(
                port=self.serial_port,
                baudrate=self.baud_rate,
                timeout=self.serial_timeout,
            )
            self.pico_serial.reset_input_buffer()
            self.pico_serial.reset_output_buffer()
            self.get_logger().info(
                f'Connected to {self.serial_port}'
            )
        except serial.SerialException as error:
            self.get_logger().error(
                f'Failed to open serial: {error}'
            )
            sys.exit(1)

        # Robot parameters.
        # Verify these against the actual robot dimensions.
        self.wheel_separation = 1.0
        self.wheel_radius = 0.085
        self.max_lin_vel_x = 0.5
        self.max_ang_vel_z = 1.0
        self.max_rail_vel = 0.5

        self.odom_pose = OdomPose()
        self.joint = Joint()
        self.timestamp_previous = self.get_clock().now()

        self.last_cmd_time = time.time()

        self.is_on_rail = False

        qos_profile = QoSProfile(
            depth=20,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )

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
        self.pub_rail_state = self.create_publisher(
            String,
            '/rail_state',
            qos_profile,
        )

        self.pub_tof = self.create_publisher(
            Float32,
            '/tof_distance',
            qos_profile,
        )
        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_timer(0.05, self.update_robot)

    def cb_cmd_vel_msg(self, cmd_vel_msg):
        vx = cmd_vel_msg.linear.x
        wz = cmd_vel_msg.angular.z

        if abs(vx) < 0.01:
            vx = 0.0
        if abs(wz) < 0.01:
            wz = 0.0

        vx = max(
            -self.max_lin_vel_x,
            min(self.max_lin_vel_x, vx),
        )
        wz = max(
            -self.max_ang_vel_z,
            min(self.max_ang_vel_z, wz),
        )

        self.send_serial_command('CMD', vx, wz)

    def cb_rail_cmd_msg(self, msg):
        command = msg.data.strip().upper()
        # OUT 명령어가 추가되었습니다.
        if command in ['DETECTED', 'FORWARD', 'BACK', 'STOP', 'OUT', 'ON']:
            try:
                pico_msg = command + '\n'
                self.pico_serial.write(pico_msg.encode('utf-8'))

                self.get_logger().info(
                    f'Sent [{command}] command to STM32.'
                )

            except serial.SerialException as error:
                self.get_logger().error(
                    f'Failed to send [{command}]: {error}'
                )
        else:
            self.get_logger().warning(
                f'Unknown rail command received: {command}'
            )

    def cb_cmd_rail_msg(self, cmd_rail_msg):
        # RAIL 명령어는 이미 여기서 Twist 메시지(속도값)를 통해 처리됩니다.
        rail_velocity_1 = cmd_rail_msg.linear.x
        rail_velocity_2 = cmd_rail_msg.angular.z

        if abs(rail_velocity_1) < 0.01:
            rail_velocity_1 = 0.0
        if abs(rail_velocity_2) < 0.01:
            rail_velocity_2 = 0.0

        rail_velocity_1 = max(
            -self.max_rail_vel,
            min(self.max_rail_vel, rail_velocity_1),
        )
        rail_velocity_2 = max(
            -self.max_rail_vel,
            min(self.max_rail_vel, rail_velocity_2),
        )

        self.send_serial_command(
            'RAIL',
            rail_velocity_1,
            rail_velocity_2,
        )

    def send_serial_command(self, command_type, value_1, value_2):
        try:
            command = (
                f'{command_type},{value_1:.3f},'
                f'{value_2:.3f}\n'
            )
            self.pico_serial.write(
                command.encode('utf-8')
            )
        except serial.SerialException as error:
            self.get_logger().error(
                f'Serial write failed: {error}'
            )

    def update_robot(self):
        """Receive STM32 data and publish raw wheel odometry/IMU."""
        timestamp_now = self.get_clock().now()
        dt = (
            timestamp_now - self.timestamp_previous
        ).nanoseconds * 1e-9

        if self.pico_serial.in_waiting <= 0:
            return

        try:
            latest_odom_line = None
            latest_wheel_line = None
            latest_imu_line = None
            latest_tof_line = None

            while self.pico_serial.in_waiting > 0:
                line = (
                    self.pico_serial.readline()
                    .decode('utf-8', errors='ignore')
                    .strip()
                )
                # print(f"Received from STM32: {line}")
                # self.get_logger().info(f'serial line =>  [{line}]')
                if self.use_stm_odom and 'ODOM' in line:
                    latest_odom_line = line
                elif self.use_stm_wheel and 'WHEEL' in line:
                    latest_wheel_line = line
                elif self.use_stm_imu and 'IMU' in line:
                    latest_imu_line = line
                elif 'TOF' in line:  
                    latest_tof_line = line
                elif any(keyword in line for keyword in ('ODOM', 'WHEEL', 'IMU')):
                    pass
                else:
                    if line.startswith('STATE,'):
                        rail_state = line.split(',', 1)[1].strip().upper()

                        if rail_state in ('ON_RAIL', 'OUT_RAIL','EXITING','ENTERING'):
                            self.get_logger().info(
                                f'현재 레일 상태 =>  [{rail_state}]'
                            )

                            if rail_state in ('ON_RAIL', 'ENTERING'):
                                self.is_on_rail = True
                            elif rail_state in ('OUT_RAIL', 'EXITING'):
                                self.is_on_rail = False

                            state_msg = String()
                            state_msg.data = rail_state
                            self.pub_rail_state.publish(state_msg)

                        else:
                            self.get_logger().warning(
                                f'Unknown rail state received: {line}'
                            )

                    else:
                        print(f"Unknown STM message => {line}")

            if latest_imu_line is not None:
                self.publish_imu_from_line(
                    timestamp_now,
                    latest_imu_line,
                )
            if self.is_on_rail:
                self.publish_zero_odometry(timestamp_now,dt)
            else:

                if latest_odom_line is not None:
                    self.publish_odom_from_line(
                        timestamp_now,
                        latest_odom_line,
                        dt,
                    )
                elif latest_wheel_line is not None:
                    self.publish_wheel_odom_from_line(
                        timestamp_now,
                        latest_wheel_line,
                        dt,
                    )

            if latest_tof_line is not None:
                self.publish_tof_from_line(latest_tof_line)

        except Exception as error:
            self.get_logger().warning(
                f'Receive error: {error}'
            )

    def publish_zero_odometry(self, timestamp_now, dt):
        self.publish_odometry(
            timestamp_now,0.0,0.0,)
        self.update_joint_states(
            timestamp_now, 0.0,0.0, dt)
        self.timestamp_previous = timestamp_now


    def publish_tof_from_line(self, tof_line):
        # 파싱 예시: "TOF: 150.50 mm"
        try:
            parts = tof_line.split(':')
            if len(parts) > 1:
                # " 150.50 mm" 형태에서 공백을 기준으로 분리하여 숫자만 추출
                dist_str = parts[1].strip().split(' ')[0]
                distance_mm = float(dist_str)
                
                msg = Float32()
                msg.data = distance_mm
                self.pub_tof.publish(msg)
        except (ValueError, IndexError):
            self.get_logger().debug(f'Failed to parse TOF data: {tof_line}')

    def publish_odom_from_line(
        self,
        timestamp_now,
        odom_line,
        dt,
    ):
        parts = odom_line.split(',')

        if len(parts) < 6:
            return

        try:
            index = parts.index('ODOM')

            self.odom_pose.x = float(parts[index + 1])
            self.odom_pose.y = float(parts[index + 2])
            self.odom_pose.theta = float(parts[index + 3])

            linear_velocity = float(parts[index + 4])
            angular_velocity = float(parts[index + 5])

        except (ValueError, IndexError):
            return

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

    def publish_wheel_odom_from_line(
        self,
        timestamp_now,
        wheel_line,
        dt,
    ):
        parts = wheel_line.split(',')

        if len(parts) < 3:
            return

        try:
            index = parts.index('WHEEL')

            right_velocity = float(parts[index + 2])
            left_velocity = float(parts[index + 1])

        except (ValueError, IndexError):
            return

        if abs(right_velocity) < 0.01:
            right_velocity = 0.0
        if abs(left_velocity) < 0.01:
            left_velocity = 0.0

        if dt <= 0.0:
            dt = 0.001

        linear_velocity = (right_velocity + left_velocity) / 2.0
        angular_velocity = (
            right_velocity - left_velocity
        ) / self.wheel_separation

        delta_theta = angular_velocity * dt
        theta_mid = self.odom_pose.theta + delta_theta * 0.5

        self.odom_pose.x += (
            linear_velocity * dt * math.cos(theta_mid)
        )
        self.odom_pose.y += (
            linear_velocity * dt * math.sin(theta_mid)
        )
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

    def publish_imu_from_line(self, timestamp_now, imu_line):
        parts = imu_line.split(',')

        try:
            index = parts.index('IMU')

            roll = float(parts[index + 1])
            pitch = float(parts[index + 2])
            yaw = float(parts[index + 3])

            angular_velocity = None
            linear_acceleration = None

            if len(parts) >= index + 7:
                angular_velocity = (
                    float(parts[index + 4]),
                    float(parts[index + 5]),
                    float(parts[index + 6]),
                )

            if len(parts) >= index + 10:
                linear_acceleration = (
                    float(parts[index + 7]),
                    float(parts[index + 8]),
                    float(parts[index + 9]),
                )

        except (ValueError, IndexError):
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

        if angular_velocity is not None:
            message.angular_velocity.x = angular_velocity[0]
            message.angular_velocity.y = angular_velocity[1]
            message.angular_velocity.z = angular_velocity[2]
            message.angular_velocity_covariance = [
                0.20, 0.0, 0.0,
                0.0, 0.20, 0.0,
                0.0, 0.0, 0.05,
            ]
        else:
            message.angular_velocity_covariance[0] = -1.0

        if linear_acceleration is not None:
            message.linear_acceleration.x = linear_acceleration[0]
            message.linear_acceleration.y = linear_acceleration[1]
            message.linear_acceleration.z = linear_acceleration[2]
            message.linear_acceleration_covariance = [
                0.50, 0.0, 0.0,
                0.0, 0.50, 0.0,
                0.0, 0.0, 0.50,
            ]
        else:
            message.linear_acceleration_covariance[0] = -1.0

        self.pub_imu.publish(message)

    def publish_odometry(
        self,
        timestamp_now,
        linear_velocity,
        angular_velocity,
    ):
        quaternion = quaternion_from_euler(
            0.0,
            0.0,
            self.odom_pose.theta,
        )

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

        t = TransformStamped()
        t.header.stamp = timestamp_now.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'

        t.transform.translation.x = self.odom_pose.x
        t.transform.translation.y = self.odom_pose.y
        t.transform.translation.z = 0.0

        t.transform.rotation.x = quaternion[0]
        t.transform.rotation.y = quaternion[1]
        t.transform.rotation.z = quaternion[2]
        t.transform.rotation.w = quaternion[3]

        #self.tf_broadcaster.sendTransform(t)

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

        left_angular_velocity = (
            left_linear_velocity / self.wheel_radius
        )
        right_angular_velocity = (
            right_linear_velocity / self.wheel_radius
        )

        self.joint.joint_pos[0] += (
            left_angular_velocity * dt
        )
        self.joint.joint_pos[1] += (
            right_angular_velocity * dt
        )
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
        self.send_serial_command('CMD', 0.0, 0.0)
        self.send_serial_command('RAIL', 0.0, 0.0)


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