'''
last_pose_manager_node.py

1. 시작할 때 /home/jeff/theimc_robot/config/last_pose.yaml 읽기
2. 저장된 pose가 있으면 몇 초 뒤 /initialpose publish
3. 실행 중에는 /amcl_pose를 구독
4. pose가 들어오면 주기적으로 last_pose.yaml에 저장
'''
#!/usr/bin/env python3

import os
import yaml
import math

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node

from geometry_msgs.msg import PoseWithCovarianceStamped


class LastPoseManagerNode(Node):
    def __init__(self):
        super().__init__('last_pose_manager_node')

        self.declare_parameter(
            'pose_file',
            '/home/jeff/theimc_robot/config/last_pose.yaml'
        )
        self.declare_parameter('auto_publish_initial_pose', True)
        self.declare_parameter('publish_delay_sec', 5.0)
        self.declare_parameter('save_interval_sec', 2.0)
        self.declare_parameter('initial_pose_stamp_offset_sec', 0.2)

        # covariance가 너무 크면 위치가 불안정한 상태일 수 있으니 저장하지 않기 위한 기준
        self.declare_parameter('max_xy_covariance', 1.0)
        self.declare_parameter('max_yaw_covariance', 1.0)

        self.pose_file = self.get_parameter('pose_file').value
        self.auto_publish_initial_pose = self.get_parameter('auto_publish_initial_pose').value
        self.publish_delay_sec = self.get_parameter('publish_delay_sec').value
        self.save_interval_sec = self.get_parameter('save_interval_sec').value
        self.initial_pose_stamp_offset_sec = (
            self.get_parameter('initial_pose_stamp_offset_sec').value
        )
        self.max_xy_covariance = self.get_parameter('max_xy_covariance').value
        self.max_yaw_covariance = self.get_parameter('max_yaw_covariance').value

        self.last_pose_msg = None

        self.initialpose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            '/initialpose',
            10
        )

        self.amcl_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.amcl_pose_callback,
            10
        )

        self.save_timer = self.create_timer(
            self.save_interval_sec,
            self.save_last_pose
        )

        if self.auto_publish_initial_pose:
            self.init_timer = self.create_timer(
                self.publish_delay_sec,
                self.publish_saved_initial_pose_once
            )
        else:
            self.init_timer = None

        self.get_logger().info('LastPoseManagerNode started.')
        self.get_logger().info(f'pose_file: {self.pose_file}')

    def amcl_pose_callback(self, msg):
        self.last_pose_msg = msg

    def is_pose_reliable(self, msg):
        cov = msg.pose.covariance

        # covariance index
        # x variance: cov[0]
        # y variance: cov[7]
        # yaw variance: cov[35]
        x_cov = cov[0]
        y_cov = cov[7]
        yaw_cov = cov[35]

        if x_cov > self.max_xy_covariance:
            return False

        if y_cov > self.max_xy_covariance:
            return False

        if yaw_cov > self.max_yaw_covariance:
            return False

        return True

    def save_last_pose(self):
        if self.last_pose_msg is None:
            return

        if not self.is_pose_reliable(self.last_pose_msg):
            self.get_logger().warn(
                'AMCL pose covariance is too high. Skip saving last pose.',
                throttle_duration_sec=5.0
            )
            return

        msg = self.last_pose_msg
        pose = msg.pose.pose

        data = {
            'frame_id': msg.header.frame_id if msg.header.frame_id else 'map',
            'x': float(pose.position.x),
            'y': float(pose.position.y),
            'z': float(pose.position.z),
            'qx': float(pose.orientation.x),
            'qy': float(pose.orientation.y),
            'qz': float(pose.orientation.z),
            'qw': float(pose.orientation.w),
            'covariance': [float(v) for v in msg.pose.covariance],
            'stamp_sec': int(self.get_clock().now().to_msg().sec),
        }

        try:
            os.makedirs(os.path.dirname(self.pose_file), exist_ok=True)

            tmp_file = self.pose_file + '.tmp'

            with open(tmp_file, 'w') as f:
                yaml.safe_dump(data, f, default_flow_style=False)

            os.replace(tmp_file, self.pose_file)

        except Exception as e:
            self.get_logger().error(f'Failed to save last pose: {e}')

    def publish_saved_initial_pose_once(self):
        # AMCL은 lifecycle configure 단계에서 /initialpose 구독자를 만든다.
        # Nav2 구성이 늦어질 경우 첫 발행이 유실될 수 있으므로 구독자가
        # 확인될 때까지 이 타이머를 유지해 다음 주기에 다시 시도한다.
        if self.initialpose_pub.get_subscription_count() == 0:
            self.get_logger().warn(
                'AMCL /initialpose subscriber is not ready. Retrying...',
                throttle_duration_sec=5.0
            )
            return

        # 구독자가 준비된 뒤에는 한 번만 발행한다.
        if self.init_timer is not None:
            self.init_timer.cancel()
            self.init_timer = None

        if not os.path.exists(self.pose_file):
            self.get_logger().warn(
                f'No saved pose file found: {self.pose_file}'
            )
            return

        try:
            with open(self.pose_file, 'r') as f:
                data = yaml.safe_load(f)

            if data is None:
                self.get_logger().warn('Saved pose file is empty.')
                return

            msg = PoseWithCovarianceStamped()
            msg.header.stamp = (
                self.get_clock().now()
                - Duration(seconds=self.initial_pose_stamp_offset_sec)
            ).to_msg()
            msg.header.frame_id = data.get('frame_id', 'map')

            msg.pose.pose.position.x = float(data['x'])
            msg.pose.pose.position.y = float(data['y'])
            msg.pose.pose.position.z = float(data.get('z', 0.0))

            msg.pose.pose.orientation.x = float(data.get('qx', 0.0))
            msg.pose.pose.orientation.y = float(data.get('qy', 0.0))
            msg.pose.pose.orientation.z = float(data.get('qz', 0.0))
            msg.pose.pose.orientation.w = float(data.get('qw', 1.0))

            if 'covariance' in data:
                msg.pose.covariance = [float(v) for v in data['covariance']]
            else:
                # 저장된 covariance가 없을 경우 기본값
                msg.pose.covariance = [
                    0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
                    0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0685
                ]

            self.initialpose_pub.publish(msg)

            self.get_logger().info(
                f'Published saved initial pose: '
                f'x={msg.pose.pose.position.x:.3f}, '
                f'y={msg.pose.pose.position.y:.3f}'
            )

        except Exception as e:
            self.get_logger().error(f'Failed to publish saved initial pose: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = LastPoseManagerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        # Safe even when launch already shut the rclpy context down on SIGINT.
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
