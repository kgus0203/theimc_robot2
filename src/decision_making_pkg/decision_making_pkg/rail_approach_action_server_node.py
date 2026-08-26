import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Bool

from interfaces_pkg.action import RailApproach
from interfaces_pkg.msg import RailInfo


class SimpleRailApproachAction(Node):
    """
    단순 레일 접근 제어.

    FAR / MIDDLE:
        중심이 많이 틀리면 짧게 중심 보정
        각도가 많이 틀리면 짧게 각도 보정
        둘 다 허용 범위이면 짧게 전진

    NEAR:
        Action goal의 x_tolerance / angle_tolerance로 최종 판정
        맞으면 성공
        안 맞으면 FAR까지 직선 후진

    의도적으로 제거한 기능:
        ALIGN_STABLE
        BACKUP_ALIGN_ANGLE
        거리별 복잡한 상태 전환
        고정 시간 후진
        중심/각도 혼합 제어
    """

    def __init__(self):
        super().__init__('simple_rail_approach_action_node')

        self.cb_group = ReentrantCallbackGroup()
        self.goal_lock = threading.Lock()
        self.goal_active = False

        # GUI 수동 테스트용:
        # /rail_approach_manual_success=true를 받으면
        # 현재 실행 중인 /rail_approach Action 자체를 SUCCESS 처리한다.
        self.manual_success_event = threading.Event()

        self.declare_parameter(
            'perception_enable_topic',
            '/rail_perception_enable',
        )
        self.declare_parameter('rail_info_topic', '/rail_info')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel_rail')
        self.declare_parameter('success_topic', '/rail_approach_success')
        self.declare_parameter(
            'manual_success_topic',
            '/rail_approach_manual_success',
        )

        self.declare_parameter('control_rate_hz', 20.0)
        self.declare_parameter('rail_timeout_sec', 0.5)

        # 직진으로 레일에 들어갈 수 있었던 실제 카메라 측정값.
        self.declare_parameter('target_x_error', -0.047)
        self.declare_parameter('target_angle_deg', 0.0)

        self.declare_parameter('turn_speed', 0.08)
        self.declare_parameter('forward_speed', 0.08)
        self.declare_parameter('backup_speed', 0.05)
        self.declare_parameter('backup_angle_tolerance', 1.0)

        self.declare_parameter('turn_pulse_sec', 0.08)
        self.declare_parameter('far_forward_sec', 0.50)
        self.declare_parameter('middle_forward_sec', 0.25)
        self.declare_parameter('success_hold_sec', 0.30)
        self.declare_parameter('near_fail_hold_sec', 0.30)
        self.declare_parameter('middle_align_timeout_sec', 2.0)

        self.perception_enable_topic = str(
            self.get_parameter('perception_enable_topic').value
        )
        self.rail_info_topic = str(
            self.get_parameter('rail_info_topic').value
        )
        self.cmd_vel_topic = str(
            self.get_parameter('cmd_vel_topic').value
        )
        self.success_topic = str(
            self.get_parameter('success_topic').value
        )
        self.manual_success_topic = str(
            self.get_parameter('manual_success_topic').value
        )

        self.control_rate_hz = float(
            self.get_parameter('control_rate_hz').value
        )
        self.rail_timeout_sec = float(
            self.get_parameter('rail_timeout_sec').value
        )
        self.target_x_error = float(
            self.get_parameter('target_x_error').value
        )
        self.target_angle_deg = float(
            self.get_parameter('target_angle_deg').value
        )

        self.turn_speed = float(
            self.get_parameter('turn_speed').value
        )
        self.forward_speed = float(
            self.get_parameter('forward_speed').value
        )
        self.backup_speed = float(
            self.get_parameter('backup_speed').value
        )
        self.backup_angle_tolerance = float(
            self.get_parameter('backup_angle_tolerance').value
        )

        self.turn_pulse_sec = float(
            self.get_parameter('turn_pulse_sec').value
        )
        self.far_forward_sec = float(
            self.get_parameter('far_forward_sec').value
        )
        self.middle_forward_sec = float(
            self.get_parameter('middle_forward_sec').value
        )
        self.success_hold_sec = float(
            self.get_parameter('success_hold_sec').value
        )
        self.near_fail_hold_sec = float(
            self.get_parameter('near_fail_hold_sec').value
        )
        self.middle_align_timeout_sec = float(
            self.get_parameter('middle_align_timeout_sec').value
        )

        self.latest_rail = None
        self.latest_rail_time = None

        self.perception_enable_pub = self.create_publisher(
            Bool,
            self.perception_enable_topic,
            10,
        )
        self.cmd_pub = self.create_publisher(
            Twist,
            self.cmd_vel_topic,
            10,
        )
        self.success_pub = self.create_publisher(
            Bool,
            self.success_topic,
            10,
        )
        self.rail_sub = self.create_subscription(
            RailInfo,
            self.rail_info_topic,
            self.rail_callback,
            10,
            callback_group=self.cb_group,
        )

        self.manual_success_sub = self.create_subscription(
            Bool,
            self.manual_success_topic,
            self.manual_success_callback,
            10,
            callback_group=self.cb_group,
        )

        self.action_server = ActionServer(
            self,
            RailApproach,
            'rail_approach',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.cb_group,
        )

        self.get_logger().info(
            '[SIMPLE_RAIL_ACTION] ready '
            f'target_x={self.target_x_error:.3f}, '
            f'target_angle={self.target_angle_deg:.2f}'
        )

    def manual_success_callback(self, msg):
        if not bool(msg.data):
            return

        with self.goal_lock:
            active = self.goal_active

        if not active:
            self.get_logger().warn(
                '[SIMPLE_RAIL_ACTION] manual success ignored: '
                'no active /rail_approach goal'
            )
            return

        self.manual_success_event.set()
        self.get_logger().warn(
            '[SIMPLE_RAIL_ACTION] GUI MANUAL SUCCESS requested'
        )

    def rail_callback(self, msg):
        self.latest_rail = msg
        self.latest_rail_time = time.monotonic()

    def goal_callback(self, request):
        with self.goal_lock:
            if self.goal_active:
                return GoalResponse.REJECT
            self.goal_active = True
            self.manual_success_event.clear()

        self.get_logger().info(
            '[SIMPLE_RAIL_ACTION] goal accepted '
            f'x_tol={request.x_tolerance:.3f}, '
            f'angle_tol={request.angle_tolerance:.3f}'
        )
        return GoalResponse.ACCEPT

    def cancel_callback(self, _goal_handle):
        self.stop_robot()
        return CancelResponse.ACCEPT

    def get_rail(self):
        if self.latest_rail is None or self.latest_rail_time is None:
            return None

        if time.monotonic() - self.latest_rail_time > self.rail_timeout_sec:
            return None

        if not bool(getattr(self.latest_rail, 'has_rail', False)):
            return None

        return self.latest_rail

    def extract(self, rail):
        width = float(getattr(rail, 'img_width', 0.0))
        rail_cx = float(getattr(rail, 'rail_cx', 0.0))
        img_cx = float(getattr(rail, 'img_cx', 0.0))

        measured_x_error = (
            (rail_cx - img_cx) / (width / 2.0)
            if width > 0.0
            else 0.0
        )

        # 카메라의 0이 아니라, 실제로 직진 진입이 됐던 값을 0으로 본다.
        x_error = measured_x_error - self.target_x_error
        angle_error = (
            float(getattr(rail, 'angle_deg', 0.0))
            - self.target_angle_deg
        )
        distance = str(
            getattr(rail, 'distance', 'far')
        ).strip().lower()

        if distance not in ('far', 'middle', 'near'):
            distance = 'far'

        return x_error, angle_error, distance

    @staticmethod
    def approach_limits(distance):
        if distance == 'far':
            return 0.20, 6.0
        return 0.12, 4.0

    def set_perception(self, enabled):
        msg = Bool()
        msg.data = bool(enabled)

        for _ in range(3):
            self.perception_enable_pub.publish(msg)
            time.sleep(0.02)

    def stop_robot(self):
        cmd = Twist()
        for _ in range(3):
            self.cmd_pub.publish(cmd)
            time.sleep(0.01)

    def publish_feedback(
        self,
        goal_handle,
        state,
        x_error,
        angle_error,
        distance,
    ):
        feedback = RailApproach.Feedback()
        feedback.state = state
        feedback.x_error = float(x_error)
        feedback.angle_error = float(angle_error)
        feedback.distance = distance
        goal_handle.publish_feedback(feedback)

    def publish_success(self):
        msg = Bool()
        msg.data = True
        self.success_pub.publish(msg)

    def check_terminal(self, goal_handle, deadline):
        if goal_handle.is_cancel_requested:
            return 'canceled'
        if self.manual_success_event.is_set():
            return 'manual_success'
        if time.monotonic() >= deadline:
            return 'timeout'
        if not rclpy.ok():
            return 'shutdown'
        return None

    def pulse(
        self,
        goal_handle,
        deadline,
        linear_x=0.0,
        angular_z=0.0,
        duration=0.0,
        stop_on_near=False,
    ):
        cmd = Twist()
        cmd.linear.x = float(linear_x)
        cmd.angular.z = float(angular_z)

        period = 1.0 / self.control_rate_hz
        end_time = time.monotonic() + duration

        while time.monotonic() < end_time:
            terminal = self.check_terminal(goal_handle, deadline)
            if terminal is not None:
                self.stop_robot()
                return terminal

            rail = self.get_rail()
            if rail is None:
                self.stop_robot()
                return 'rail_lost'

            x_error, angle_error, distance = self.extract(rail)
            state = 'FORWARD' if linear_x > 0.0 else 'ALIGN'
            self.publish_feedback(
                goal_handle,
                state,
                x_error,
                angle_error,
                distance,
            )

            if stop_on_near and distance == 'near':
                self.stop_robot()
                return 'near'

            self.cmd_pub.publish(cmd)
            time.sleep(period)

        self.stop_robot()
        return 'ok'

    def backup_until_far(self, goal_handle, deadline):
        cmd = Twist()
        cmd.linear.x = -abs(self.backup_speed)

        period = 1.0 / self.control_rate_hz
        far_since = None

        while rclpy.ok():
            terminal = self.check_terminal(goal_handle, deadline)
            if terminal is not None:
                self.stop_robot()
                return terminal

            rail = self.get_rail()
            cmd.angular.z = 0.0

            if rail is None:
                self.publish_feedback(
                    goal_handle,
                    'BACKUP',
                    0.0,
                    0.0,
                    'unknown',
                )
                far_since = None
            else:
                x_error, angle_error, distance = self.extract(rail)

                # 후진 중에는 중심 오차를 무시하고 각도만 0으로 맞춘다.
                if abs(angle_error) > self.backup_angle_tolerance:
                    cmd.angular.z = (
                        self.turn_speed
                        if angle_error > 0.0
                        else -self.turn_speed
                    )

                self.publish_feedback(
                    goal_handle,
                    'BACKUP',
                    x_error,
                    angle_error,
                    distance,
                )

                if distance == 'far':
                    if far_since is None:
                        far_since = time.monotonic()
                    elif time.monotonic() - far_since >= 0.20:
                        self.stop_robot()
                        return 'ok'
                else:
                    far_since = None

            self.cmd_pub.publish(cmd)
            time.sleep(period)

        self.stop_robot()
        return 'shutdown'

    def finish_manual_success(self, goal_handle, result):
        self.manual_success_event.clear()

        # 접근 제어가 내던 속도를 먼저 확실히 정지한다.
        self.stop_robot()

        # 자동 성공과 동일한 success topic도 발행한다.
        self.publish_success()

        # 핵심: 현재 실행 중인 /rail_approach Action goal 자체를 SUCCESS 처리.
        goal_handle.succeed()
        result.success = True
        result.reason = 'manual_rail_approach_success'

        self.get_logger().warn(
            '[SIMPLE_RAIL_ACTION] GUI MANUAL SUCCESS -> '
            '/rail_approach ACTION SUCCEEDED'
        )
        return result

    def finish_error(self, goal_handle, result, status):
        self.stop_robot()

        if status == 'canceled':
            goal_handle.canceled()
            result.reason = 'canceled'
        else:
            goal_handle.abort()
            result.reason = (
                'timeout'
                if status == 'timeout'
                else 'rclpy_shutdown'
            )

        result.success = False
        return result

    def execute_callback(self, goal_handle):
        request = goal_handle.request

        timeout_sec = (
            float(request.timeout_sec)
            if float(request.timeout_sec) > 0.0
            else 60.0
        )
        x_tolerance = (
            float(request.x_tolerance)
            if float(request.x_tolerance) > 0.0
            else 0.25
        )
        angle_tolerance = (
            float(request.angle_tolerance)
            if float(request.angle_tolerance) > 0.0
            else 5.0
        )
        allow_reverse = bool(
            getattr(request, 'allow_reverse_align', True)
        )

        deadline = time.monotonic() + timeout_sec
        period = 1.0 / self.control_rate_hz

        success_since = None
        fail_since = None
        middle_align_since = None

        result = RailApproach.Result()
        self.set_perception(True)

        try:
            while rclpy.ok():
                terminal = self.check_terminal(goal_handle, deadline)
                if terminal == 'manual_success':
                    return self.finish_manual_success(
                        goal_handle,
                        result,
                    )
                if terminal is not None:
                    return self.finish_error(
                        goal_handle,
                        result,
                        terminal,
                    )

                rail = self.get_rail()
                if rail is None:
                    self.stop_robot()
                    self.publish_feedback(
                        goal_handle,
                        'WAIT_RAIL',
                        0.0,
                        0.0,
                        'unknown',
                    )
                    success_since = None
                    fail_since = None
                    middle_align_since = None
                    time.sleep(period)
                    continue

                x_error, angle_error, distance = self.extract(rail)

                if distance == 'near':
                    middle_align_since = None
                    self.stop_robot()
                    self.publish_feedback(
                        goal_handle,
                        'FINAL_CHECK',
                        x_error,
                        angle_error,
                        distance,
                    )

                    near_ok = (
                        abs(x_error) <= x_tolerance
                        and abs(angle_error) <= angle_tolerance
                    )

                    if near_ok:
                        fail_since = None

                        if success_since is None:
                            success_since = time.monotonic()

                        if (
                            time.monotonic() - success_since
                            >= self.success_hold_sec
                        ):
                            self.publish_success()
                            goal_handle.succeed()
                            result.success = True
                            result.reason = 'rail_approach_success'
                            return result
                    else:
                        success_since = None

                        if fail_since is None:
                            fail_since = time.monotonic()

                        if (
                            time.monotonic() - fail_since
                            >= self.near_fail_hold_sec
                        ):
                            if not allow_reverse:
                                goal_handle.abort()
                                result.success = False
                                result.reason = 'near_alignment_failed'
                                return result

                            status = self.backup_until_far(
                                goal_handle,
                                deadline,
                            )
                            if status == 'manual_success':
                                return self.finish_manual_success(
                                    goal_handle,
                                    result,
                                )
                            if status != 'ok':
                                return self.finish_error(
                                    goal_handle,
                                    result,
                                    status,
                                )

                            fail_since = None

                    time.sleep(period)
                    continue

                success_since = None
                fail_since = None

                x_limit, angle_limit = self.approach_limits(distance)

                needs_alignment = (
                    abs(x_error) > x_limit
                    or abs(angle_error) > angle_limit
                )

                if (
                    distance == 'middle'
                    and needs_alignment
                    and allow_reverse
                ):
                    if middle_align_since is None:
                        middle_align_since = time.monotonic()
                    elif (
                        time.monotonic() - middle_align_since
                        >= self.middle_align_timeout_sec
                    ):
                        status = self.backup_until_far(
                            goal_handle,
                            deadline,
                        )
                        if status == 'manual_success':
                            return self.finish_manual_success(
                                goal_handle,
                                result,
                            )
                        if status != 'ok':
                            return self.finish_error(
                                goal_handle,
                                result,
                                status,
                            )

                        middle_align_since = None
                        continue
                else:
                    middle_align_since = None

                if abs(x_error) > x_limit:
                    angular = (
                        -self.turn_speed
                        if x_error > 0.0
                        else self.turn_speed
                    )
                    self.publish_feedback(
                        goal_handle,
                        'ALIGN_CENTER',
                        x_error,
                        angle_error,
                        distance,
                    )
                    status = self.pulse(
                        goal_handle,
                        deadline,
                        angular_z=angular,
                        duration=self.turn_pulse_sec,
                    )

                elif abs(angle_error) > angle_limit:
                    angular = (
                        self.turn_speed
                        if angle_error > 0.0
                        else -self.turn_speed
                    )
                    self.publish_feedback(
                        goal_handle,
                        'ALIGN_ANGLE',
                        x_error,
                        angle_error,
                        distance,
                    )
                    status = self.pulse(
                        goal_handle,
                        deadline,
                        angular_z=angular,
                        duration=self.turn_pulse_sec,
                    )

                else:
                    forward_sec = (
                        self.far_forward_sec
                        if distance == 'far'
                        else self.middle_forward_sec
                    )
                    self.publish_feedback(
                        goal_handle,
                        'FORWARD',
                        x_error,
                        angle_error,
                        distance,
                    )
                    status = self.pulse(
                        goal_handle,
                        deadline,
                        linear_x=self.forward_speed,
                        duration=forward_sec,
                        stop_on_near=True,
                    )

                if status == 'manual_success':
                    return self.finish_manual_success(
                        goal_handle,
                        result,
                    )

                if status in ('canceled', 'timeout', 'shutdown'):
                    return self.finish_error(
                        goal_handle,
                        result,
                        status,
                    )

                # rail_lost, near, ok 모두 정지 후 처음부터 다시 판단한다.

            return self.finish_error(
                goal_handle,
                result,
                'shutdown',
            )

        finally:
            self.stop_robot()
            self.set_perception(False)

            self.manual_success_event.clear()

            with self.goal_lock:
                self.goal_active = False


def main(args=None):
    rclpy.init(args=args)

    node = SimpleRailApproachAction()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        executor.shutdown()
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()