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
        최신 중심/각도 오차로 전진과 회전을 동시에 연속 제어
        조향량이 클수록 전진 속도를 낮춰 캐스터가 자연스럽게 따라오게 함

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

        self.declare_parameter('turn_speed', 0.035)
        self.declare_parameter('forward_speed', 0.07)
        self.declare_parameter('min_forward_speed', 0.05)
        self.declare_parameter('min_rail_confidence', 0.45)
        # 접근 제어는 최종 성공 허용치보다 더 엄격하게 맞춘다.
        self.declare_parameter('control_tolerance_ratio', 0.65)
        self.declare_parameter('min_control_x_tolerance', 0.06)
        self.declare_parameter('min_control_angle_tolerance', 1.5)
        self.declare_parameter('backup_speed', 0.05)
        self.declare_parameter('backup_angle_tolerance', 1.0)

        self.declare_parameter('success_hold_sec', 0.30)
        self.declare_parameter('near_fail_hold_sec', 1.00)

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
        self.min_forward_speed = float(
            self.get_parameter('min_forward_speed').value
        )
        self.min_rail_confidence = float(
            self.get_parameter('min_rail_confidence').value
        )
        self.control_tolerance_ratio = float(
            self.get_parameter('control_tolerance_ratio').value
        )
        self.min_control_x_tolerance = float(
            self.get_parameter('min_control_x_tolerance').value
        )
        self.min_control_angle_tolerance = float(
            self.get_parameter('min_control_angle_tolerance').value
        )
        self.backup_speed = float(
            self.get_parameter('backup_speed').value
        )
        self.backup_angle_tolerance = float(
            self.get_parameter('backup_angle_tolerance').value
        )

        self.success_hold_sec = float(
            self.get_parameter('success_hold_sec').value
        )
        self.near_fail_hold_sec = float(
            self.get_parameter('near_fail_hold_sec').value
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
                self.get_logger().warn(
                    '[SIMPLE_RAIL_ACTION] goal rejected: '
                    'another goal is active'
                )
                return GoalResponse.REJECT

        self.get_logger().info(
            '[SIMPLE_RAIL_ACTION] goal request accepted '
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

        confidence = float(getattr(self.latest_rail, 'confidence', 0.0))
        if confidence < self.min_rail_confidence:
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
        result = RailApproach.Result()
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

        # goal_callback에서는 실행 상태를 선점하지 않는다. 요청이 수락된 뒤
        # 실행 콜백이 호출되지 않는 경우에도 goal_active가 고착되지 않도록
        # 실제 제어가 시작되는 이 지점에서만 활성 상태를 잡는다.
        with self.goal_lock:
            busy = self.goal_active
            if not busy:
                self.goal_active = True
                self.manual_success_event.clear()

        if busy:
            goal_handle.abort()
            result.success = False
            result.reason = 'another_goal_active'
            return result

        try:
            self.set_perception(True)

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
                    time.sleep(period)
                    continue

                x_error, angle_error, distance = self.extract(rail)

                x_limit, angle_limit = self.approach_limits(distance)

                # Action goal tolerance는 최종 성공 판정값이다. 접근 중에는
                # 이보다 엄격한 값을 사용해 FINAL_CHECK 전에 여유를 확보한다.
                x_limit = min(
                    x_limit,
                    max(
                        self.min_control_x_tolerance,
                        x_tolerance * self.control_tolerance_ratio,
                    ),
                )
                angle_limit = min(
                    angle_limit,
                    max(
                        self.min_control_angle_tolerance,
                        angle_tolerance * self.control_tolerance_ratio,
                    ),
                )

                prefinal_x_aligned = abs(x_error) <= x_limit

                # NEAR에서는 더 이상 전진하지 않는다. 레일을 카메라 시야에서
                # 잃지 않도록 x 오차만 저속 제자리 회전으로 맞춘다.
                if distance == 'near' and not prefinal_x_aligned:
                    success_since = None
                    fail_since = None

                    center_term = max(
                        -1.0,
                        min(1.0, -x_error / max(x_limit, 1.0e-6)),
                    )

                    self.publish_feedback(
                        goal_handle,
                        'FINAL_ALIGN_X',
                        x_error,
                        angle_error,
                        distance,
                    )
                    cmd = Twist()
                    cmd.linear.x = 0.0
                    cmd.angular.z = self.turn_speed * center_term
                    self.cmd_pub.publish(cmd)
                    time.sleep(period)
                    continue

                # x 오차가 제어 허용치 안에 들어온 뒤 완전히 정지한 상태로
                # 액션 목표의 x/angle tolerance를 최종 판정한다.
                if distance == 'near':
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

                needs_alignment = (
                    abs(x_error) > x_limit
                    or abs(angle_error) > angle_limit
                )

                # ArUco 제어처럼 중심 오차와 각도 오차를 한 조향값으로
                # 합친다. 기존 제어의 부호 규칙은 그대로 유지한다.
                center_term = max(
                    -1.0,
                    min(1.0, -x_error / max(x_limit, 1.0e-6)),
                )
                angle_term = max(
                    -1.0,
                    min(1.0, angle_error / max(angle_limit, 1.0e-6)),
                )
                # Nav2가 이미 정면에 접근하므로 큰 회전보다 중심 추종을
                # 우선하고, 검출 각도는 미세 보정으로만 사용한다.
                normalized_steer = 0.70 * center_term + 0.30 * angle_term

                if needs_alignment:
                    # 두 오차가 서로 상쇄되어 조향이 사라지는 경우에는
                    # 허용 범위를 더 크게 벗어난 오차의 방향을 따른다.
                    if abs(normalized_steer) < 0.25:
                        normalized_steer = (
                            center_term
                            if abs(center_term) >= abs(angle_term)
                            else angle_term
                        )
                    target_angular = self.turn_speed * normalized_steer
                    state = 'GUIDED_FORWARD'
                else:
                    target_angular = 0.0
                    state = 'FORWARD'

                # 최대 조향일 때도 절반 속도로 전진해 캐스터 방향을 잡는다.
                steer_ratio = (
                    min(1.0, abs(target_angular) / self.turn_speed)
                    if self.turn_speed > 0.0
                    else 0.0
                )
                # MIDDLE에서는 NEAR 판정 전에 미세 조정할 시간을 확보한다.
                distance_max_linear = (
                    self.forward_speed
                    if distance == 'far'
                    else max(
                        self.min_forward_speed,
                        self.forward_speed * 0.75,
                    )
                )
                target_linear = max(
                    self.min_forward_speed,
                    distance_max_linear * (1.0 - 0.6 * steer_ratio),
                )

                self.publish_feedback(
                    goal_handle,
                    state,
                    x_error,
                    angle_error,
                    distance,
                )
                cmd = Twist()
                cmd.linear.x = target_linear
                cmd.angular.z = target_angular
                self.cmd_pub.publish(cmd)
                time.sleep(period)
                continue

            return self.finish_error(
                goal_handle,
                result,
                'shutdown',
            )

        finally:
            try:
                self.stop_robot()
                self.set_perception(False)
            finally:
                # 정지/퍼셉션 publisher에서 예외가 발생하더라도 다음 goal을
                # 영구 거절하지 않도록 활성 상태 해제를 반드시 수행한다.
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
