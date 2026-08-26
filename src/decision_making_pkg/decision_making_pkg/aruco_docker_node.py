import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from std_srvs.srv import SetBool
from std_msgs.msg import Bool, String
import math

class ArucoDockerNode(Node):
    def __init__(self):
        super().__init__('aruco_docker_node')

        self.is_docking_active = False

        # --- [1] 목표 위치 및 각도 파라미터 ---
        self.target_z = 0.5     # 전후 거리 (20cm)
        self.target_x = 0.0   # 좌우 오프셋
        self.target_yaw = 0.0   # 목표 각도 (마커와 정면으로 평행)
        
        # --- [2] 제어 허용 오차 ---
        self.tol_z = 0.01    # 거리 오차 1cm 허용
        self.tol_x = 0.01    # 좌우 오차 1cm 허용
        self.tol_yaw = 0.02  # 각도 오차 약 3도 허용 (라디안)
        
        # --- [3] P 제어기 파라미터 ---
        self.k_linear = 0.4
        self.k_angular = 0.5 
        self.k_yaw = 1.0     
        
        self.max_linear_vel = 0.1
        self.max_angular_vel = 0.15
        self.min_linear_vel = 0.04
        self.min_angular_vel = 0.05

        # --- [4] ArUco marker search rotation ---
        # If the marker is not visible while docking is active, rotate roughly
        # one full turn. This is time-based because this node does not currently
        # subscribe to IMU/odom heading.
        self.declare_parameter('marker_lost_start_sec', 1.5)
        self.marker_lost_start_sec = float(
            self.get_parameter('marker_lost_start_sec').value
        )
        self.search_angular_vel = 0.15       # rad/s
        self.search_rotation_rad = 2.0 * math.pi
        self.search_timeout_sec = (
            self.search_rotation_rad / abs(self.search_angular_vel)
        )

        self.is_searching_marker = False
        self.search_start_time = None
        self.last_pose_log_time = None

        self.subscription = self.create_subscription(
            PoseStamped,
            '/aruco_pose',
            self.pose_callback,
            10
        )
        
        # self.cmd_pub = self.create_publisher(Twist, '/dock_vel', 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # GUI/BT command service.
        self.srv = self.create_service(
            SetBool,
            '/toggle_docking',
            self.toggle_callback
        )

        # Actual docking state/result topics.
        self.status_pub = self.create_publisher(
            String,
            '/docking_status',
            10
        )
        self.complete_pub = self.create_publisher(
            Bool,
            '/docking_complete',
            10
        )

        # --- [5] 마커 소실 / 탐색 회전 타이머 ---
        self.last_pose_time = self.get_clock().now()
        self.watchdog_timer = self.create_timer(0.5, self.check_timeout)

        self.get_logger().info("Aruco Docker Node Started! (Status: Standby)")
        self.publish_docking_status('STANDBY', completed=False)

    def publish_docking_status(self, status, completed=False):
        status_msg = String()
        status_msg.data = str(status)
        self.status_pub.publish(status_msg)

        complete_msg = Bool()
        complete_msg.data = bool(completed)
        self.complete_pub.publish(complete_msg)

    # 쿼터니언(x,y,z,w)을 오일러 각도(roll, pitch, yaw)로 변환
    def euler_from_quaternion(self, x, y, z, w):
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
        return yaw_z

    def toggle_callback(self, request, response):
        self.is_docking_active = request.data

        if self.is_docking_active:
            self.last_pose_time = self.get_clock().now()
            self.is_searching_marker = False
            self.search_start_time = None

            self.get_logger().info("▶️ 도킹 모드가 활성화되었습니다.")
            self.publish_docking_status('ACTIVE', completed=False)
            response.message = "Docking activated"
        else:
            self.is_searching_marker = False
            self.search_start_time = None

            self.get_logger().info(
                "⏸️ 도킹 모드가 비활성화되었습니다. 로봇을 정지합니다."
            )
            self.cmd_pub.publish(Twist())
            self.publish_docking_status('OFF', completed=False)
            response.message = "Docking deactivated and robot stopped"

        # IMPORTANT:
        # This means the ON/OFF command was accepted.
        # It does NOT mean physical docking is complete.
        response.success = True
        return response

    def check_timeout(self):
        if not self.is_docking_active:
            return

        now = self.get_clock().now()
        elapsed_since_pose = (
            now - self.last_pose_time
        ).nanoseconds / 1e9

        # Marker is still visible recently: normal docking control owns cmd_vel.
        if elapsed_since_pose <= self.marker_lost_start_sec:
            return

        # Marker lost / not yet found -> start one-turn search.
        if not self.is_searching_marker:
            self.is_searching_marker = True
            self.search_start_time = now

            self.get_logger().warn(
                "⚠️ ArUco 마커가 보이지 않습니다. "
                "한 바퀴 탐색 회전을 시작합니다."
            )
            self.publish_docking_status('SEARCHING', completed=False)

        elapsed_search = (
            now - self.search_start_time
        ).nanoseconds / 1e9

        if elapsed_search < self.search_timeout_sec:
            cmd = Twist()
            cmd.angular.z = self.search_angular_vel
            self.cmd_pub.publish(cmd)
            return

        # One approximate full rotation completed without detecting marker.
        self.cmd_pub.publish(Twist())
        self.is_searching_marker = False
        self.search_start_time = None
        self.is_docking_active = False

        self.get_logger().warn(
            "❌ 한 바퀴 탐색했지만 ArUco 마커를 찾지 못했습니다. 정지합니다."
        )
        self.publish_docking_status('MARKER_NOT_FOUND', completed=False)

    def pose_callback(self, msg):
        self.last_pose_time = self.get_clock().now()

        if not self.is_docking_active:
            return

        # Visible confirmation that /aruco_pose is actually arriving.
        now = self.get_clock().now()
        if (
            self.last_pose_log_time is None or
            (now - self.last_pose_log_time).nanoseconds / 1e9 >= 1.0
        ):
            self.last_pose_log_time = now
            self.get_logger().info(
                '✅ /aruco_pose 수신: x=%.3f y=%.3f z=%.3f',
                msg.pose.position.x,
                msg.pose.position.y,
                msg.pose.position.z,
            )

        # Marker found during search: immediately leave search mode.
        if self.is_searching_marker:
            self.is_searching_marker = False
            self.search_start_time = None
            self.cmd_pub.publish(Twist())

            self.get_logger().info(
                "✅ 탐색 중 ArUco 마커를 찾았습니다. 도킹 제어로 전환합니다."
            )
            self.publish_docking_status('ACTIVE', completed=False)

        # 1. 위치 및 각도 오차 계산
        current_z = msg.pose.position.z
        current_x = msg.pose.position.x
        error_z = current_z - self.target_z
        error_x = current_x - self.target_x
        
        q = msg.pose.orientation
        current_yaw = self.euler_from_quaternion(q.x, q.y, q.z, q.w)
        error_yaw = current_yaw - self.target_yaw
        
        cmd = Twist()
        
        # 2. 완료 조건 검사 (거리, 좌우, 각도 모두 허용 오차 이내일 때)
        if abs(error_z) <= self.tol_z and abs(error_x) <= self.tol_x and abs(error_yaw) <= self.tol_yaw:
            self.get_logger().info("✅ 도킹 완료")
            self.cmd_pub.publish(cmd)
            self.is_docking_active = False
            self.is_searching_marker = False
            self.search_start_time = None
            self.publish_docking_status('COMPLETED', completed=True)
            return
            
        # --- [3] 데드밴드(Deadband)가 적용된 속도 제어 ---
        
        # 3-1. 직진 속도 (V) 계산
        # 거리가 이미 허용 오차 이내라면 전후진 속도를 0으로 고정하여 탭댄스 현상 방지
        if abs(error_z) <= self.tol_z:
            cmd.linear.x = 0.0
        else:
            raw_linear_x = self.k_linear * error_z
            if raw_linear_x > 0:
                cmd.linear.x = max(min(raw_linear_x, self.max_linear_vel), self.min_linear_vel)
            elif raw_linear_x < 0:
                cmd.linear.x = min(max(raw_linear_x, -self.max_linear_vel), -self.min_linear_vel)

        # 3-2. 회전 속도 (W) 계산
        # 좌우 위치와 각도가 모두 맞았다면 회전을 멈춤
        if abs(error_x) <= self.tol_x and abs(error_yaw) <= self.tol_yaw:
            cmd.angular.z = 0.0
        else:
            raw_angular_z = (-self.k_angular * error_x) - (self.k_yaw * error_yaw)
            if raw_angular_z > 0:
                cmd.angular.z = max(min(raw_angular_z, self.max_angular_vel), self.min_angular_vel)
            elif raw_angular_z < 0:
                cmd.angular.z = min(max(raw_angular_z, -self.max_angular_vel), -self.min_angular_vel)
            else:
                cmd.angular.z = 0.0


        # 4. 명령 퍼블리시 및 로그 출력
        self.cmd_pub.publish(cmd)
        
        #self.get_logger().info(
        #    f"Err Z:{error_z:.2f}m X:{error_x:.2f}m Yaw:{math.degrees(error_yaw):.1f}° | Cmd V:{cmd.linear.x:.3f} W:{cmd.angular.z:.3f}",
        #    throttle_duration_sec=0.5
        #)

def main(args=None):
    rclpy.init(args=args)
    node = ArucoDockerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down Aruco Docker Node...")
        if rclpy.ok():
            try:
                node.cmd_pub.publish(Twist())
            except Exception:
                pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()