import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped, TransformStamped
from std_msgs.msg import Bool
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from cv_bridge import CvBridge
import cv2
import numpy as np
import tf_transformations
from tf2_ros import TransformBroadcaster

class ArucoDetectorNode(Node):
    def __init__(self):
        super().__init__('aruco_detector_node')

        # --- 파라미터 선언 ---
        self.declare_parameter('image_topic', '/image_raw')
        self.declare_parameter('pose_topic', '/aruco_pose')
        self.declare_parameter('marker_size', 0.10)  # 마커의 실제 물리적 크기 (단위: 미터, 10cm)
        self.declare_parameter('marker_id_to_detect', 0)  # 충전 스테이션에 부착된 마커 ID
        self.declare_parameter('camera_frame_id', 'camera_color_optical_frame') # 카메라 프레임

        self.image_topic = self.get_parameter('image_topic').value
        self.pose_topic = self.get_parameter('pose_topic').value
        self.marker_size = self.get_parameter('marker_size').value
        self.target_id = self.get_parameter('marker_id_to_detect').value
        self.camera_frame_id = self.get_parameter('camera_frame_id').value

        # --- [필수] 카메라 캘리브레이션 파라미터 입력 ---
        # RealSense D435 기본 해상도(640x480) 기준 예시 값입니다. 
        self.camera_matrix = np.array([[530.0,   0.0, 320.0],
                                       [  0.0, 530.0, 240.0],
                                       [  0.0,   0.0,   1.0]], dtype=np.float32)
        self.dist_coeffs = np.zeros((5, 1), dtype=np.float32)

        # --- ROS 2 구성 ---
        self.bridge = CvBridge()
        
        # image_publisher.py와 호환되도록 BEST_EFFORT QoS 프로필 적용
        self.image_qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1
        )

        # 평소에는 /image_raw을 구독하지 않음.
        # BT의 ToggleDocking(true) -> aruco_docker_node ->
        # /aruco_detector_enable=true일 때만 subscription 생성.
        self.subscription = None
        self.detector_enabled = False

        enable_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )

        self.enable_subscription = self.create_subscription(
            Bool,
            '/aruco_detector_enable',
            self.enable_callback,
            enable_qos
        )

        self.pose_pub = self.create_publisher(PoseStamped, self.pose_topic, 10)
        
        # TF 브로드캐스터 생성
        self.tf_broadcaster = TransformBroadcaster(self)

        # ArUco 사전 정의 
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_250)
        self.aruco_params = cv2.aruco.DetectorParameters_create()
        
        # OpenCV 4.7.0 이상 대응 객체 생성
        #self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)

        # 3D 마커 코너 좌표 정의 (마커 중심 기준)
        s = self.marker_size / 2.0
        self.marker_3d_edges = np.array([
            [-s,  s, 0],  
            [ s,  s, 0],  
            [ s, -s, 0],  
            [-s, -s, 0]   
        ], dtype=np.float32)

        self.get_logger().info(
            f"ArucoDetectorNode started IDLE. Target ID: {self.target_id}. "
            "/image_raw is OFF until docking starts."
        )

    def enable_callback(self, msg):
        if bool(msg.data):
            self.enable_detector()
        else:
            self.disable_detector()

    def enable_detector(self):
        if self.subscription is None:
            self.subscription = self.create_subscription(
                Image,
                self.image_topic,
                self.image_callback,
                self.image_qos_profile
            )

        self.detector_enabled = True
        self.get_logger().info(
            f"[ARUCO] ENABLED -> subscribed to {self.image_topic}"
        )

    def disable_detector(self):
        self.detector_enabled = False

        if self.subscription is not None:
            try:
                self.destroy_subscription(self.subscription)
            except Exception as e:
                self.get_logger().warning(
                    f"[ARUCO] destroy_subscription failed: {e}"
                )
            self.subscription = None

        self.get_logger().info(
            "[ARUCO] DISABLED -> /image_raw subscription removed"
        )

    def image_callback(self, msg):
        if not self.detector_enabled:
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"CvBridge Error: {e}")
            return

        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        
        # 마커 검출
        corners, ids, rejected = cv2.aruco.detectMarkers(
            gray, 
            self.aruco_dict, 
            parameters=self.aruco_params
        )

        if ids is not None:
            for i, marker_id in enumerate(ids.flatten()):
                if marker_id == self.target_id:
                    # solvePnP를 이용한 3D Pose 추정
                    success, rvec, tvec = cv2.solvePnP(
                        self.marker_3d_edges, 
                        corners[i][0], 
                        self.camera_matrix, 
                        self.dist_coeffs
                    )

                    if success:
                        x, y, z = tvec[0][0], tvec[1][0], tvec[2][0]

                        # rvec을 쿼터니언 회전 정보로 변환
                        rmat, _ = cv2.Rodrigues(rvec)
                        transform_matrix = np.eye(4)
                        transform_matrix[:3, :3] = rmat
                        quat = tf_transformations.quaternion_from_matrix(transform_matrix)

                        # 1. PoseStamped 메시지 퍼블리시
                        pose_msg = PoseStamped()
                        # msg.header가 유효하지 않을 경우를 대비해 파라미터 프레임 ID 사용
                        pose_msg.header.stamp = msg.header.stamp
                        pose_msg.header.frame_id = msg.header.frame_id if msg.header.frame_id else self.camera_frame_id
                        
                        pose_msg.pose.position.x = float(x)
                        pose_msg.pose.position.y = float(y)
                        pose_msg.pose.position.z = float(z)
                        pose_msg.pose.orientation.x = quat[0]
                        pose_msg.pose.orientation.y = quat[1]
                        pose_msg.pose.orientation.z = quat[2]
                        pose_msg.pose.orientation.w = quat[3]

                        self.pose_pub.publish(pose_msg)

                        # 2. TF (TransformStamped) 브로드캐스트 (rqt_tf_tree 및 rviz용)
                        t = TransformStamped()
                        t.header.stamp = msg.header.stamp
                        t.header.frame_id = pose_msg.header.frame_id
                        t.child_frame_id = f'aruco_marker_{marker_id}'

                        t.transform.translation.x = float(x)
                        t.transform.translation.y = float(y)
                        t.transform.translation.z = float(z)
                        t.transform.rotation.x = quat[0]
                        t.transform.rotation.y = quat[1]
                        t.transform.rotation.z = quat[2]
                        t.transform.rotation.w = quat[3]

                        self.tf_broadcaster.sendTransform(t)

                        # 디버그 로그
                        # self.get_logger().info(
                        #     f"Marker {marker_id} X:{x:.3f}, Y:{y:.3f}, Z:{z:.3f}",
                        #     throttle_duration_sec=0.5
                        # )

                        # 카메라 피드 시각화
                        cv2.drawFrameAxes(cv_image, self.camera_matrix, self.dist_coeffs, rvec, tvec, 0.05)
                        cv2.aruco.drawDetectedMarkers(cv_image, corners)
                        
        # 필요 시 디버그용 이미지 윈도우 활성화
        # cv2.imshow("ArUco Debug", cv_image)
        # cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.disable_detector()
        except Exception:
            pass

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()