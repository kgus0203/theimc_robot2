#!/usr/bin/env python3

import math

import cv2
import numpy as np
import rclpy
import tf_transformations
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool, Int32, String
from tf2_ros import TransformBroadcaster


class ArucoDetectorNode(Node):
    """Robust ArUco detector for the rail robot docking test.

    Key points:
      * Reports whether image frames are actually arriving.
      * Reports IDs that are visible even when they are not the configured target.
      * Can automatically try common ArUco dictionaries when the configured
        dictionary does not find the target.
      * Uses CameraInfo when available and falls back to scaled D435-like
        intrinsics when CameraInfo is absent.
    """

    COMMON_DICTIONARIES = [
        'DICT_4X4_50',
        'DICT_4X4_100',
        'DICT_4X4_250',
        'DICT_5X5_50',
        'DICT_5X5_100',
        'DICT_5X5_250',
        'DICT_6X6_50',
        'DICT_6X6_100',
        'DICT_6X6_250',
    ]

    def __init__(self):
        super().__init__('aruco_detector_node')

        self.declare_parameter('image_topic', '/image_raw')
        self.declare_parameter('camera_info_topic', '/camera_info')
        self.declare_parameter('pose_topic', '/aruco_pose')
        self.declare_parameter('marker_size', 0.10)
        # -1 means accept the first detected marker. Keep 0 as the normal
        # docking target unless explicitly changed.
        self.declare_parameter('marker_id_to_detect', 0)
        self.declare_parameter('camera_frame_id', 'camera_color_optical_frame')
        self.declare_parameter('dictionary_name', 'DICT_5X5_250')
        self.declare_parameter('auto_dictionary_search', True)
        self.declare_parameter('auto_search_every_n_frames', 5)

        self.image_topic = str(self.get_parameter('image_topic').value)
        self.camera_info_topic = str(self.get_parameter('camera_info_topic').value)
        self.pose_topic = str(self.get_parameter('pose_topic').value)
        self.marker_size = float(self.get_parameter('marker_size').value)
        self.target_id = int(self.get_parameter('marker_id_to_detect').value)
        self.camera_frame_id = str(self.get_parameter('camera_frame_id').value)
        self.dictionary_name = str(self.get_parameter('dictionary_name').value)
        self.auto_dictionary_search = bool(
            self.get_parameter('auto_dictionary_search').value
        )
        self.auto_search_every_n_frames = max(
            1, int(self.get_parameter('auto_search_every_n_frames').value)
        )

        self.bridge = CvBridge()
        self.tf_broadcaster = TransformBroadcaster(self)

        image_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=2,
        )

        self.image_sub = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            image_qos,
        )
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self.camera_info_callback,
            image_qos,
        )

        self.pose_pub = self.create_publisher(PoseStamped, self.pose_topic, 10)
        self.detected_pub = self.create_publisher(Bool, '/aruco_detected', 10)
        self.detected_id_pub = self.create_publisher(Int32, '/aruco_detected_id', 10)
        self.status_pub = self.create_publisher(String, '/aruco_detector_status', 10)

        self.aruco_params = self._make_detector_parameters()
        self._dictionary_cache = {}
        self.active_dictionary_name = self.dictionary_name
        self._get_dictionary(self.active_dictionary_name)  # validate now

        # Fallback intrinsics. They are rescaled to the actual image size.
        self.camera_matrix = None
        self.dist_coeffs = np.zeros((5, 1), dtype=np.float64)
        self.have_camera_info = False

        s = self.marker_size / 2.0
        self.marker_3d_edges = np.array(
            [
                [-s,  s, 0.0],
                [ s,  s, 0.0],
                [ s, -s, 0.0],
                [-s, -s, 0.0],
            ],
            dtype=np.float32,
        )

        self.frame_count = 0
        self.last_image_log_ns = 0
        self.last_no_target_log_ns = 0
        self.last_detect_log_ns = 0

        self._publish_status(
            f'STARTED target_id={self.target_id} '
            f'dict={self.dictionary_name} image={self.image_topic}'
        )
        self.get_logger().info(
            '[ARUCO] started: image=%s pose=%s target_id=%d dict=%s auto_dict=%s',
            self.image_topic,
            self.pose_topic,
            self.target_id,
            self.dictionary_name,
            str(self.auto_dictionary_search),
        )

    def _make_detector_parameters(self):
        if hasattr(cv2.aruco, 'DetectorParameters'):
            params = cv2.aruco.DetectorParameters()
        else:
            params = cv2.aruco.DetectorParameters_create()

        # A little more permissive for a marker viewed from a few metres away.
        if hasattr(params, 'minMarkerPerimeterRate'):
            params.minMarkerPerimeterRate = 0.02
        if hasattr(params, 'cornerRefinementMethod') and hasattr(
            cv2.aruco, 'CORNER_REFINE_SUBPIX'
        ):
            params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        return params

    def _get_dictionary(self, name):
        if name in self._dictionary_cache:
            return self._dictionary_cache[name]

        if not hasattr(cv2.aruco, name):
            raise RuntimeError(f'Unsupported ArUco dictionary: {name}')

        dictionary_id = getattr(cv2.aruco, name)
        dictionary = cv2.aruco.getPredefinedDictionary(dictionary_id)
        self._dictionary_cache[name] = dictionary
        return dictionary

    def _publish_status(self, text):
        msg = String()
        msg.data = str(text)
        self.status_pub.publish(msg)

    def camera_info_callback(self, msg):
        if len(msg.k) != 9:
            return

        k = np.array(msg.k, dtype=np.float64).reshape((3, 3))
        if not np.isfinite(k).all() or k[0, 0] <= 0.0 or k[1, 1] <= 0.0:
            return

        self.camera_matrix = k
        if msg.d:
            self.dist_coeffs = np.array(msg.d, dtype=np.float64).reshape((-1, 1))
        else:
            self.dist_coeffs = np.zeros((5, 1), dtype=np.float64)

        if not self.have_camera_info:
            self.get_logger().info(
                '[ARUCO] CameraInfo received: fx=%.1f fy=%.1f cx=%.1f cy=%.1f',
                k[0, 0], k[1, 1], k[0, 2], k[1, 2],
            )
        self.have_camera_info = True

    def _camera_matrix_for_image(self, width, height):
        if self.have_camera_info and self.camera_matrix is not None:
            return self.camera_matrix

        # Original code assumed 640x480 with fx=fy=530. Scale the fallback
        # instead of keeping the same matrix for every resolution.
        sx = float(width) / 640.0
        sy = float(height) / 480.0
        return np.array(
            [
                [530.0 * sx, 0.0, width * 0.5],
                [0.0, 530.0 * sy, height * 0.5],
                [0.0, 0.0, 1.0],
            ],
            dtype=np.float64,
        )

    def _detect_with_dictionary(self, gray, dictionary_name):
        dictionary = self._get_dictionary(dictionary_name)

        # New and old OpenCV APIs are both supported.
        if hasattr(cv2.aruco, 'ArucoDetector'):
            detector = cv2.aruco.ArucoDetector(dictionary, self.aruco_params)
            return detector.detectMarkers(gray)

        return cv2.aruco.detectMarkers(
            gray,
            dictionary,
            parameters=self.aruco_params,
        )

    def _pick_target_index(self, ids):
        if ids is None or len(ids) == 0:
            return None

        flat = [int(v) for v in ids.flatten()]
        if self.target_id < 0:
            return 0

        try:
            return flat.index(self.target_id)
        except ValueError:
            return None

    def _detect_target(self, gray):
        # 1) Try the configured/current dictionary first on every frame.
        corners, ids, rejected = self._detect_with_dictionary(
            gray, self.active_dictionary_name
        )
        idx = self._pick_target_index(ids)
        if idx is not None:
            return self.active_dictionary_name, corners, ids, idx, rejected

        # If something was detected but it is the wrong ID, make this visible.
        if ids is not None and len(ids) > 0:
            now_ns = self.get_clock().now().nanoseconds
            if now_ns - self.last_no_target_log_ns > int(1.0e9):
                self.last_no_target_log_ns = now_ns
                visible = [int(v) for v in ids.flatten()]
                self.get_logger().warning(
                    '[ARUCO] marker(s) visible with %s: ids=%s, target_id=%d',
                    self.active_dictionary_name,
                    visible,
                    self.target_id,
                )

        # 2) Periodically try common dictionaries. This avoids multiplying
        # CPU load by N dictionaries on every image frame.
        if (
            not self.auto_dictionary_search
            or self.frame_count % self.auto_search_every_n_frames != 0
        ):
            return None

        for name in self.COMMON_DICTIONARIES:
            if name == self.active_dictionary_name:
                continue
            try:
                alt_corners, alt_ids, alt_rejected = self._detect_with_dictionary(
                    gray, name
                )
            except Exception:
                continue

            alt_idx = self._pick_target_index(alt_ids)
            if alt_idx is not None:
                old = self.active_dictionary_name
                self.active_dictionary_name = name
                self.get_logger().warning(
                    '[ARUCO] target found with dictionary %s (configured=%s). '
                    'Switching active dictionary %s -> %s',
                    name,
                    self.dictionary_name,
                    old,
                    name,
                )
                self._publish_status(
                    f'DICTIONARY_AUTO_SELECTED {name} target_id={self.target_id}'
                )
                return name, alt_corners, alt_ids, alt_idx, alt_rejected

        return None

    def image_callback(self, msg):
        self.frame_count += 1

        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().error('[ARUCO] CvBridge error: %s', str(exc))
            self._publish_status(f'CV_BRIDGE_ERROR {exc}')
            return

        if image is None or image.size == 0:
            return

        height, width = image.shape[:2]
        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self.last_image_log_ns > int(2.0e9):
            self.last_image_log_ns = now_ns
            self.get_logger().info(
                '[ARUCO] image alive: %dx%d frames=%d active_dict=%s',
                width,
                height,
                self.frame_count,
                self.active_dictionary_name,
            )

        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        detection = self._detect_target(gray)

        detected_msg = Bool()
        detected_msg.data = detection is not None
        self.detected_pub.publish(detected_msg)

        if detection is None:
            return

        dictionary_name, corners, ids, target_index, _ = detection
        marker_id = int(ids.flatten()[target_index])

        camera_matrix = self._camera_matrix_for_image(width, height)
        image_points = corners[target_index][0].astype(np.float32)

        flags = getattr(cv2, 'SOLVEPNP_IPPE_SQUARE', cv2.SOLVEPNP_ITERATIVE)
        success, rvec, tvec = cv2.solvePnP(
            self.marker_3d_edges,
            image_points,
            camera_matrix,
            self.dist_coeffs,
            flags=flags,
        )

        if not success:
            self.get_logger().warning('[ARUCO] solvePnP failed for marker %d', marker_id)
            return

        tvec = np.asarray(tvec, dtype=np.float64).reshape(3)
        if not np.isfinite(tvec).all() or tvec[2] <= 0.0:
            self.get_logger().warning('[ARUCO] invalid pose for marker %d: %s', marker_id, tvec)
            return

        rmat, _ = cv2.Rodrigues(rvec)
        transform_matrix = np.eye(4)
        transform_matrix[:3, :3] = rmat
        quat = tf_transformations.quaternion_from_matrix(transform_matrix)

        pose_msg = PoseStamped()
        pose_msg.header.stamp = msg.header.stamp
        pose_msg.header.frame_id = (
            msg.header.frame_id if msg.header.frame_id else self.camera_frame_id
        )
        pose_msg.pose.position.x = float(tvec[0])
        pose_msg.pose.position.y = float(tvec[1])
        pose_msg.pose.position.z = float(tvec[2])
        pose_msg.pose.orientation.x = float(quat[0])
        pose_msg.pose.orientation.y = float(quat[1])
        pose_msg.pose.orientation.z = float(quat[2])
        pose_msg.pose.orientation.w = float(quat[3])
        self.pose_pub.publish(pose_msg)

        id_msg = Int32()
        id_msg.data = marker_id
        self.detected_id_pub.publish(id_msg)

        tf_msg = TransformStamped()
        tf_msg.header = pose_msg.header
        tf_msg.child_frame_id = f'aruco_marker_{marker_id}'
        tf_msg.transform.translation.x = float(tvec[0])
        tf_msg.transform.translation.y = float(tvec[1])
        tf_msg.transform.translation.z = float(tvec[2])
        tf_msg.transform.rotation = pose_msg.pose.orientation
        self.tf_broadcaster.sendTransform(tf_msg)

        if now_ns - self.last_detect_log_ns > int(0.5e9):
            self.last_detect_log_ns = now_ns
            self.get_logger().info(
                '[ARUCO] DETECTED id=%d dict=%s x=%.3f y=%.3f z=%.3f',
                marker_id,
                dictionary_name,
                float(tvec[0]),
                float(tvec[1]),
                float(tvec[2]),
            )
            self._publish_status(
                f'DETECTED id={marker_id} dict={dictionary_name} '
                f'x={tvec[0]:.3f} y={tvec[1]:.3f} z={tvec[2]:.3f}'
            )


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()