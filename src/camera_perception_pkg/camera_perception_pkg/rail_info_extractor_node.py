import math
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

from interfaces_pkg.msg import DetectionArray, RailInfo


class SimpleRailInfoExtractor(Node):
    """
    rail_start 하나를 선택해 RailInfo로 변환하는 단순 노드.

    규칙:
    - 항상 화면 중심에 가장 가까운 rail_start 하나를 선택한다.
    - 중심 X/Y와 각도만 EMA로 간단히 평활화한다.
    - 거리 구간은 화면 높이 기준으로 고정한다.
        FAR    : 0 ~ 50%
        MIDDLE : 50 ~ 75%
        NEAR   : 75 ~ 100%
    """

    def __init__(self):
        super().__init__('simple_rail_info_extractor_node')

        self.declare_parameter('enable', False)
        self.declare_parameter(
            'perception_enable_topic',
            '/rail_perception_enable',
        )
        self.declare_parameter('detections_topic', '/detections')
        self.declare_parameter('rail_info_topic', '/rail_info')
        self.declare_parameter('target_class_name', 'rail_start')

        self.declare_parameter('img_width', 640)
        self.declare_parameter('img_height', 480)
        self.declare_parameter('far_ratio', 0.40)
        # 이보다 가까우면 검출이 불안정해지므로 조금 일찍 near로 판정한다.
        self.declare_parameter('near_ratio', 0.70)

        self.declare_parameter('min_score', 0.25)
        self.declare_parameter('stale_sec', 0.40)
        self.declare_parameter('publish_hz', 20.0)
        self.declare_parameter('ema_alpha', 0.40)
        self.declare_parameter('angle_trim_ratio', 0.12)

        self.enabled = bool(self.get_parameter('enable').value)
        self.target_class_name = str(
            self.get_parameter('target_class_name').value
        )
        self.img_width = int(self.get_parameter('img_width').value)
        self.img_height = int(self.get_parameter('img_height').value)
        self.far_ratio = float(self.get_parameter('far_ratio').value)
        self.near_ratio = float(self.get_parameter('near_ratio').value)
        self.min_score = float(self.get_parameter('min_score').value)
        self.stale_sec = float(self.get_parameter('stale_sec').value)
        self.publish_hz = float(self.get_parameter('publish_hz').value)
        self.ema_alpha = float(self.get_parameter('ema_alpha').value)
        self.angle_trim_ratio = float(
            self.get_parameter('angle_trim_ratio').value
        )

        perception_enable_topic = str(
            self.get_parameter('perception_enable_topic').value
        )
        detections_topic = str(
            self.get_parameter('detections_topic').value
        )
        rail_info_topic = str(
            self.get_parameter('rail_info_topic').value
        )

        self.enable_sub = self.create_subscription(
            Bool,
            perception_enable_topic,
            self.enable_callback,
            10,
        )
        self.detections_sub = self.create_subscription(
            DetectionArray,
            detections_topic,
            self.detections_callback,
            10,
        )
        self.rail_info_pub = self.create_publisher(
            RailInfo,
            rail_info_topic,
            10,
        )
        self.timer = self.create_timer(
            1.0 / self.publish_hz,
            self.publish_rail_info,
        )

        self.last_header = None
        self.last_seen = None

        self.cx = None
        self.cy = None
        self.angle = None
        self.confidence = 0.0
        self.bbox_width = 0.0
        self.bbox_height = 0.0

        self.get_logger().info(
            '[SIMPLE_RAIL_INFO] ready '
            f'far<={self.far_ratio:.2f}, near>={self.near_ratio:.2f}'
        )

    def reset(self):
        self.last_seen = None
        self.cx = None
        self.cy = None
        self.angle = None
        self.confidence = 0.0
        self.bbox_width = 0.0
        self.bbox_height = 0.0

    def enable_callback(self, msg):
        self.enabled = bool(msg.data)
        if not self.enabled:
            self.reset()

        self.get_logger().info(
            '[SIMPLE_RAIL_INFO] '
            + ('ENABLED' if self.enabled else 'DISABLED')
        )

    def detections_callback(self, msg):
        self.last_header = msg.header

        if not self.enabled:
            return

        det = self.select_target(msg)
        if det is None:
            return

        score = self.get_confidence(det)
        raw_cx = self.get_bbox_center_x(det)
        raw_cy = self.get_bbox_center_y(det)
        raw_angle = self.estimate_angle(det)

        if self.cx is None:
            self.cx = raw_cx
            self.cy = raw_cy
            self.angle = raw_angle
        else:
            self.cx = self.ema(self.cx, raw_cx)
            self.cy = self.ema(self.cy, raw_cy)
            self.angle = self.ema(self.angle, raw_angle)

        self.confidence = score
        self.bbox_width = self.get_bbox_width(det)
        self.bbox_height = self.get_bbox_height(det)
        self.last_seen = time.monotonic()

    def select_target(self, msg):
        """화면 중심에 가장 가까운 rail_start를 선택한다."""
        image_center_x = self.img_width / 2.0
        candidates = []

        for det in msg.detections:
            if self.get_class_name(det) != self.target_class_name:
                continue

            score = self.get_confidence(det)
            if score < self.min_score:
                continue

            cx = self.get_bbox_center_x(det)
            center_error = abs(cx - image_center_x)

            # 중심 거리가 우선이고, 같으면 confidence가 높은 검출을 선택한다.
            candidates.append((center_error, -score, det))

        if not candidates:
            return None

        candidates.sort(key=lambda item: (item[0], item[1]))
        return candidates[0][2]

    def publish_rail_info(self):
        msg = RailInfo()

        if self.last_header is not None:
            msg.header = self.last_header

        msg.img_width = self.img_width
        msg.img_height = self.img_height
        msg.img_cx = self.img_width / 2.0
        msg.img_cy = self.img_height / 2.0

        alive = (
            self.enabled
            and self.last_seen is not None
            and (time.monotonic() - self.last_seen) <= self.stale_sec
            and self.cx is not None
            and self.cy is not None
            and self.angle is not None
        )

        if not alive:
            msg.has_rail = False
            msg.rail_cx = 0.0
            msg.rail_cy = 0.0
            msg.angle_deg = 0.0
            msg.distance = 'far'
            msg.confidence = 0.0
            msg.rail_bbox_width = 0.0
            msg.rail_bbox_height = 0.0
            msg.rail_bbox_area_ratio = 0.0
            self.rail_info_pub.publish(msg)
            return

        msg.has_rail = True
        msg.rail_cx = float(self.cx)
        msg.rail_cy = float(self.cy)
        msg.angle_deg = float(self.angle)
        msg.distance = self.distance_label(self.cy)
        msg.confidence = float(self.confidence)
        msg.rail_bbox_width = float(self.bbox_width)
        msg.rail_bbox_height = float(self.bbox_height)

        image_area = float(self.img_width * self.img_height)
        bbox_area = max(0.0, self.bbox_width) * max(0.0, self.bbox_height)
        msg.rail_bbox_area_ratio = (
            bbox_area / image_area if image_area > 0.0 else 0.0
        )

        self.rail_info_pub.publish(msg)

    def distance_label(self, cy):
        ratio = cy / max(float(self.img_height), 1.0)

        if ratio >= self.near_ratio:
            return 'near'
        if ratio >= self.far_ratio:
            return 'middle'
        return 'far'

    def ema(self, previous, new):
        return (
            (1.0 - self.ema_alpha) * previous
            + self.ema_alpha * new
        )

    def estimate_angle(self, det):
        """mask의 아래쪽 경계 중앙부를 강인하게 피팅해 각도를 구한다."""
        mask = getattr(det, 'mask', None)
        if mask is None or not hasattr(mask, 'data'):
            return 0.0

        polygon = []
        for point in mask.data:
            x = self.get_point_x(point)
            y = self.get_point_y(point)

            if x is not None and y is not None:
                polygon.append((x, y))

        if len(polygon) < 3:
            return 0.0

        polygon = np.asarray(polygon, dtype=np.float32)
        polygon[:, 0] = np.clip(polygon[:, 0], 0, self.img_width - 1)
        polygon[:, 1] = np.clip(polygon[:, 1], 0, self.img_height - 1)

        binary_mask = np.zeros(
            (self.img_height, self.img_width),
            dtype=np.uint8,
        )
        cv2.fillPoly(binary_mask, [polygon.astype(np.int32)], 255)

        _, nonzero_x = np.where(binary_mask > 0)
        if nonzero_x.size < 2:
            return 0.0

        x_min = int(nonzero_x.min())
        x_max = int(nonzero_x.max())
        trim = int((x_max - x_min + 1) * self.angle_trim_ratio)
        left = x_min + trim
        right = x_max - trim

        # 각 X열에서 가장 아래쪽 픽셀 하나만 사용한다.
        bottom_points = []
        for x in range(left, right + 1):
            ys = np.flatnonzero(binary_mask[:, x])
            if ys.size:
                bottom_points.append((float(x), float(ys[-1])))

        if len(bottom_points) < 5:
            return 0.0

        points = np.asarray(bottom_points, dtype=np.float32)

        # 주변 중앙값에서 크게 벗어난 점을 먼저 제거한다.
        y_values = points[:, 1]
        local_median = np.asarray([
            np.median(y_values[max(0, i - 4):i + 5])
            for i in range(len(y_values))
        ])
        deviation = np.abs(y_values - local_median)
        mad = float(np.median(deviation))
        points = points[deviation <= max(2.0, 3.0 * mad)]

        if len(points) < 5:
            return 0.0

        vx, vy, _, _ = cv2.fitLine(
            points,
            cv2.DIST_WELSCH,
            0,
            0.01,
            0.01,
        ).flatten()

        if vx < 0.0:
            vx = -vx
            vy = -vy

        return float(-math.degrees(math.atan2(float(vy), float(vx))))

    @staticmethod
    def get_class_name(det):
        for name in ('class_name', 'label', 'name'):
            if hasattr(det, name):
                return str(getattr(det, name))
        return ''

    @staticmethod
    def get_confidence(det):
        for name in ('score', 'confidence', 'conf'):
            if hasattr(det, name):
                return float(getattr(det, name))
        return 0.0

    @staticmethod
    def get_bbox_center_x(det):
        bbox = getattr(det, 'bbox', None)
        if bbox is None:
            return 0.0

        if hasattr(bbox, 'center'):
            center = bbox.center
            if hasattr(center, 'position') and hasattr(center.position, 'x'):
                return float(center.position.x)
            if hasattr(center, 'x'):
                return float(center.x)

        if hasattr(bbox, 'cx'):
            return float(bbox.cx)

        return 0.0

    @staticmethod
    def get_bbox_center_y(det):
        bbox = getattr(det, 'bbox', None)
        if bbox is None:
            return 0.0

        if hasattr(bbox, 'center'):
            center = bbox.center
            if hasattr(center, 'position') and hasattr(center.position, 'y'):
                return float(center.position.y)
            if hasattr(center, 'y'):
                return float(center.y)

        if hasattr(bbox, 'cy'):
            return float(bbox.cy)

        return 0.0

    @staticmethod
    def get_bbox_width(det):
        bbox = getattr(det, 'bbox', None)
        if bbox is None:
            return 0.0

        if hasattr(bbox, 'size') and hasattr(bbox.size, 'x'):
            return float(bbox.size.x)
        if hasattr(bbox, 'width'):
            return float(bbox.width)
        if hasattr(bbox, 'w'):
            return float(bbox.w)

        return 0.0

    @staticmethod
    def get_bbox_height(det):
        bbox = getattr(det, 'bbox', None)
        if bbox is None:
            return 0.0

        if hasattr(bbox, 'size') and hasattr(bbox.size, 'y'):
            return float(bbox.size.y)
        if hasattr(bbox, 'height'):
            return float(bbox.height)
        if hasattr(bbox, 'h'):
            return float(bbox.h)

        return 0.0

    @staticmethod
    def get_point_x(point):
        if hasattr(point, 'point') and hasattr(point.point, 'x'):
            return float(point.point.x)
        if hasattr(point, 'position') and hasattr(point.position, 'x'):
            return float(point.position.x)
        if hasattr(point, 'x'):
            return float(point.x)
        return None

    @staticmethod
    def get_point_y(point):
        if hasattr(point, 'point') and hasattr(point.point, 'y'):
            return float(point.point.y)
        if hasattr(point, 'position') and hasattr(point.position, 'y'):
            return float(point.position.y)
        if hasattr(point, 'y'):
            return float(point.y)
        return None


def main(args=None):
    rclpy.init(args=args)
    node = SimpleRailInfoExtractor()

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
