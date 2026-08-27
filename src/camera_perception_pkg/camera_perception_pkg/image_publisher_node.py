import os
import sys
import time
import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSReliabilityPolicy

from sensor_msgs.msg import Image
from std_msgs.msg import Header
from cv_bridge import CvBridge

try:
    import pyrealsense2 as rs
except ImportError:
    rs = None


# --------------- Variable Setting ---------------
PUB_TOPIC_NAME = 'image_raw'

# realsense / image / video
DATA_SOURCE = 'realsense'

IMAGE_DIRECTORY_PATH = 'src/camera_perception_pkg/camera_perception_pkg/lib/Collected_Datasets/sample_dataset'
VIDEO_FILE_PATH = 'src/camera_perception_pkg/camera_perception_pkg/lib/Collected_Datasets/sangju.mp4'

SHOW_IMAGE = False

# 안정성 테스트용으로 일단 15fps 추천
TIMER = 1.0 / 15.0

# RealSense D435 안정 시작값
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
CAMERA_FPS = 15

FRAME_ID = 'camera_color_optical_frame'

# 이 노드가 사용할 RealSense 카메라를 serial로 고정한다.
REALSENSE_SERIAL = '244622071272'

# RealSense frame wait timeout
# 기존 1000ms는 너무 길어서 callback이 오래 막힘
REALSENSE_TIMEOUT_MS = 100

# 연속 실패 몇 번이면 pipeline 재시작할지
MAX_REALSENSE_FAIL_COUNT = 10
# ------------------------------------------------


class ImagePublisherNode(Node):
    def __init__(
        self,
        data_source=DATA_SOURCE,
        img_dir=IMAGE_DIRECTORY_PATH,
        video_path=VIDEO_FILE_PATH,
        pub_topic=PUB_TOPIC_NAME,
        logger=SHOW_IMAGE,
        timer=TIMER,
        frame_width=FRAME_WIDTH,
        frame_height=FRAME_HEIGHT,
        camera_fps=CAMERA_FPS,
        frame_id=FRAME_ID,
        realsense_serial=REALSENSE_SERIAL,
        realsense_timeout_ms=REALSENSE_TIMEOUT_MS,
        max_realsense_fail_count=MAX_REALSENSE_FAIL_COUNT,
    ):
        super().__init__('image_publisher_node')

        self.declare_parameter('data_source', data_source)
        self.declare_parameter('img_dir', img_dir)
        self.declare_parameter('video_path', video_path)
        self.declare_parameter('pub_topic', pub_topic)
        self.declare_parameter('logger', logger)
        self.declare_parameter('timer', timer)
        self.declare_parameter('frame_width', frame_width)
        self.declare_parameter('frame_height', frame_height)
        self.declare_parameter('camera_fps', camera_fps)
        self.declare_parameter('frame_id', frame_id)
        self.declare_parameter('realsense_serial', realsense_serial)
        self.declare_parameter('realsense_timeout_ms', realsense_timeout_ms)
        self.declare_parameter('max_realsense_fail_count', max_realsense_fail_count)

        self.data_source = self.get_parameter('data_source').get_parameter_value().string_value
        self.img_dir = self.get_parameter('img_dir').get_parameter_value().string_value
        self.video_path = self.get_parameter('video_path').get_parameter_value().string_value
        self.pub_topic = self.get_parameter('pub_topic').get_parameter_value().string_value
        self.logger = self.get_parameter('logger').get_parameter_value().bool_value
        self.timer_period = self.get_parameter('timer').get_parameter_value().double_value
        self.frame_width = self.get_parameter('frame_width').get_parameter_value().integer_value
        self.frame_height = self.get_parameter('frame_height').get_parameter_value().integer_value
        self.camera_fps = self.get_parameter('camera_fps').get_parameter_value().integer_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.realsense_serial = (
            self.get_parameter('realsense_serial').get_parameter_value().string_value
        )
        self.realsense_timeout_ms = self.get_parameter('realsense_timeout_ms').get_parameter_value().integer_value
        self.max_realsense_fail_count = (
            self.get_parameter('max_realsense_fail_count')
            .get_parameter_value().integer_value
        )

        # 카메라 영상은 BEST_EFFORT / depth=1 권장
        # 프레임 하나 놓쳐도 최신 프레임을 받는 게 중요함
        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1
        )

        self.br = CvBridge()

        self.cap = None
        self.pipeline = None
        self.rs_config = None
        self.realsense_started = False
        self.realsense_fail_count = 0

        self.img_list = []
        self.img_num = 0

        # publisher는 무조건 한 번만 생성하고 유지
        # 카메라가 끊겨도 publisher 자체는 사라지지 않게 함
        self.publisher = self.create_publisher(Image, self.pub_topic, self.qos_profile)

        if self.data_source == 'realsense':
            self._init_realsense()
        elif self.data_source == 'video':
            self._init_video()
        elif self.data_source == 'image':
            self._init_image_dir()
        else:
            self.get_logger().error(
                f"Wrong data source: {self.data_source}. Use 'realsense', 'image', or 'video'."
            )
            rclpy.shutdown()
            sys.exit(1)

        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        topic_text = self.pub_topic if self.pub_topic.startswith('/') else '/' + self.pub_topic
        self.get_logger().info(
            f"ImagePublisherNode started | source={self.data_source} | topic={topic_text} | "
            f"{self.frame_width}x{self.frame_height}@{self.camera_fps}fps | "
            f"timer={self.timer_period:.4f}s | QoS=BEST_EFFORT depth=1"
        )

    def _init_realsense(self):
        if rs is None:
            self.get_logger().error(
                "pyrealsense2 is not installed. Install Intel RealSense SDK first."
            )
            rclpy.shutdown()
            sys.exit(1)

        ctx = rs.context()
        devices = ctx.query_devices()

        if len(devices) == 0:
            self.get_logger().error("No RealSense device found.")
            # 여기서 바로 종료하지 않고 계속 재시도하고 싶으면 return False로 바꿔도 됨
            rclpy.shutdown()
            sys.exit(1)

        self.get_logger().info(f"Found {len(devices)} RealSense device(s).")

        for dev in devices:
            try:
                name = dev.get_info(rs.camera_info.name)
                serial = dev.get_info(rs.camera_info.serial_number)
                self.get_logger().info(f"RealSense device: {name}, serial={serial}")
            except Exception as e:
                self.get_logger().warning(f"Could not read device info: {e}")

        return self._start_realsense_pipeline()

    def _start_realsense_pipeline(self):
        self._stop_realsense_pipeline()

        self.pipeline = rs.pipeline()
        self.rs_config = rs.config()

        if self.realsense_serial:
            self.rs_config.enable_device(self.realsense_serial)
            self.get_logger().info(f"Using RealSense serial: {self.realsense_serial}")

        self.rs_config.enable_stream(
            rs.stream.color,
            self.frame_width,
            self.frame_height,
            rs.format.bgr8,
            self.camera_fps
        )

        try:
            profile = self.pipeline.start(self.rs_config)
            self.realsense_started = True
            self.realsense_fail_count = 0

            stream_profile = profile.get_stream(rs.stream.color).as_video_stream_profile()
            intr = stream_profile.get_intrinsics()

            self.get_logger().info(
                f"RealSense color stream started | "
                f"{intr.width}x{intr.height}@{self.camera_fps} | "
                f"fx={intr.fx:.2f}, fy={intr.fy:.2f}, "
                f"ppx={intr.ppx:.2f}, ppy={intr.ppy:.2f}"
            )

            self._set_realsense_options(profile)
            return True

        except Exception as e:
            self.get_logger().error(f"Failed to start RealSense pipeline: {e}")
            self.realsense_started = False
            self.pipeline = None
            return False

    def _stop_realsense_pipeline(self):
        if self.pipeline is not None:
            try:
                self.pipeline.stop()
                self.get_logger().warning("RealSense pipeline stopped.")
            except Exception:
                pass

        self.pipeline = None
        self.rs_config = None
        self.realsense_started = False

    def _restart_realsense_pipeline(self):
        self.get_logger().error("Restarting RealSense pipeline...")
        self._stop_realsense_pipeline()
        time.sleep(0.2)
        return self._start_realsense_pipeline()

    def _set_realsense_options(self, profile):
        try:
            device = profile.get_device()
            sensors = device.query_sensors()

            # 보통 D435에서 color sensor는 RGB Camera 이름을 가짐
            color_sensor = None
            for sensor in sensors:
                try:
                    sensor_name = sensor.get_info(rs.camera_info.name)
                    if 'RGB' in sensor_name or 'Color' in sensor_name:
                        color_sensor = sensor
                        break
                except Exception:
                    pass

            # 못 찾으면 기존 코드처럼 두 번째 센서 시도
            if color_sensor is None and len(sensors) > 1:
                color_sensor = sensors[1]

            if color_sensor is not None:
                if color_sensor.supports(rs.option.enable_auto_exposure):
                    color_sensor.set_option(rs.option.enable_auto_exposure, 1)
                    self.get_logger().info("RealSense auto exposure enabled.")

                # 필요 시 노출이 너무 흔들리면 나중에 수동 노출로 바꿀 수 있음
                # if color_sensor.supports(rs.option.exposure):
                #     color_sensor.set_option(rs.option.exposure, 200)

        except Exception as e:
            self.get_logger().warning(f"Could not set RealSense sensor options: {e}")

    def _init_video(self):
        self.cap = cv2.VideoCapture(self.video_path)

        if not self.cap.isOpened():
            self.get_logger().error(f'Cannot open video file: {self.video_path}')
            rclpy.shutdown()
            sys.exit(1)

    def _init_image_dir(self):
        if not os.path.isdir(self.img_dir):
            self.get_logger().error(f'Not a directory: {self.img_dir}')
            rclpy.shutdown()
            sys.exit(1)

        self.img_list = sorted(os.listdir(self.img_dir))
        self.img_num = 0

        if len(self.img_list) == 0:
            self.get_logger().error(f'No files in directory: {self.img_dir}')
            rclpy.shutdown()
            sys.exit(1)

    def _publish_frame(self, frame):
        if frame is None:
            return

        image_msg = self.br.cv2_to_imgmsg(frame, encoding='bgr8')
        image_msg.header = Header()
        image_msg.header.stamp = self.get_clock().now().to_msg()
        image_msg.header.frame_id = self.frame_id

        self.publisher.publish(image_msg)

        if self.logger:
            cv2.imshow('Image Publisher', frame)
            cv2.waitKey(1)

    def _read_realsense_frame(self):
        if self.pipeline is None or not self.realsense_started:
            self.get_logger().warning("RealSense pipeline is not running. Trying to restart...")
            self._restart_realsense_pipeline()
            return None

        try:
            # 기존 timeout_ms=1000은 너무 김
            # 15fps 기준 프레임 주기 약 66ms, 그래서 100ms 정도로 제한
            frames = self.pipeline.wait_for_frames(timeout_ms=self.realsense_timeout_ms)
            color_frame = frames.get_color_frame()

            if not color_frame:
                self.realsense_fail_count += 1
                self.get_logger().warning(
                    f'No RealSense color frame | fail_count={self.realsense_fail_count}'
                )
                return None

            self.realsense_fail_count = 0
            frame = np.asanyarray(color_frame.get_data())
            return frame

        except Exception as e:
            self.realsense_fail_count += 1
            self.get_logger().warning(
                f'Failed to read RealSense frame: {e} | fail_count={self.realsense_fail_count}'
            )

            if self.realsense_fail_count >= self.max_realsense_fail_count:
                self.get_logger().error(
                    f"RealSense failed {self.realsense_fail_count} times. Restarting pipeline."
                )
                self.realsense_fail_count = 0
                self._restart_realsense_pipeline()

            return None

    def timer_callback(self):
        if self.data_source == 'realsense':
            frame = self._read_realsense_frame()

            if frame is None:
                return

            self._publish_frame(frame)

        elif self.data_source == 'image':
            if self.img_num >= len(self.img_list):
                self.img_num = 0

            img_file = self.img_list[self.img_num]
            img_path = os.path.join(self.img_dir, img_file)
            img = cv2.imread(img_path)

            if img is None:
                self.get_logger().warning(f'Skipping non-image file: {img_file}')
            else:
                img = cv2.resize(img, (self.frame_width, self.frame_height))
                self._publish_frame(img)

                if self.logger:
                    self.get_logger().info(f'Published image: {img_file}')

            self.img_num += 1

        elif self.data_source == 'video':
            ret, frame = self.cap.read()

            if not ret or frame is None:
                self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                return

            frame = cv2.resize(frame, (self.frame_width, self.frame_height))
            self._publish_frame(frame)

    def destroy_node(self):
        self._stop_realsense_pipeline()

        if self.cap is not None and self.cap.isOpened():
            self.cap.release()

        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ImagePublisherNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nshutdown\n")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
