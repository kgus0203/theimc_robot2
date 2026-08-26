# Copyright (C) 2023 Miguel Ángel González Santamarta
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
# See the GNU General Public License for more details.

from typing import List, Dict

import rclpy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSReliabilityPolicy
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.lifecycle import LifecycleState

from cv_bridge import CvBridge

import torch
from torch import cuda

from ultralytics import YOLO
from ultralytics.engine.results import Results
from ultralytics.engine.results import Boxes
from ultralytics.engine.results import Masks
from ultralytics.engine.results import Keypoints

from sensor_msgs.msg import Image

from interfaces_pkg.msg import Point2D
from interfaces_pkg.msg import BoundingBox2D
from interfaces_pkg.msg import Mask
from interfaces_pkg.msg import KeyPoint2D
from interfaces_pkg.msg import KeyPoint2DArray
from interfaces_pkg.msg import Detection
from interfaces_pkg.msg import DetectionArray

from std_srvs.srv import SetBool
from std_msgs.msg import Bool


class Yolov26Node(LifecycleNode):

    def __init__(self, **kwargs) -> None:
        super().__init__("yolov26_node", **kwargs)

        # ============================================================
        # Parameters
        # ============================================================

        # YOLO 모델
        self.declare_parameter(
            "model",
            "/home/jeff/theimc_robot/src/"
            "camera_perception_pkg/test/best_260803.pt"
        )

        # 추론 장치
        # "cpu"
        # "cuda:0"
        self.declare_parameter("device", "cuda:0")

        # confidence threshold
        self.declare_parameter("threshold", 0.5)

        # perception 활성화 여부
        self.declare_parameter("enable", False)

        # perception enable topic
        self.declare_parameter(
            "perception_enable_topic",
            "/rail_perception_enable"
        )

        # image QoS
        self.declare_parameter(
            "image_reliability",
            QoSReliabilityPolicy.BEST_EFFORT
        )

        # 실제 모델 객체
        self.yolo = None

        # image subscriber
        self._sub = None

        self.get_logger().info("Yolov26Node created")


    # ================================================================
    # Lifecycle - Configure
    # ================================================================

    def on_configure(
        self,
        state: LifecycleState
    ) -> TransitionCallbackReturn:

        self.get_logger().info(
            f"Configuring {self.get_name()}"
        )

        # ------------------------------------------------------------
        # Parameters
        # ------------------------------------------------------------

        self.model = self.get_parameter(
            "model"
        ).get_parameter_value().string_value

        self.device = self.get_parameter(
            "device"
        ).get_parameter_value().string_value

        self.threshold = self.get_parameter(
            "threshold"
        ).get_parameter_value().double_value

        self.enable = self.get_parameter(
            "enable"
        ).get_parameter_value().bool_value

        self.perception_enable_topic = self.get_parameter(
            "perception_enable_topic"
        ).get_parameter_value().string_value

        self.reliability = self.get_parameter(
            "image_reliability"
        ).get_parameter_value().integer_value

        # ------------------------------------------------------------
        # QoS
        # ------------------------------------------------------------

        self.image_qos_profile = QoSProfile(
            reliability=self.reliability,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1
        )

        # ------------------------------------------------------------
        # Publisher
        # ------------------------------------------------------------

        self._pub = self.create_lifecycle_publisher(
            DetectionArray,
            "detections",
            10
        )

        # ------------------------------------------------------------
        # Service
        # ------------------------------------------------------------

        self._srv = self.create_service(
            SetBool,
            "enable",
            self.enable_cb
        )

        # ------------------------------------------------------------
        # Enable topic subscriber
        # ------------------------------------------------------------

        self._enable_sub = self.create_subscription(
            Bool,
            self.perception_enable_topic,
            self.perception_enable_cb,
            10
        )

        # ------------------------------------------------------------
        # CV Bridge
        # ------------------------------------------------------------

        self.cv_bridge = CvBridge()

        return TransitionCallbackReturn.SUCCESS


    # ================================================================
    # Enable Service
    # ================================================================

    def enable_cb(self, request, response):

        self.enable = request.data

        response.success = True

        return response


    # ================================================================
    # Enable Topic
    # ================================================================

    def perception_enable_cb(self, msg: Bool):

        prev = self.enable

        self.enable = bool(msg.data)

        if prev != self.enable:

            if self.enable:

                self.get_logger().info(
                    "[YOLOV26] perception ENABLED by topic"
                )

            else:

                self.get_logger().info(
                    "[YOLOV26] perception DISABLED by topic"
                )


    # ================================================================
    # Lifecycle - Activate
    # ================================================================

    def on_activate(
        self,
        state: LifecycleState
    ) -> TransitionCallbackReturn:

        self.get_logger().info(
            f"Activating {self.get_name()}"
        )

        try:

            # ========================================================
            # PyTorch / CUDA / cuDNN 상태 확인
            # ========================================================

            self.get_logger().info(
                "[YOLOV26] "
                f"torch={torch.__version__}, "
                f"cuda={torch.version.cuda}, "
                f"cuda_available={torch.cuda.is_available()}"
            )

            if torch.backends.cudnn.is_available():

                self.get_logger().info(
                    "[YOLOV26] "
                    f"cuDNN={torch.backends.cudnn.version()}"
                )

            else:

                self.get_logger().warn(
                    "[YOLOV26] cuDNN unavailable"
                )

            # ========================================================
            # Device 검사
            # ========================================================

            if "cuda" in self.device:

                if not torch.cuda.is_available():

                    self.get_logger().error(
                        "[YOLOV26] "
                        "CUDA device requested but CUDA is unavailable"
                    )

                    return TransitionCallbackReturn.FAILURE

                self.get_logger().info(
                    "[YOLOV26] "
                    f"CUDA device: "
                    f"{torch.cuda.get_device_name(0)}"
                )

            # ========================================================
            # YOLO 모델 로딩
            # ========================================================

            self.get_logger().info(
                f"[YOLOV26] Loading model: {self.model}"
            )

            self.yolo = YOLO(self.model)

            self.yolo.fuse()

            self.get_logger().info(
                "[YOLOV26] Model loaded"
            )

            # ========================================================
            # GPU 이동 + CUDA/cuDNN Warm-up
            # ========================================================

            if "cuda" in self.device:

                self.get_logger().info(
                    f"[YOLOV26] Moving model to {self.device}"
                )

                self.yolo.to(self.device)

                self.get_logger().info(
                    "[YOLOV26] CUDA warm-up start"
                )

                # 실제 카메라 입력과 같은 형태를 흉내 내는
                # dummy tensor
                dummy = torch.zeros(
                    (1, 3, 640, 640),
                    dtype=torch.float32,
                    device=self.device
                )

                # 실제 YOLO predict 경로를 한 번 실행해서
                # CUDA / cuDNN context를 미리 초기화
                self.yolo.predict(
                    source=dummy,
                    device=self.device,
                    verbose=False,
                    stream=False
                )

                # GPU 연산 완료 대기
                torch.cuda.synchronize()

                self.get_logger().info(
                    "[YOLOV26] CUDA warm-up OK"
                )

                del dummy

            # ========================================================
            # Image subscription
            #
            # 중요:
            # CUDA/cuDNN warm-up이 성공한 뒤 subscriber 생성
            # ========================================================

            self._sub = self.create_subscription(
                Image,
                "image_raw",
                self.image_cb,
                self.image_qos_profile
            )

            self.get_logger().info(
                "[YOLOV26] image subscriber created"
            )

        except FileNotFoundError:

            self.get_logger().error(
                f"Error: Model file '{self.model}' not found!"
            )

            return TransitionCallbackReturn.FAILURE

        except Exception as e:

            self.get_logger().error(
                "[YOLOV26] "
                f"Model initialization / warm-up failed: {str(e)}"
            )

            return TransitionCallbackReturn.FAILURE

        super().on_activate(state)

        return TransitionCallbackReturn.SUCCESS


    # ================================================================
    # Lifecycle - Deactivate
    # ================================================================

    def on_deactivate(
        self,
        state: LifecycleState
    ) -> TransitionCallbackReturn:

        self.get_logger().info(
            f"Deactivating {self.get_name()}"
        )

        # image subscription 먼저 제거
        if self._sub is not None:

            self.destroy_subscription(self._sub)

            self._sub = None

        # 모델 제거
        if self.yolo is not None:

            del self.yolo

            self.yolo = None

        # CUDA cache 정리
        if "cuda" in self.device:

            self.get_logger().info(
                "Clearing CUDA cache"
            )

            cuda.empty_cache()

        super().on_deactivate(state)

        return TransitionCallbackReturn.SUCCESS


    # ================================================================
    # Lifecycle - Cleanup
    # ================================================================

    def on_cleanup(
        self,
        state: LifecycleState
    ) -> TransitionCallbackReturn:

        self.get_logger().info(
            f"Cleaning up {self.get_name()}"
        )

        self.destroy_publisher(
            self._pub
        )

        del self.image_qos_profile

        return TransitionCallbackReturn.SUCCESS


    # ================================================================
    # Parse Hypothesis
    # ================================================================

    def parse_hypothesis(
        self,
        results: Results
    ) -> List[Dict]:

        hypothesis_list = []

        box_data: Boxes

        for box_data in results.boxes:

            hypothesis = {
                "class_id": int(box_data.cls),
                "class_name": self.yolo.names[
                    int(box_data.cls)
                ],
                "score": float(box_data.conf)
            }

            hypothesis_list.append(
                hypothesis
            )

        return hypothesis_list


    # ================================================================
    # Parse Bounding Boxes
    # ================================================================

    def parse_boxes(
        self,
        results: Results
    ) -> List[BoundingBox2D]:

        boxes_list = []

        box_data: Boxes

        for box_data in results.boxes:

            msg = BoundingBox2D()

            box = box_data.xywh[0]

            msg.center.position.x = float(
                box[0]
            )

            msg.center.position.y = float(
                box[1]
            )

            msg.size.x = float(
                box[2]
            )

            msg.size.y = float(
                box[3]
            )

            boxes_list.append(
                msg
            )

        return boxes_list


    # ================================================================
    # Parse Masks
    # ================================================================

    def parse_masks(
        self,
        results: Results
    ) -> List[Mask]:

        masks_list = []

        def create_point2d(
            x: float,
            y: float
        ) -> Point2D:

            p = Point2D()

            p.x = x
            p.y = y

            return p

        mask: Masks

        for mask in results.masks:

            msg = Mask()

            msg.data = [
                create_point2d(
                    float(ele[0]),
                    float(ele[1])
                )
                for ele in mask.xy[0].tolist()
            ]

            msg.height = results.orig_img.shape[0]
            msg.width = results.orig_img.shape[1]

            masks_list.append(
                msg
            )

        return masks_list


    # ================================================================
    # Parse Keypoints
    # ================================================================

    def parse_keypoints(
        self,
        results: Results
    ) -> List[KeyPoint2DArray]:

        keypoints_list = []

        points: Keypoints

        for points in results.keypoints:

            msg_array = KeyPoint2DArray()

            if points.conf is None:

                continue

            for kp_id, (p, conf) in enumerate(
                zip(
                    points.xy[0],
                    points.conf[0]
                )
            ):

                if conf >= self.threshold:

                    msg = KeyPoint2D()

                    msg.id = kp_id + 1

                    msg.point.x = float(
                        p[0]
                    )

                    msg.point.y = float(
                        p[1]
                    )

                    msg.score = float(
                        conf
                    )

                    msg_array.data.append(
                        msg
                    )

            keypoints_list.append(
                msg_array
            )

        return keypoints_list


    # ================================================================
    # Image Callback
    # ================================================================

    def image_cb(
        self,
        msg: Image
    ) -> None:

        if not self.enable:

            return

        try:

            # --------------------------------------------------------
            # ROS Image -> OpenCV
            # --------------------------------------------------------

            cv_image = self.cv_bridge.imgmsg_to_cv2(
                msg
            )

            # --------------------------------------------------------
            # YOLO inference
            # --------------------------------------------------------

            results = self.yolo.predict(
                source=cv_image,
                verbose=False,
                stream=False,
                conf=self.threshold,
                device=self.device
            )

            # --------------------------------------------------------
            # GPU -> CPU
            # --------------------------------------------------------

            results: Results = results[0].cpu()

            # --------------------------------------------------------
            # DetectionArray
            # --------------------------------------------------------

            detections_msg = DetectionArray()

            detections_msg.header = msg.header

            has_boxes = (
                results.boxes is not None
                and len(results.boxes) > 0
            )

            has_masks = (
                results.masks is not None
                and len(results.masks) > 0
            )

            has_keypoints = (
                results.keypoints is not None
                and len(results.keypoints) > 0
            )

            # --------------------------------------------------------
            # Detection 없음
            # --------------------------------------------------------

            if not has_boxes:

                self._pub.publish(
                    detections_msg
                )

                return

            # --------------------------------------------------------
            # Parse YOLO result
            # --------------------------------------------------------

            hypothesis = self.parse_hypothesis(
                results
            )

            boxes = self.parse_boxes(
                results
            )

            masks = []

            if has_masks:

                masks = self.parse_masks(
                    results
                )

            keypoints = []

            if has_keypoints:

                keypoints = self.parse_keypoints(
                    results
                )

            # --------------------------------------------------------
            # ROS Detection Message
            # --------------------------------------------------------

            for i in range(
                len(boxes)
            ):

                aux_msg = Detection()

                aux_msg.class_id = (
                    hypothesis[i]["class_id"]
                )

                aux_msg.class_name = (
                    hypothesis[i]["class_name"]
                )

                aux_msg.score = (
                    hypothesis[i]["score"]
                )

                aux_msg.bbox = boxes[i]

                if i < len(masks):

                    aux_msg.mask = masks[i]

                if i < len(keypoints):

                    aux_msg.keypoints = (
                        keypoints[i]
                    )

                detections_msg.detections.append(
                    aux_msg
                )

            # --------------------------------------------------------
            # Publish
            # --------------------------------------------------------

            self._pub.publish(
                detections_msg
            )

            del results
            del cv_image

        except Exception as e:

            self.get_logger().error(
                "[YOLOV26] "
                f"image_cb error: {str(e)}"
            )


# ====================================================================
# Main
# ====================================================================

def main():

    rclpy.init()

    node = Yolov26Node()

    node.trigger_configure()

    node.trigger_activate()

    rclpy.spin(
        node
    )

    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()