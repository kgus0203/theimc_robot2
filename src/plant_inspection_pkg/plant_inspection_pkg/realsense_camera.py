from typing import Optional

from plant_inspection_pkg.models import FilePayload


class RealSenseD435iCamera:
    def __init__(
        self,
        width: int = 640,
        height: int = 480,
        fps: int = 30,
        jpeg_quality: int = 95,
        warmup_frames: int = 15,
        serial_number: str = "",
    ):
        try:
            import cv2
            import numpy as np
            import pyrealsense2 as rs
        except ImportError as exc:
            raise RuntimeError(
                "RealSense capture requires pyrealsense2, opencv-python, and numpy."
            ) from exc

        self.cv2 = cv2
        self.np = np
        self.rs = rs
        self.width = width
        self.height = height
        self.fps = fps
        self.jpeg_quality = jpeg_quality
        self.warmup_frames = warmup_frames
        self.serial_number = serial_number.strip()
        self.pipeline: Optional[object] = None
        self.align: Optional[object] = None

    def capture(self, inspection_id: str, viewpoint_id: str):
        self._ensure_started()
        frames = None
        for _ in range(max(1, self.warmup_frames)):
            frames = self.pipeline.wait_for_frames()

        aligned = self.align.process(frames)
        color_frame = aligned.get_color_frame()
        depth_frame = aligned.get_depth_frame()
        if not color_frame or not depth_frame:
            raise RuntimeError("RealSense did not provide both color and depth frames.")

        color_bgr = self.np.asanyarray(color_frame.get_data())
        depth_mm = self.np.asanyarray(depth_frame.get_data())

        ok, rgb_buffer = self.cv2.imencode(
            ".jpg",
            color_bgr,
            [self.cv2.IMWRITE_JPEG_QUALITY, int(self.jpeg_quality)],
        )
        if not ok:
            raise RuntimeError("Failed to encode RealSense color frame as JPEG.")

        ok, depth_buffer = self.cv2.imencode(".png", depth_mm)
        if not ok:
            raise RuntimeError("Failed to encode RealSense depth frame as PNG.")

        prefix = f"{inspection_id}_{viewpoint_id}"
        return (
            FilePayload(
                field_name=f"{viewpoint_id}_rgb",
                filename=f"{prefix}_rgb.jpg",
                content_type="image/jpeg",
                data=rgb_buffer.tobytes(),
            ),
            FilePayload(
                field_name=f"{viewpoint_id}_depth",
                filename=f"{prefix}_depth.png",
                content_type="image/png",
                data=depth_buffer.tobytes(),
            ),
        )

    def close(self) -> None:
        if self.pipeline is not None:
            self.pipeline.stop()
            self.pipeline = None

    def _ensure_started(self) -> None:
        if self.pipeline is not None:
            return

        config = self.rs.config()
        if self.serial_number:
            config.enable_device(self.serial_number)
        config.enable_stream(
            self.rs.stream.color,
            self.width,
            self.height,
            self.rs.format.bgr8,
            self.fps,
        )
        config.enable_stream(
            self.rs.stream.depth,
            self.width,
            self.height,
            self.rs.format.z16,
            self.fps,
        )
        self.pipeline = self.rs.pipeline()
        self.pipeline.start(config)
        self.align = self.rs.align(self.rs.stream.color)
