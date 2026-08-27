import json
import os
import queue
import shutil
import subprocess
import threading
import time
from collections import deque
from typing import Optional

from plant_inspection_pkg.models import FilePayload


DEVICE_TYPE = os.getenv("DEVICE_TYPE", "mobile")
REGION_NAME = os.getenv("REGION_NAME", "sj")
DEVICE_NUMBER = os.getenv("DEVICE_NUMBER", "01")
DEVICE_PATH = f"{DEVICE_TYPE}/{REGION_NAME}/{DEVICE_NUMBER}"

MEDIAMTX_HOST = os.getenv("MEDIAMTX_HOST", "192.168.0.17")
MEDIAMTX_RTSP_URL = os.getenv(
    "MEDIAMTX_RTSP_URL",
    f"rtsp://{MEDIAMTX_HOST}:8554/{DEVICE_PATH}",
)
MEDIAMTX_WEBRTC_URL = os.getenv(
    "MEDIAMTX_WEBRTC_URL",
    f"http://{MEDIAMTX_HOST}:8889/{DEVICE_PATH}",
)
RTSP_PKT_SIZE = int(os.getenv("RTSP_PKT_SIZE", "1400"))


class RealSenseD435iCamera:
    """Own one RealSense pipeline shared by preview streaming and captures."""

    def __init__(
        self,
        color_width: int = 1280,
        color_height: int = 720,
        color_fps: int = 15,
        depth_width: int = 640,
        depth_height: int = 480,
        depth_fps: int = 15,
        jpeg_quality: int = 95,
        warmup_frames: int = 15,
        serial_number: str = "",
        stream_enabled: bool = True,
        preview_width: int = 320,
        preview_height: int = 180,
        preview_fps: int = 8,
        preview_bitrate_kbps: int = 350,
        rtsp_url: str = MEDIAMTX_RTSP_URL,
        rtsp_pkt_size: int = RTSP_PKT_SIZE,
        camera_frame_timeout_sec: float = 3.0,
        camera_failure_threshold: int = 3,
        preview_frame_timeout_sec: float = 3.0,
        watchdog_interval_sec: float = 0.5,
        max_preview_restarts: int = 5,
        max_camera_restarts: int = 3,
        restart_window_sec: float = 60.0,
        logger=None,
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
        self.color_width = color_width
        self.color_height = color_height
        self.color_fps = color_fps
        self.depth_width = depth_width
        self.depth_height = depth_height
        self.depth_fps = depth_fps
        self.jpeg_quality = jpeg_quality
        self.warmup_frames = warmup_frames
        self.serial_number = serial_number.strip()
        self.stream_enabled = stream_enabled
        self.preview_width = preview_width
        self.preview_height = preview_height
        self.preview_fps = max(1, preview_fps)
        self.preview_bitrate_kbps = max(100, preview_bitrate_kbps)
        self.rtsp_url = rtsp_url or MEDIAMTX_RTSP_URL
        self.rtsp_pkt_size = rtsp_pkt_size
        self.camera_frame_timeout_sec = max(0.5, camera_frame_timeout_sec)
        self.camera_failure_threshold = max(1, camera_failure_threshold)
        self.preview_frame_timeout_sec = max(0.5, preview_frame_timeout_sec)
        self.watchdog_interval_sec = max(0.1, watchdog_interval_sec)
        self.max_preview_restarts = max(1, max_preview_restarts)
        self.max_camera_restarts = max(1, max_camera_restarts)
        self.restart_window_sec = max(10.0, restart_window_sec)
        self.logger = logger

        self.pipeline: Optional[object] = None
        self._camera_info = None
        self._stop_event = threading.Event()
        self._pipeline_lock = threading.Lock()
        self._recovery_lock = threading.Lock()
        self._ffmpeg_lock = threading.Lock()
        self._latest_condition = threading.Condition()
        self._latest_bundle = None
        self._frame_error: Optional[Exception] = None
        self._preview_queue = queue.Queue(maxsize=1)
        self._capture_queue = queue.Queue(maxsize=4)
        self._frame_thread = None
        self._preview_thread = None
        self._save_thread = None
        self._watchdog_thread = None
        self._ffmpeg_process = None
        self._ffmpeg_started_at = 0.0
        self._ffmpeg_retry_at = 0.0
        self._ffmpeg_missing_reported = False

        now = time.monotonic()
        self.camera_state = "RECOVERING"
        self.preview_state = "STARTING" if stream_enabled else "OFF"
        self.recovery_state = "RECOVERING_CAMERA"
        self.last_rgb_frame_time = 0.0
        self.last_depth_frame_time = 0.0
        self.preview_last_frame_time = 0.0
        self.preview_publisher_alive = False
        self.preview_restart_count = 0
        self.camera_restart_count = 0
        self.last_error = ""
        self._started_at = now
        self._camera_stale_count = 0
        self._next_camera_recovery_at = 0.0
        self._pipeline_generation = 0
        self._preview_restart_times = deque()
        self._camera_restart_times = deque()

        self._start()

    def capture(self, inspection_id: str, viewpoint_id: str):
        if not self._recovery_lock.acquire(blocking=False):
            raise RuntimeError("Capture rejected while camera recovery is in progress.")
        try:
            health = self.get_health()
            if health["camera_state"] != "OK":
                raise RuntimeError(
                    f"Capture rejected while camera is {health['camera_state']}: "
                    f"{health['last_error']}"
                )
            frame_age = health["last_frame_age_sec"]
            if frame_age is None or frame_age > self.camera_frame_timeout_sec:
                raise RuntimeError("Capture rejected because the camera frame is stale.")
            with self._latest_condition:
                bundle = self._latest_bundle
            if bundle is None:
                raise RuntimeError("Capture rejected because no frame is available.")
        finally:
            self._recovery_lock.release()

        request = {
            "inspection_id": inspection_id,
            "viewpoint_id": viewpoint_id,
            "color": bundle[0],
            "depth": bundle[1],
            "camera_info": bundle[3],
            "done": threading.Event(),
            "result": None,
            "error": None,
        }
        try:
            self._capture_queue.put(request, timeout=2.0)
        except queue.Full as exc:
            raise RuntimeError("Capture queue is full.") from exc

        if not request["done"].wait(timeout=15.0):
            raise RuntimeError("Timed out while encoding capture images.")
        if request["error"] is not None:
            raise RuntimeError(str(request["error"])) from request["error"]
        return request["result"]

    def get_health(self):
        now = time.monotonic()
        last_frame_time = min(
            value for value in (
                self.last_rgb_frame_time,
                self.last_depth_frame_time,
            ) if value > 0.0
        ) if self.last_rgb_frame_time > 0.0 and self.last_depth_frame_time > 0.0 else 0.0
        frame_age = now - last_frame_time if last_frame_time > 0.0 else None
        return {
            "camera_state": self.camera_state,
            "preview_state": self.preview_state,
            "camera_restart_count": self.camera_restart_count,
            "preview_restart_count": self.preview_restart_count,
            "last_frame_age_sec": round(frame_age, 3) if frame_age is not None else None,
            "capture_queue_size": self._capture_queue.qsize(),
            "preview_publisher_alive": self.preview_publisher_alive,
            "last_error": self.last_error,
        }

    def close(self) -> None:
        self._stop_event.set()
        with self._latest_condition:
            self._latest_condition.notify_all()

        self._put_latest(self._preview_queue, None)
        try:
            self._capture_queue.put_nowait(None)
        except queue.Full:
            pass

        self._stop_pipeline()

        threads = (
            self._frame_thread,
            self._preview_thread,
            self._save_thread,
            self._watchdog_thread,
        )
        for thread in threads:
            if thread is not None and thread.is_alive():
                thread.join(timeout=2.0)

        self._stop_ffmpeg()
        self.pipeline = None
        self._camera_info = None

    def _start(self) -> None:
        self._start_pipeline()
        self.recovery_state = "NORMAL"

        self._frame_thread = threading.Thread(
            target=self._frame_loop,
            name="realsense-frame-loop",
            daemon=True,
        )
        self._save_thread = threading.Thread(
            target=self._save_loop,
            name="capture-save-worker",
            daemon=True,
        )
        self._watchdog_thread = threading.Thread(
            target=self._watchdog_loop,
            name="camera-watchdog",
            daemon=True,
        )
        self._frame_thread.start()
        self._save_thread.start()
        self._watchdog_thread.start()

        if self.stream_enabled:
            self._start_preview_thread()

        self._log(
            "info",
            f"[CAMERA] frame loop started: serial={self.serial_number or 'auto'}, "
            f"RGB={self.color_width}x{self.color_height}@{self.color_fps}, "
            f"Depth={self.depth_width}x{self.depth_height}@{self.depth_fps}",
        )
        if self.stream_enabled:
            self._log(
                "info",
                f"[PREVIEW] MediaMTX target: {self.rtsp_url} "
                f"(WebRTC: {MEDIAMTX_WEBRTC_URL})",
            )

    def _start_pipeline(self) -> None:
        config = self.rs.config()
        if self.serial_number:
            config.enable_device(self.serial_number)
        config.enable_stream(
            self.rs.stream.color,
            self.color_width,
            self.color_height,
            self.rs.format.bgr8,
            self.color_fps,
        )
        config.enable_stream(
            self.rs.stream.depth,
            self.depth_width,
            self.depth_height,
            self.rs.format.z16,
            self.depth_fps,
        )

        pipeline = self.rs.pipeline()
        try:
            profile = pipeline.start(config)
        except Exception:
            try:
                pipeline.stop()
            except Exception:
                pass
            raise

        camera_info = self._build_camera_info(profile)
        with self._pipeline_lock:
            self.pipeline = pipeline
            self._camera_info = camera_info
            self._pipeline_generation += 1

    def _build_camera_info(self, profile):
        color_profile = profile.get_stream(
            self.rs.stream.color
        ).as_video_stream_profile()
        depth_profile = profile.get_stream(
            self.rs.stream.depth
        ).as_video_stream_profile()
        color_intrinsics = color_profile.get_intrinsics()
        depth_intrinsics = depth_profile.get_intrinsics()
        depth_to_color = depth_profile.get_extrinsics_to(color_profile)
        color_to_depth = color_profile.get_extrinsics_to(depth_profile)
        depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()

        return {
            "schema_version": 1,
            "device_serial": self.serial_number,
            "frames_aligned": False,
            "color": {
                "encoding": "bgr8",
                "intrinsics": self._intrinsics_to_dict(color_intrinsics),
            },
            "depth": {
                "encoding": "16UC1",
                "unit": "raw_uint16",
                "scale_meters_per_unit": float(depth_scale),
                "intrinsics": self._intrinsics_to_dict(depth_intrinsics),
            },
            "extrinsics": {
                "convention": (
                    "target_point_m = rotation(row-major) * source_point_m "
                    "+ translation_meters"
                ),
                "depth_to_color": self._extrinsics_to_dict(depth_to_color),
                "color_to_depth": self._extrinsics_to_dict(color_to_depth),
            },
        }

    @staticmethod
    def _intrinsics_to_dict(intrinsics):
        return {
            "width": int(intrinsics.width),
            "height": int(intrinsics.height),
            "fx": float(intrinsics.fx),
            "fy": float(intrinsics.fy),
            "ppx": float(intrinsics.ppx),
            "ppy": float(intrinsics.ppy),
            "distortion_model": str(intrinsics.model),
            "coefficients": [float(value) for value in intrinsics.coeffs],
        }

    @staticmethod
    def _extrinsics_to_dict(extrinsics):
        return {
            "rotation": [float(value) for value in extrinsics.rotation],
            "translation_meters": [
                float(value) for value in extrinsics.translation
            ],
        }

    def _stop_pipeline(self) -> None:
        with self._pipeline_lock:
            pipeline = self.pipeline
            self.pipeline = None
            self._camera_info = None
        if pipeline is not None:
            try:
                pipeline.stop()
            except Exception:
                pass

    def _start_preview_thread(self) -> None:
        if self._preview_thread is not None and self._preview_thread.is_alive():
            return
        self._preview_thread = threading.Thread(
            target=self._preview_loop,
            name="mediamtx-preview-worker",
            daemon=True,
        )
        self._preview_thread.start()

    def _frame_loop(self) -> None:
        warmed_frames = 0
        observed_generation = -1
        preview_period = 1.0 / self.preview_fps
        next_preview_at = 0.0

        while not self._stop_event.is_set():
            try:
                with self._pipeline_lock:
                    pipeline = self.pipeline
                    camera_info = self._camera_info
                    generation = self._pipeline_generation
                if pipeline is None or camera_info is None:
                    time.sleep(0.05)
                    continue
                if generation != observed_generation:
                    warmed_frames = 0
                    observed_generation = generation

                frames = pipeline.wait_for_frames(timeout_ms=1000)
                color_frame = frames.get_color_frame()
                depth_frame = frames.get_depth_frame()
                if not color_frame or not depth_frame:
                    continue

                color = self.np.asanyarray(color_frame.get_data()).copy()
                depth = self.np.asanyarray(depth_frame.get_data()).copy()
                warmed_frames += 1
                if warmed_frames < max(1, self.warmup_frames):
                    continue

                now = time.monotonic()
                with self._latest_condition:
                    self._latest_bundle = (color, depth, now, camera_info)
                    self._frame_error = None
                    self._latest_condition.notify_all()
                self.last_rgb_frame_time = now
                self.last_depth_frame_time = now
                if self.recovery_state == "NORMAL":
                    self.camera_state = "OK"
                self._camera_stale_count = 0

                if self.stream_enabled and now >= next_preview_at:
                    self._put_latest(self._preview_queue, color)
                    next_preview_at = now + preview_period
            except Exception as exc:
                if self._stop_event.is_set():
                    break
                with self._latest_condition:
                    self._frame_error = exc
                    self._latest_condition.notify_all()
                self.last_error = str(exc)
                time.sleep(0.2)

    def _watchdog_loop(self) -> None:
        while not self._stop_event.wait(self.watchdog_interval_sec):
            if self.recovery_state != "NORMAL":
                continue

            now = time.monotonic()
            last_camera_frame = min(
                self.last_rgb_frame_time,
                self.last_depth_frame_time,
            )
            if last_camera_frame <= 0.0:
                frame_age = now - self._started_at
            else:
                frame_age = now - last_camera_frame

            if frame_age > self.camera_frame_timeout_sec:
                self._camera_stale_count += 1
                if self._camera_stale_count == 1:
                    self.camera_state = "STALE"
                    self.last_error = f"camera frame timeout: {frame_age:.1f} sec"
                    self._log("warning", f"[CAMERA] {self.last_error}")
                threshold_reached = (
                    self._camera_stale_count >= self.camera_failure_threshold
                )
                retry_ready = now >= self._next_camera_recovery_at
                if threshold_reached and retry_ready:
                    self._log("error", "[WATCHDOG] camera frame stale")
                    self._recover_camera()
                continue

            self._camera_stale_count = 0
            self.camera_state = "OK"
            if self.stream_enabled:
                self._watch_preview(now)

    def _watch_preview(self, now: float) -> None:
        thread_alive = (
            self._preview_thread is not None and self._preview_thread.is_alive()
        )
        process, return_code = self._ffmpeg_status()

        if not thread_alive:
            self.last_error = "preview worker stopped"
            self._recover_preview(self.last_error)
            return
        if process is not None and return_code is not None:
            self.last_error = f"ffmpeg process exited code={return_code}"
            self._log("warning", f"[PREVIEW] {self.last_error}")
            self._recover_preview(self.last_error)
            return

        if process is None:
            self.preview_publisher_alive = False
            return

        self.preview_publisher_alive = True
        reference_time = self.preview_last_frame_time or self._ffmpeg_started_at
        if now - reference_time > self.preview_frame_timeout_sec:
            self.last_error = (
                f"preview frame timeout: {now - reference_time:.1f} sec"
            )
            self._log("warning", f"[PREVIEW] {self.last_error}")
            self._recover_preview(self.last_error)

    def _recover_preview(self, reason: str) -> None:
        if not self._recovery_lock.acquire(blocking=False):
            return
        try:
            self.recovery_state = "RECOVERING_PREVIEW"
            self.preview_state = "RECOVERING"
            now = time.monotonic()
            recent = self._record_restart(
                self._preview_restart_times,
                now,
            )
            self.preview_restart_count += 1
            delay = min(10.0, 2.0 * (2 ** min(recent - 1, 3)))
            if recent >= self.max_preview_restarts:
                window_age = now - self._preview_restart_times[0]
                window_remaining = self.restart_window_sec - window_age
                delay = max(delay, window_remaining + 0.1)
                self.preview_state = "ERROR"

            self._log(
                "warning",
                f"[WATCHDOG] restarting preview publisher: {reason}; "
                f"retry in {delay:.1f}s",
            )
            self._stop_ffmpeg()
            self._drain_queue(self._preview_queue)
            self._ffmpeg_retry_at = now + delay
            self._start_preview_thread()
        finally:
            self.recovery_state = "NORMAL"
            self._recovery_lock.release()

    def _recover_camera(self) -> None:
        if not self._recovery_lock.acquire(blocking=False):
            return
        try:
            self.recovery_state = "RECOVERING_CAMERA"
            self.camera_state = "RECOVERING"
            self.preview_state = "RECOVERING" if self.stream_enabled else "OFF"
            now = time.monotonic()
            recent = self._record_restart(self._camera_restart_times, now)
            self.camera_restart_count += 1
            if recent > self.max_camera_restarts:
                self._request_process_restart(
                    "camera restart storm exceeded configured limit"
                )
                return

            backoffs = (2.0, 5.0, 10.0)
            delay = backoffs[min(recent - 1, len(backoffs) - 1)]
            self._log(
                "warning",
                f"[WATCHDOG] restarting RealSense pipeline; "
                f"attempt={recent}, wait={delay:.1f}s",
            )
            self._stop_ffmpeg()
            self._drain_queue(self._preview_queue)
            self._stop_pipeline()
            with self._latest_condition:
                self._latest_bundle = None
                self._latest_condition.notify_all()
            self.last_rgb_frame_time = 0.0
            self.last_depth_frame_time = 0.0
            self.preview_last_frame_time = 0.0
            self.preview_publisher_alive = False

            if self._stop_event.wait(delay):
                return

            recovery_started_at = time.monotonic()
            try:
                self._start_pipeline()
                if not self._wait_for_recovery_frame(
                    recovery_started_at,
                    timeout_sec=self.camera_frame_timeout_sec + 3.0,
                ):
                    raise RuntimeError("no RGB/Depth frame after pipeline restart")
            except Exception as exc:
                self.camera_state = "ERROR"
                self.last_error = f"camera recovery failed: {exc}"
                self._log("error", f"[CAMERA] {self.last_error}")
                self._next_camera_recovery_at = time.monotonic() + delay
                if recent >= self.max_camera_restarts:
                    self._request_process_restart(self.last_error)
                return

            self.camera_state = "OK"
            self.last_error = ""
            self._camera_stale_count = 0
            self._next_camera_recovery_at = 0.0
            self._ffmpeg_retry_at = time.monotonic() + 0.5
            self._log("info", "[CAMERA] recovery successful")
        finally:
            self.recovery_state = "NORMAL"
            self._recovery_lock.release()

    def _wait_for_recovery_frame(self, started_at: float, timeout_sec: float) -> bool:
        deadline = time.monotonic() + timeout_sec
        with self._latest_condition:
            while not self._stop_event.is_set():
                rgb_recovered = self.last_rgb_frame_time >= started_at
                depth_recovered = self.last_depth_frame_time >= started_at
                if rgb_recovered and depth_recovered:
                    return True
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    return False
                self._latest_condition.wait(timeout=remaining)
        return False

    def _record_restart(self, history, now: float) -> int:
        while history and now - history[0] > self.restart_window_sec:
            history.popleft()
        history.append(now)
        return len(history)

    def _request_process_restart(self, reason: str) -> None:
        self.camera_state = "ERROR"
        self.recovery_state = "FAILED"
        self.last_error = reason
        self._log(
            "fatal",
            f"[WATCHDOG] recovery failed {self.camera_restart_count} times, "
            f"exiting process: {reason}",
        )
        os._exit(1)

    def _wait_for_latest_bundle(self, timeout_sec: float):
        deadline = time.monotonic() + timeout_sec
        with self._latest_condition:
            while self._latest_bundle is None and not self._stop_event.is_set():
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    detail = f": {self._frame_error}" if self._frame_error else ""
                    raise RuntimeError(f"Timed out waiting for RealSense frames{detail}")
                self._latest_condition.wait(timeout=remaining)
            if self._latest_bundle is None:
                raise RuntimeError("RealSense camera is closed.")
            return self._latest_bundle

    def _save_loop(self) -> None:
        while not self._stop_event.is_set():
            try:
                request = self._capture_queue.get(timeout=0.5)
            except queue.Empty:
                continue
            if request is None:
                break

            try:
                ok, rgb_buffer = self.cv2.imencode(
                    ".jpg",
                    request["color"],
                    [self.cv2.IMWRITE_JPEG_QUALITY, int(self.jpeg_quality)],
                )
                if not ok:
                    raise RuntimeError("Failed to encode RealSense color frame as JPEG.")

                ok, depth_buffer = self.cv2.imencode(".png", request["depth"])
                if not ok:
                    raise RuntimeError("Failed to encode RealSense depth frame as PNG.")

                camera_info_buffer = json.dumps(
                    request["camera_info"],
                    ensure_ascii=False,
                    indent=2,
                ).encode("utf-8")

                inspection_id = request["inspection_id"]
                viewpoint_id = request["viewpoint_id"]
                prefix = f"{inspection_id}_{viewpoint_id}"
                request["result"] = (
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
                    FilePayload(
                        field_name=f"{viewpoint_id}_camera_info",
                        filename=f"{prefix}_camera_info.json",
                        content_type="application/json",
                        data=camera_info_buffer,
                    ),
                )
            except Exception as exc:
                request["error"] = exc
            finally:
                request["done"].set()

    def _preview_loop(self) -> None:
        while not self._stop_event.is_set():
            try:
                frame = self._preview_queue.get(timeout=0.5)
            except queue.Empty:
                continue
            if frame is None:
                break

            if self._ffmpeg_process is None and not self._start_ffmpeg():
                continue

            resized = self.cv2.resize(
                frame,
                (self.preview_width, self.preview_height),
                interpolation=self.cv2.INTER_AREA,
            )
            try:
                with self._ffmpeg_lock:
                    process = self._ffmpeg_process
                    if process is None or process.poll() is not None:
                        raise BrokenPipeError("ffmpeg is not running")
                    process.stdin.write(resized.tobytes())
                self.preview_last_frame_time = time.monotonic()
                self.preview_publisher_alive = True
                if self.preview_state != "STREAMING":
                    self.preview_state = "STREAMING"
                    self._log("info", "[PREVIEW] restart successful")
            except (BrokenPipeError, OSError, ValueError) as exc:
                self.last_error = f"MediaMTX stream disconnected: {exc}"
                self._recover_preview(self.last_error)

    def _start_ffmpeg(self) -> bool:
        if time.monotonic() < self._ffmpeg_retry_at:
            return False
        ffmpeg = shutil.which("ffmpeg")
        if ffmpeg is None:
            self.preview_state = "ERROR"
            self.preview_publisher_alive = False
            if not self._ffmpeg_missing_reported:
                self._log(
                    "error",
                    "ffmpeg is not installed; capture remains available "
                    "but streaming is disabled.",
                )
                self._ffmpeg_missing_reported = True
            self._ffmpeg_retry_at = time.monotonic() + 30.0
            return False

        command = [
            ffmpeg,
            "-hide_banner",
            "-loglevel", "warning",
            "-f", "rawvideo",
            "-pixel_format", "bgr24",
            "-video_size", f"{self.preview_width}x{self.preview_height}",
            "-framerate", str(self.preview_fps),
            "-i", "pipe:0",
            "-an",
            "-c:v", "libx264",
            "-preset", "ultrafast",
            "-tune", "zerolatency",
            "-pix_fmt", "yuv420p",
            "-g", str(self.preview_fps * 2),
            "-b:v", f"{self.preview_bitrate_kbps}k",
            "-f", "rtsp",
            "-rtsp_transport", "tcp",
            "-pkt_size", str(self.rtsp_pkt_size),
            self.rtsp_url,
        ]
        try:
            process = subprocess.Popen(
                command,
                stdin=subprocess.PIPE,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                bufsize=0,
            )
            with self._ffmpeg_lock:
                self._ffmpeg_process = process
                self._ffmpeg_started_at = time.monotonic()
            self.preview_state = "STARTING"
            self.preview_publisher_alive = True
            self._log("info", f"[PREVIEW] publishing H.264 to {self.rtsp_url}")
            return True
        except OSError as exc:
            self.last_error = f"could not start ffmpeg: {exc}"
            self._log("error", f"[PREVIEW] {self.last_error}")
            with self._ffmpeg_lock:
                self._ffmpeg_process = None
            self.preview_state = "ERROR"
            self.preview_publisher_alive = False
            self._ffmpeg_retry_at = time.monotonic() + 5.0
            return False

    def _stop_ffmpeg(self) -> None:
        with self._ffmpeg_lock:
            process = self._ffmpeg_process
            self._ffmpeg_process = None
            self._ffmpeg_started_at = 0.0
        self.preview_publisher_alive = False
        if process is None:
            return
        try:
            if process.stdin is not None:
                process.stdin.close()
        except OSError:
            pass
        try:
            process.wait(timeout=1.0)
        except subprocess.TimeoutExpired:
            process.terminate()
            try:
                process.wait(timeout=1.0)
            except subprocess.TimeoutExpired:
                process.kill()

    def _ffmpeg_status(self):
        with self._ffmpeg_lock:
            process = self._ffmpeg_process
            return_code = process.poll() if process is not None else None
        return process, return_code

    @staticmethod
    def _drain_queue(target_queue) -> None:
        while True:
            try:
                target_queue.get_nowait()
            except queue.Empty:
                return

    @staticmethod
    def _put_latest(target_queue, item) -> None:
        try:
            target_queue.put_nowait(item)
            return
        except queue.Full:
            pass
        try:
            target_queue.get_nowait()
        except queue.Empty:
            pass
        try:
            target_queue.put_nowait(item)
        except queue.Full:
            pass

    def _log(self, level: str, message: str) -> None:
        if self.logger is None:
            return
        callback = getattr(self.logger, level, None)
        if callback is not None:
            callback(message)
