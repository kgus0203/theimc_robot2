import json
import threading
import traceback
from datetime import date
from typing import Dict, List

import rclpy
from interfaces_pkg.action import StartInspection
from rclpy.action import ActionServer, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String

from plant_inspection_pkg.environment_client import RandomEnvironmentClient
from plant_inspection_pkg.models import (
    Capture,
    FilePayload,
    Inspection,
    Location,
    make_inspection_id,
    utc_offset_timestamp,
)
from plant_inspection_pkg.pending_queue import PendingQueue
from plant_inspection_pkg.realsense_camera import RealSenseD435iCamera
from plant_inspection_pkg.robot_client import RobotArmClient
from plant_inspection_pkg.uploader import UploadError, Uploader


CUCUMBER_DEFAULT_WAYPOINTS: Dict[str, List[float]] = {
    "home": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    "upper": [-90.0, -7.0, -28.0, -6.0, 3.0, 2.0],
    "middle": [-95.0, 13.0, -105.0, 51.0, 7.0, -2.0],
    "lower": [-99.0, 10.0, -135.0, 77.0, 10.0, -5.0],
}

STRAWBERRY_DEFAULT_WAYPOINTS: Dict[str, List[float]] = {
    '''
    여기는 높이X 다각도 촬영 이건 나중에 넣을 거임
    '''
}

# 생육 단계는 정식일로부터 지난 날짜를 기준으로 결정한다.
TRANSPLANT_DATE = date(2026, 8, 14)  # 이건 실제 오이 정식일
# TRANSPLANT_DATE = date(2025, 8, 14) # 60일 이상이면 제일 상단만 찍는 test용


class InspectionManager(Node):
    def __init__(self):
        super().__init__("inspection_manager")
        self.declare_parameter("auto_start", False)
        self.declare_parameter("dry_run", False)
        self.declare_parameter("upload_url", "http://192.168.0.10:8080/api/v1/inspections")
        self.declare_parameter("pending_dir", "/tmp/plant_inspection_pending")
        self.declare_parameter("region", "sj")
        self.declare_parameter("device_number", 1)
        self.declare_parameter("bed_id", 1)
        self.declare_parameter("hole_id", 1)
        self.declare_parameter("waypoints_json", "")
        self.declare_parameter("move_speed", 30)
        self.declare_parameter("settle_seconds", 1.0)
        self.declare_parameter("robot_control_mode", "trajectory")
        self.declare_parameter(
            "trajectory_action",
            "/arm_group_controller/follow_joint_trajectory",
        )
        self.declare_parameter("trajectory_time_sec", 3.0)
        self.declare_parameter("set_angles_service", "/set_angles")
        self.declare_parameter("get_angles_service", "/get_angles")
        self.declare_parameter("get_coords_service", "/get_coords")
        self.declare_parameter("camera_color_width", 1280)
        self.declare_parameter("camera_color_height", 720)
        self.declare_parameter("camera_color_fps", 15)
        self.declare_parameter("camera_depth_width", 640)
        self.declare_parameter("camera_depth_height", 480)
        self.declare_parameter("camera_depth_fps", 15)
        self.declare_parameter("jpeg_quality", 95)
        self.declare_parameter("camera_warmup_frames", 15)
        self.declare_parameter("camera_serial", "234322070133")
        self.declare_parameter("stream_enabled", True)
        self.declare_parameter("preview_width", 320)
        self.declare_parameter("preview_height", 180)
        self.declare_parameter("preview_fps", 8)
        self.declare_parameter("preview_bitrate_kbps", 350)
        self.declare_parameter("mediamtx_rtsp_url", "")
        self.declare_parameter("rtsp_pkt_size", 1400)
        self.declare_parameter("camera_frame_timeout_sec", 3.0)
        self.declare_parameter("camera_failure_threshold", 3)
        self.declare_parameter("preview_frame_timeout_sec", 3.0)
        self.declare_parameter("watchdog_interval_sec", 0.5)
        self.declare_parameter("max_preview_restarts", 5)
        self.declare_parameter("max_camera_restarts", 3)
        self.declare_parameter("restart_window_sec", 60.0)

        upload_url = self.get_parameter("upload_url").value
        pending_dir = self.get_parameter("pending_dir").value
        dry_run = bool(self.get_parameter("dry_run").value)
        mediamtx_rtsp_url = str(
            self.get_parameter("mediamtx_rtsp_url").value
        ).strip()

        self.camera = RealSenseD435iCamera(
            color_width=int(self.get_parameter("camera_color_width").value),
            color_height=int(self.get_parameter("camera_color_height").value),
            color_fps=int(self.get_parameter("camera_color_fps").value),
            depth_width=int(self.get_parameter("camera_depth_width").value),
            depth_height=int(self.get_parameter("camera_depth_height").value),
            depth_fps=int(self.get_parameter("camera_depth_fps").value),
            jpeg_quality=int(self.get_parameter("jpeg_quality").value),
            warmup_frames=int(self.get_parameter("camera_warmup_frames").value),
            serial_number=str(self.get_parameter("camera_serial").value),
            stream_enabled=bool(self.get_parameter("stream_enabled").value),
            preview_width=int(self.get_parameter("preview_width").value),
            preview_height=int(self.get_parameter("preview_height").value),
            preview_fps=int(self.get_parameter("preview_fps").value),
            preview_bitrate_kbps=int(
                self.get_parameter("preview_bitrate_kbps").value
            ),
            rtsp_url=mediamtx_rtsp_url,
            rtsp_pkt_size=int(self.get_parameter("rtsp_pkt_size").value),
            camera_frame_timeout_sec=float(
                self.get_parameter("camera_frame_timeout_sec").value
            ),
            camera_failure_threshold=int(
                self.get_parameter("camera_failure_threshold").value
            ),
            preview_frame_timeout_sec=float(
                self.get_parameter("preview_frame_timeout_sec").value
            ),
            watchdog_interval_sec=float(
                self.get_parameter("watchdog_interval_sec").value
            ),
            max_preview_restarts=int(
                self.get_parameter("max_preview_restarts").value
            ),
            max_camera_restarts=int(
                self.get_parameter("max_camera_restarts").value
            ),
            restart_window_sec=float(
                self.get_parameter("restart_window_sec").value
            ),
            logger=self.get_logger(),
        )
        self.environment_client = RandomEnvironmentClient()
        self.pending_queue = PendingQueue(pending_dir)
        self.uploader = Uploader(upload_url)
        self.robot = RobotArmClient(
            self,
            dry_run=dry_run,
            control_mode=str(self.get_parameter("robot_control_mode").value),
            set_angles_service=self.get_parameter("set_angles_service").value,
            get_angles_service=self.get_parameter("get_angles_service").value,
            get_coords_service=self.get_parameter("get_coords_service").value,
            trajectory_action=str(self.get_parameter("trajectory_action").value),
            trajectory_time_sec=float(self.get_parameter("trajectory_time_sec").value),
        )

        self._busy = False
        self._busy_lock = threading.Lock()
        self._arm_command_callback_group = ReentrantCallbackGroup()
        self.arm_command_result_pub = self.create_publisher(
            String,
            "/arm_command_result",
            10,
        )
        self.camera_health_pub = self.create_publisher(
            String,
            "/camera_manager/health",
            10,
        )
        self.create_timer(1.0, self._publish_camera_health)
        self.arm_command_sub = self.create_subscription(
            String,
            "/arm_command",
            self._handle_arm_command,
            10,
            callback_group=self._arm_command_callback_group,
        )
        self.action_server = ActionServer(
            self,
            StartInspection,
            "/start_inspection",
            execute_callback=self._execute_start_inspection,
            goal_callback=self._handle_start_inspection_goal,
        )

        self._started = False
        if bool(self.get_parameter("auto_start").value):
            self.create_timer(0.5, self._run_once)

        self.get_logger().info("Ready for inspection goals on /start_inspection")

    def _publish_camera_health(self) -> None:
        self.camera_health_pub.publish(
            String(data=json.dumps(self.camera.get_health(), ensure_ascii=False))
        )

    def _handle_start_inspection_goal(self, goal_request):
        with self._busy_lock:
            if self._busy:
                self.get_logger().warning(
                    "Rejecting inspection goal because an arm operation is running."
                )
                return GoalResponse.REJECT
            self._busy = True
        return GoalResponse.ACCEPT

    def _execute_start_inspection(self, goal_handle):
        result = StartInspection.Result()
        goal = goal_handle.request
        location = Location(
            region=goal.region,
            device_number=int(goal.device_number),
            bed_id=int(goal.bed_id),
            hole_id=int(goal.hole_id),
        )

        try:
            self.retry_pending()
            inspection, files = self.perform_inspection(
                location=location,
                feedback_publisher=goal_handle.publish_feedback,
            )
            upload_success, upload_message = self.upload_or_queue(inspection, files)
            goal_handle.succeed()
            result.success = upload_success
            result.inspection_id = inspection.inspection_id
            result.message = upload_message
        except Exception as exc:
            self.get_logger().error(f"Inspection action failed: {exc}")
            self.get_logger().error(traceback.format_exc())
            goal_handle.abort()
            result.success = False
            result.inspection_id = ""
            result.message = str(exc)
        finally:
            with self._busy_lock:
                self._busy = False

        return result

    def _handle_arm_command(self, msg: String) -> None:
        command_id = ""
        command = ""
        try:
            payload = json.loads(msg.data)
            if not isinstance(payload, dict):
                raise ValueError("arm command must be a JSON object")

            command_id = str(payload.get("command_id") or "")
            command = str(payload.get("command") or "").upper()
            params = payload.get("params") or {}
            if not command_id:
                raise ValueError("command_id is required")
            if command != "MOVE_PRESET":
                raise ValueError(f"unsupported arm command: {command}")
            if not isinstance(params, dict):
                raise ValueError("params must be a JSON object")

            target = str(params.get("target") or "").lower()
            waypoints = self._waypoints()
            if target not in waypoints:
                raise ValueError(f"unknown arm preset: {target}")

            with self._busy_lock:
                if self._busy:
                    self.get_logger().warning(
                        f"Rejecting MOVE_PRESET {target}: arm is busy"
                    )
                    self._publish_arm_command_result(
                        command_id,
                        command,
                        "REJECTED",
                        "ARM_BUSY",
                        target,
                    )
                    return
                self._busy = True

            try:
                self.get_logger().info(f"Moving arm to preset: {target}")
                self.robot.move_to_angles(
                    waypoints[target],
                    speed=int(self.get_parameter("move_speed").value),
                    settle_seconds=float(self.get_parameter("settle_seconds").value),
                )
                self._publish_arm_command_result(
                    command_id,
                    command,
                    "SUCCEEDED",
                    f"arm reached preset: {target}",
                    target,
                )
            except Exception as exc:
                self.get_logger().error(f"MOVE_PRESET {target} failed: {exc}")
                self._publish_arm_command_result(
                    command_id,
                    command,
                    "FAILED",
                    str(exc),
                    target,
                )
            finally:
                with self._busy_lock:
                    self._busy = False
        except Exception as exc:
            self.get_logger().warning(f"Invalid /arm_command: {exc}")
            self._publish_arm_command_result(
                command_id,
                command or "MOVE_PRESET",
                "REJECTED",
                str(exc),
            )

    def _publish_arm_command_result(
        self,
        command_id: str,
        command: str,
        status: str,
        message: str,
        target: str = "",
    ) -> None:
        payload = {
            "command_id": command_id,
            "command": command,
            "status": status,
            "message": message,
        }
        if target:
            payload["target"] = target
        self.arm_command_result_pub.publish(
            String(data=json.dumps(payload, ensure_ascii=False))
        )

    def _run_once(self):
        if self._started:
            return
        self._started = True

        try:
            self.retry_pending()
            inspection, files = self.perform_inspection()
            self.upload_or_queue(inspection, files)
        except Exception as exc:
            self.get_logger().error(f"Inspection failed: {exc}")
            self.get_logger().error(traceback.format_exc())
        finally:
            self.get_logger().info("Inspection manager cycle finished.")

    def perform_inspection(self, location=None, feedback_publisher=None):
        if location is None:
            location = Location(
                region=str(self.get_parameter("region").value),
                device_number=int(self.get_parameter("device_number").value),
                bed_id=int(self.get_parameter("bed_id").value),
                hole_id=int(self.get_parameter("hole_id").value),
            )
        timestamp = utc_offset_timestamp()
        inspection_id = make_inspection_id(timestamp, location.bed_id, location.hole_id)
        inspection = Inspection(
            inspection_id=inspection_id,
            timestamp=timestamp,
            location=location,
            environment=self.environment_client.read(location),
            captures=[],
        )

        files: List[FilePayload] = []
        viewpoints = self._viewpoints()
        total_captures = len(viewpoints)
        for index, viewpoint_id in enumerate(viewpoints):
            waypoint = self._waypoints()[viewpoint_id]
            self.get_logger().info(f"Moving to {viewpoint_id}: {waypoint}")
            self._publish_feedback(
                feedback_publisher,
                status="moving",
                viewpoint_id=viewpoint_id,
                completed_captures=index,
                total_captures=total_captures,
            )
            self.robot.move_to_angles(
                waypoint,
                speed=int(self.get_parameter("move_speed").value),
                settle_seconds=float(self.get_parameter("settle_seconds").value),
            )

            self._publish_feedback(
                feedback_publisher,
                status="capturing",
                viewpoint_id=viewpoint_id,
                completed_captures=index,
                total_captures=total_captures,
            )
            arm_pose = self.robot.get_arm_pose(fallback_angles=waypoint)
            camera_pose = self.robot.get_camera_pose()
            rgb_payload, depth_payload, camera_info_payload = self.camera.capture(
                inspection_id=inspection_id,
                viewpoint_id=viewpoint_id,
            )
            files.extend([rgb_payload, depth_payload, camera_info_payload])

            inspection.captures.append(
                Capture(
                    viewpoint_id=viewpoint_id,
                    captured_at=utc_offset_timestamp(),
                    camera_pose=camera_pose,
                    arm_pose=arm_pose,
                    files={
                        "rgb": f"/data/{inspection_id}/{rgb_payload.filename}",
                        "depth": f"/data/{inspection_id}/{depth_payload.filename}",
                        "camera_info": (
                            f"/data/{inspection_id}/{camera_info_payload.filename}"
                        ),
                    },
                    aligned_depth=False,
                )
            )
            self._publish_feedback(
                feedback_publisher,
                status="captured",
                viewpoint_id=viewpoint_id,
                completed_captures=index + 1,
                total_captures=total_captures,
            )

        return inspection, files

    def _publish_feedback(
        self,
        feedback_publisher,
        status: str,
        viewpoint_id: str,
        completed_captures: int,
        total_captures: int,
    ) -> None:
        if feedback_publisher is None:
            return
        feedback = StartInspection.Feedback()
        feedback.status = status
        feedback.current_viewpoint = viewpoint_id
        feedback.completed_captures = completed_captures
        feedback.total_captures = total_captures
        feedback_publisher(feedback)

    def upload_or_queue(self, inspection: Inspection, files: List[FilePayload]):
        try:
            response = self.uploader.upload(inspection, files)
            self.get_logger().info(
                f"Uploaded {inspection.inspection_id}: {json.dumps(response)}"
            )
            return True, "inspection completed and uploaded"
        except UploadError as exc:
            self.pending_queue.save(inspection, files)
            message = f"inspection completed but upload failed; queued locally: {exc}"
            self.get_logger().warning(f"{message} ({inspection.inspection_id})")
            return False, message

    def retry_pending(self) -> None:
        queued = self.pending_queue.list_inspections()
        if not queued:
            return

        self.get_logger().info(f"Retrying {len(queued)} pending inspection(s).")
        for item in queued:
            try:
                inspection, files = self.pending_queue.load(item)
                self.uploader.upload(inspection, files)
                self.pending_queue.delete(item)
                self.get_logger().info(f"Pending upload complete: {item}")
            except UploadError as exc:
                self.get_logger().warning(f"Pending upload still failing for {item}: {exc}")
                return

    def _viewpoints(self) -> List[str]:
        growth_days = (date.today() - TRANSPLANT_DATE).days
        if growth_days < 0:
            raise RuntimeError(
                f"Inspection date is before transplant date: {TRANSPLANT_DATE.isoformat()}"
            )

        # 표의 low/middle/high를 기존 waypoint 이름인
        # lower/middle/upper에 대응시킨다.
        if growth_days <= 20:
            viewpoints = ["lower"]
        elif growth_days <= 40:
            viewpoints = ["lower", "middle"]
        elif growth_days <= 60:
            viewpoints = ["middle", "upper"]
        else:
            viewpoints = ["upper"]

        waypoints = self._waypoints()
        missing = [item for item in viewpoints if item not in waypoints]
        if missing:
            raise ValueError(f"No waypoint configured for viewpoint(s): {missing}")

        self.get_logger().info(
            f"Growth day {growth_days} from {TRANSPLANT_DATE.isoformat()}: "
            f"viewpoints={viewpoints}"
        )
        return viewpoints

    def _waypoints(self) -> Dict[str, List[float]]:
        raw = str(self.get_parameter("waypoints_json").value).strip()
        if not raw:
            return CUCUMBER_DEFAULT_WAYPOINTS
        parsed = json.loads(raw)
        return {key: [float(value) for value in values] for key, values in parsed.items()}


def main(args=None):
    rclpy.init(args=args)
    node = InspectionManager()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.camera.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
