import json
import traceback
from typing import Dict, List

import rclpy
from interfaces_pkg.action import StartInspection
from rclpy.action import ActionServer, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

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


DEFAULT_WAYPOINTS: Dict[str, List[float]] = {
    "upper": [0.0, -35.0, -45.0, 0.0, 65.0, 0.0],
    "middle": [0.0, -20.0, -55.0, 0.0, 75.0, 0.0],
    "lower": [0.0, -5.0, -65.0, 0.0, 85.0, 0.0],
}


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
        self.declare_parameter("viewpoints", "upper,middle,lower")
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
        self.declare_parameter("camera_width", 640)
        self.declare_parameter("camera_height", 480)
        self.declare_parameter("camera_fps", 30)
        self.declare_parameter("jpeg_quality", 95)
        self.declare_parameter("camera_warmup_frames", 15)
        self.declare_parameter("camera_serial", "234322070133")

        upload_url = self.get_parameter("upload_url").value
        pending_dir = self.get_parameter("pending_dir").value
        dry_run = bool(self.get_parameter("dry_run").value)

        self.camera = RealSenseD435iCamera(
            width=int(self.get_parameter("camera_width").value),
            height=int(self.get_parameter("camera_height").value),
            fps=int(self.get_parameter("camera_fps").value),
            jpeg_quality=int(self.get_parameter("jpeg_quality").value),
            warmup_frames=int(self.get_parameter("camera_warmup_frames").value),
            serial_number=str(self.get_parameter("camera_serial").value),
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

    def _handle_start_inspection_goal(self, goal_request):
        if self._busy:
            self.get_logger().warning("Rejecting inspection goal because one is running.")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _execute_start_inspection(self, goal_handle):
        self._busy = True
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
            self._busy = False

        return result

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
            rgb_payload, depth_payload = self.camera.capture(
                inspection_id=inspection_id,
                viewpoint_id=viewpoint_id,
            )
            files.extend([rgb_payload, depth_payload])

            inspection.captures.append(
                Capture(
                    viewpoint_id=viewpoint_id,
                    captured_at=utc_offset_timestamp(),
                    camera_pose=camera_pose,
                    arm_pose=arm_pose,
                    files={
                        "rgb": f"/data/{inspection_id}/{rgb_payload.filename}",
                        "depth": f"/data/{inspection_id}/{depth_payload.filename}",
                    },
                    aligned_depth=True,
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
        raw = str(self.get_parameter("viewpoints").value)
        viewpoints = [item.strip() for item in raw.split(",") if item.strip()]
        waypoints = self._waypoints()
        missing = [item for item in viewpoints if item not in waypoints]
        if missing:
            raise ValueError(f"No waypoint configured for viewpoint(s): {missing}")
        return viewpoints

    def _waypoints(self) -> Dict[str, List[float]]:
        raw = str(self.get_parameter("waypoints_json").value).strip()
        if not raw:
            return DEFAULT_WAYPOINTS
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
