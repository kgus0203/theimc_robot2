"""Coordinate Fairino motion and synchronous captures at stopped waypoints."""
import math
import re
import threading

import rclpy
from fairino_waypoint_test.action import ScanSide
from fairino_waypoint_test.srv import CaptureWaypoint
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup

from plant_inspection_pkg.models import (
    ArmPose, CameraPose, Capture, Orientation, Position, utc_offset_timestamp,
)


def tool_metadata(request):
    """Use measured wrist pose as a camera-pose approximation (no extrinsics)."""
    p = request.tool_pose.pose.position
    q = request.tool_pose.pose.orientation
    roll = math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x**2 + q.y**2))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x))))
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y**2 + q.z**2))
    arm = ArmPose(list(request.joints_deg), [
        p.x * 1000, p.y * 1000, p.z * 1000,
        math.degrees(roll), math.degrees(pitch), math.degrees(yaw),
    ])
    camera = CameraPose(request.tool_pose.header.frame_id,
                        Position(p.x, p.y, p.z), Orientation(q.x, q.y, q.z, q.w))
    return arm, camera


class SideScan:
    def __init__(self, node, camera):
        self.node = node
        self.camera = camera
        self._lock = threading.Lock()
        self._active = None
        group = ReentrantCallbackGroup()
        self.client = ActionClient(node, ScanSide, '/scan_side', callback_group=group)
        self.service = node.create_service(
            CaptureWaypoint, '~/capture_waypoint', self._capture, callback_group=group)

    @staticmethod
    def _wait(future):
        # The node's executor services callbacks; do not spin it recursively.
        # Keep the inspection busy until ScanSide actually terminates.
        done = threading.Event()
        future.add_done_callback(lambda _: done.set())
        while not done.wait(0.1):
            if not rclpy.ok():
                raise RuntimeError('ROS shutdown while waiting for scan')
        return future.result()

    def run(self, side, inspection, files, feedback_publisher):
        if side not in ('left', 'right'):
            raise ValueError('side must be left or right')
        if not self.client.wait_for_server(timeout_sec=5.0):
            raise RuntimeError('/scan_side unavailable; start fairino_waypoint_test first')
        with self._lock:
            self._active = (side, inspection, files, feedback_publisher)
        try:
            goal = ScanSide.Goal()
            goal.side = side
            goal.inspection_id = inspection.inspection_id
            # Private names must be resolved in the inspection node, not in
            # the remote motion node. Include namespace and service remappings.
            goal.capture_service = self.node.resolve_service_name(self.service.srv_name)
            handle = self._wait(self.client.send_goal_async(
                goal, feedback_callback=self._feedback))
            if handle is None or not handle.accepted:
                raise RuntimeError('/scan_side rejected the inspection')
            wrapped = self._wait(handle.get_result_async())
            if wrapped is None or wrapped.status != 4 or not wrapped.result.success:
                message = wrapped.result.message if wrapped else 'no result'
                raise RuntimeError(f'/scan_side failed: {message}')
            if not inspection.captures or len(inspection.captures) != wrapped.result.completed_waypoints:
                raise RuntimeError('Scan completed without all expected captures')
        finally:
            with self._lock:
                self._active = None

    def _feedback(self, message):
        with self._lock:
            if self._active is None:
                return
            _, inspection, _, publisher = self._active
            update = message.feedback
            self.node._publish_feedback(
                publisher, status=update.stage, viewpoint_id=update.side,
                completed_captures=len(inspection.captures),
                total_captures=update.total_waypoints)

    def _capture(self, request, response):
        with self._lock:
            try:
                if self._active is None:
                    raise RuntimeError('No active side inspection')
                side, inspection, files, publisher = self._active
                if request.inspection_id != inspection.inspection_id or request.side != side:
                    raise RuntimeError('Capture does not match the active inspection')
                if request.waypoint_index != len(inspection.captures) + 1:
                    raise RuntimeError('Unexpected or duplicate waypoint capture')
                if not 0 < request.waypoint_index <= request.total_waypoints:
                    raise RuntimeError('Invalid waypoint count')
                viewpoint = side + '_' + re.sub(r'[^A-Za-z0-9_-]+', '_', request.viewpoint)
                self.node._publish_feedback(
                    publisher, status='capturing', viewpoint_id=viewpoint,
                    completed_captures=len(inspection.captures),
                    total_captures=request.total_waypoints)
                arm, camera = tool_metadata(request)
                rgb, depth, info = self.camera.capture(inspection.inspection_id, viewpoint)
                files.extend([rgb, depth, info])
                inspection.captures.append(Capture(
                    viewpoint_id=viewpoint, captured_at=utc_offset_timestamp(),
                    camera_pose=camera, arm_pose=arm,
                    files={key: f'/data/{inspection.inspection_id}/{item.filename}'
                           for key, item in [('rgb', rgb), ('depth', depth), ('camera_info', info)]},
                    aligned_depth=False,
                ))
                self.node._publish_feedback(
                    publisher, status='captured', viewpoint_id=viewpoint,
                    completed_captures=len(inspection.captures),
                    total_captures=request.total_waypoints)
                response.success = True
                response.message = 'capture complete'
            except Exception as exc:
                response.success = False
                response.message = str(exc)
        return response
