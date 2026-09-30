"""Exercise the real ROS action/service handshake without moving hardware."""
import threading
from types import SimpleNamespace

import pytest
import rclpy
from fairino_waypoint_test.action import ScanSide
from fairino_waypoint_test.srv import CaptureWaypoint
from rclpy.action import ActionServer, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from plant_inspection_pkg.side_scan import SideScan as InspectionSideScan


@pytest.mark.parametrize('side,mode', [
    ('left', 'ok'), ('right', 'ok'), ('left', 'camera_failure'),
    ('left', 'reject'), ('left', 'wrong_inspection'), ('right', 'duplicate'),
])
def test_handshake(side, mode):
    rclpy.init()
    node = Node('side_scan_test', namespace='/inspection')
    motion_node = Node('motion_server', namespace='/arm')
    updates = []
    node._publish_feedback = lambda *args, **kwargs: updates.append(kwargs)
    captured = []

    def capture(inspection_id, viewpoint):
        if mode == 'camera_failure':
            raise RuntimeError('camera unavailable')
        captured.append(viewpoint)
        return tuple(SimpleNamespace(filename=viewpoint + ext) for ext in ('.jpg', '.png', '.json'))

    scanner = InspectionSideScan(node, SimpleNamespace(capture=capture))
    executed = []

    def execute(handle):
        goal = handle.request
        executed.append(goal.side)
        client = motion_node.create_client(CaptureWaypoint, goal.capture_service,
                                    callback_group=ReentrantCallbackGroup())
        result = ScanSide.Result()
        try:
            if not client.wait_for_service(timeout_sec=2):
                result.message = 'Inspection capture service unavailable: ' + goal.capture_service
                handle.abort()
                return result
            for index in (1, 2):
                request = CaptureWaypoint.Request()
                request.inspection_id = 'wrong' if mode == 'wrong_inspection' else goal.inspection_id
                request.side = goal.side
                request.viewpoint = f'{index:02d} {goal.side} upper'
                request.waypoint_index = 1 if mode == 'duplicate' else index
                request.total_waypoints = 2
                request.joints_deg = [1., 2., 3., 4., 5., 6.]
                request.tool_pose.header.frame_id = 'base_link'
                request.tool_pose.pose.orientation.w = 1.
                request.tool_pose.pose.position.x = 0.25
                response = scanner._wait(client.call_async(request))
                if not response.success:
                    result.message = response.message
                    handle.abort()
                    return result
                result.completed_waypoints = index
            result.success = True
            handle.succeed()
            return result
        finally:
            motion_node.destroy_client(client)

    server = ActionServer(motion_node, ScanSide, '/scan_side', execute,
                          callback_group=ReentrantCallbackGroup(),
                          goal_callback=lambda _: GoalResponse.REJECT if mode == 'reject'
                          else GoalResponse.ACCEPT)
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    executor.add_node(motion_node)
    spinner = threading.Thread(target=executor.spin)
    spinner.start()
    inspection = SimpleNamespace(inspection_id='test_001', captures=[])
    files = []
    try:
        if mode == 'ok':
            scanner.run(side, inspection, files, None)
            assert executed == [side]
            assert captured == [f'{side}_01_{side}_upper', f'{side}_02_{side}_upper']
            assert len(files) == 6
            assert len(inspection.captures) == 2
            assert inspection.captures[0].arm_pose.coords == [250., 0., 0., 0., 0., 0.]
            assert updates[-1]['completed_captures'] == 2
        else:
            with pytest.raises(RuntimeError):
                scanner.run(side, inspection, files, None)
            assert len(captured) == (1 if mode == 'duplicate' else 0)
        assert scanner._active is None
    finally:
        executor.shutdown()
        spinner.join()
        server.destroy()
        motion_node.destroy_node()
        node.destroy_node()
        rclpy.shutdown()
