"""Integration check. Run with a mock demo in an isolated ROS domain only."""
import time
import re
from pathlib import Path
import rclpy
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from visualization_msgs.msg import MarkerArray
from rclpy.qos import QoSProfile, DurabilityPolicy
from fairino_waypoint_test.action import ScanSide

rclpy.init()
node = rclpy.create_node('scan_action_check')
client = ActionClient(node, ScanSide, '/scan_side')
paths = []
subscription = node.create_subscription(MarkerArray, '/waypoint_test/planned_path',
    lambda msg: paths.append(msg), QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))

def wait(future, timeout=240):
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
    assert future.done(), 'Action timed out'
    return future.result()

def send(side, feedback=None):
    goal = ScanSide.Goal()
    goal.side = side
    return wait(client.send_goal_async(goal, feedback_callback=feedback))

try:
    assert client.wait_for_server(timeout_sec=40), 'No scan server'
    assert not send('invalid').accepted
    source = (Path(__file__).resolve().parents[1] / 'src/recorded_poses.hpp').read_text()
    # Anchor to an active initializer so commented-out recordings are excluded.
    all_names = re.findall(r'^\s*\{"(\d+ [^"]+)", \{', source, re.MULTILINE)
    for side in ('left', 'right'):
        recorded_names = [name for name in all_names if name.split()[1] == side]
        expected = len(recorded_names)
        assert expected > 0
        updates = []
        times = []
        def receive(msg):
            updates.append(msg.feedback)
            times.append(time.monotonic())
        handle = send(side, receive)
        assert handle.accepted
        assert not send('left' if side == 'right' else 'right').accepted
        result = wait(handle.get_result_async())
        assert result.status == GoalStatus.STATUS_SUCCEEDED, result
        assert result.result.success and result.result.completed_waypoints == expected, result
        reached = [f for f in updates if f.stage.startswith('reached:')]
        assert len(reached) == expected, reached
        assert all(f.side == side and f.total_waypoints == expected for f in updates)
        assert [f.completed_waypoints for f in reached] == list(range(1, expected + 1))
        holds = [i for i, f in enumerate(updates) if f.stage.startswith('holding 3 seconds:')]
        assert len(holds) == expected
        for i in holds:
            end = times[i + 1] if i + 1 < len(times) else time.monotonic()
            assert end - times[i] >= 2.9, (i, end - times[i])
        assert paths and len(paths[-1].markers) == expected
        assert all(len(m.points) >= 2 for m in paths[-1].markers)
        assert [f.stage.split()[1] for f in reached] == [name.split()[0] for name in recorded_names]
        print(f'PASS recorded order, {expected} 3-second holds, {expected} planned tool paths', flush=True)
        print('PASS', side, f'{expected} targets, feedback, success, busy rejection', flush=True)
    handle = send('right')
    assert handle.accepted
    cancel = wait(handle.cancel_goal_async())
    assert cancel.goals_canceling
    result = wait(handle.get_result_async())
    assert result.status == GoalStatus.STATUS_CANCELED and not result.result.success, result
    print('PASS cancellation', flush=True)
finally:
    node.destroy_node()
    rclpy.shutdown()
