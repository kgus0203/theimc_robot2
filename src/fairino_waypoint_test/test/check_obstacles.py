"""Read-only scene checks against the isolated mock integration launch."""
import rclpy
from moveit_msgs.srv import GetPlanningScene, GetStateValidity
from moveit_msgs.msg import PlanningSceneComponents
import math
rclpy.init()
node = rclpy.create_node('check_scan_obstacles')

def call(client, request):
    assert client.wait_for_service(timeout_sec=40)
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=15)
    assert future.done()
    return future.result()

try:
    client = node.create_client(GetPlanningScene, '/get_planning_scene')
    request = GetPlanningScene.Request()
    request.components.components = PlanningSceneComponents.WORLD_OBJECT_GEOMETRY | PlanningSceneComponents.OBJECT_COLORS | PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
    import time
    for _ in range(100):
        scene = call(client, request).scene
        objects = {o.id: o for o in scene.world.collision_objects}
        if {'scan_left_wall', 'scan_right_wall', 'scan_base_box'} <= objects.keys():
            break
        time.sleep(0.1)
    for side, sign in [('right', -1), ('left', 1)]:
        obj = objects[f'scan_{side}_wall']
        assert list(obj.primitives[0].dimensions) == [0.02, 1.0, 1.0]
        assert math.isclose(obj.pose.position.x + obj.primitive_poses[0].position.x, sign * 0.51)
        assert math.isclose(obj.pose.position.z + obj.primitive_poses[0].position.z, 0.5)
    obj = objects['scan_base_box']
    assert list(obj.primitives[0].dimensions) == [0.44] * 3
    assert math.isclose(obj.pose.position.z + obj.primitive_poses[0].position.z, -0.141)
    assert {c.id for c in scene.object_colors} >= objects.keys() & {'scan_left_wall', 'scan_right_wall', 'scan_base_box'}
    print('PASS scene geometry: wall faces +/-0.5m, support top z=0.079m, colors', flush=True)
    acm = scene.allowed_collision_matrix
    index = acm.entry_names.index('scan_base_box')
    for link in ('base_link', 'shoulder_link'):
        assert acm.entry_values[index].enabled[acm.entry_names.index(link)]
    for link in ('upperarm_link', 'forearm_link', 'wrist1_link', 'wrist2_link', 'wrist3_link'):
        if link in acm.entry_names:
            assert not acm.entry_values[index].enabled[acm.entry_names.index(link)]
    print('PASS mounting exceptions limited to base and shoulder', flush=True)
    validity = node.create_client(GetStateValidity, '/check_state_validity')
    for name, angles in [('right', [-90, -50, -100, -23, 0, 0]), ('left', [90, -140, 100, -140, 0, 0])]:
        req = GetStateValidity.Request()
        req.group_name = 'fairino5_v6_group'
        req.robot_state.joint_state.name = [f'j{i}' for i in range(1, 7)]
        req.robot_state.joint_state.position = [math.radians(q) for q in angles]
        response = call(validity, req)
        print(name, 'ready valid:', response.valid, [(c.contact_body_1, c.contact_body_2) for c in response.contacts], flush=True)
        assert response.valid, name + ' ready intersects an obstacle'
finally:
    node.destroy_node()
    rclpy.shutdown()
