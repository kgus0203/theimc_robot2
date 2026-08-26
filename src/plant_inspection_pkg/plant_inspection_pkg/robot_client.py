import math
import time
from typing import List

import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from sensor_msgs.msg import JointState

from plant_inspection_pkg.models import ArmPose, CameraPose, Orientation, Position


MYCOBOT_280_JOINT_NAMES = [
    "joint2_to_joint1",
    "joint3_to_joint2",
    "joint4_to_joint3",
    "joint5_to_joint4",
    "joint6_to_joint5",
    "joint6output_to_joint6",
]


class RobotArmClient:
    def __init__(
        self,
        node,
        dry_run: bool,
        control_mode: str,
        set_angles_service: str,
        get_angles_service: str,
        get_coords_service: str,
        trajectory_action: str,
        trajectory_time_sec: float,
    ):
        self.node = node
        self.dry_run = dry_run
        self.control_mode = control_mode
        self.trajectory_action = trajectory_action
        self.trajectory_time_sec = trajectory_time_sec
        self._last_angles = [0.0] * 6
        self._last_coords = [320.0, 0.0, 260.0, 0.0, 0.0, 0.0]
        self._callback_group = ReentrantCallbackGroup()
        self._joint_state_sub = node.create_subscription(
            JointState,
            "/joint_states",
            self._joint_state_callback,
            10,
            callback_group=self._callback_group,
        )

        if dry_run:
            self.SetAngles = None
            self.GetAngles = None
            self.GetCoords = None
            self.set_angles_client = None
            self.get_angles_client = None
            self.get_coords_client = None
            self.FollowJointTrajectory = None
            self.trajectory_client = None
            return

        if control_mode == "trajectory":
            from control_msgs.action import FollowJointTrajectory

            self.SetAngles = None
            self.GetAngles = None
            self.GetCoords = None
            self.set_angles_client = None
            self.get_angles_client = None
            self.get_coords_client = None
            self.FollowJointTrajectory = FollowJointTrajectory
            self.trajectory_client = ActionClient(
                node,
                FollowJointTrajectory,
                trajectory_action,
                callback_group=self._callback_group,
            )
            return

        if control_mode != "service":
            raise ValueError("robot_control_mode must be 'trajectory' or 'service'")

        from mycobot_interfaces.srv import GetAngles, GetCoords, SetAngles

        self.SetAngles = SetAngles
        self.GetAngles = GetAngles
        self.GetCoords = GetCoords
        self.set_angles_client = node.create_client(SetAngles, set_angles_service)
        self.get_angles_client = node.create_client(GetAngles, get_angles_service)
        self.get_coords_client = node.create_client(GetCoords, get_coords_service)
        self.FollowJointTrajectory = None
        self.trajectory_client = None

    def move_to_angles(self, angles: List[float], speed: int, settle_seconds: float) -> None:
        self._last_angles = [float(value) for value in angles]
        if self.dry_run:
            self._last_coords = self._coords_from_angles(self._last_angles)
            time.sleep(settle_seconds)
            return

        if self.control_mode == "trajectory":
            self._send_trajectory(self._last_angles)
            time.sleep(settle_seconds)
            return

        self._wait_for_services()
        request = self.SetAngles.Request()
        for index, value in enumerate(self._last_angles, start=1):
            setattr(request, f"joint_{index}", float(value))
        request.speed = int(speed)
        future = self.set_angles_client.call_async(request)
        self._wait_for_future(future, timeout_sec=10.0)
        result = future.result()
        if result is None or not result.flag:
            raise RuntimeError("mycobot set_angles service failed")
        time.sleep(settle_seconds)

    def get_arm_pose(self, fallback_angles: List[float]) -> ArmPose:
        if self.dry_run or self.control_mode == "trajectory":
            angles = self._last_angles
            coords = self._last_coords
        else:
            angles = self._read_angles(fallback_angles)
            coords = self._read_coords()
        self._last_angles = angles
        self._last_coords = coords
        return ArmPose(joints_deg=angles, coords=coords)

    def get_camera_pose(self) -> CameraPose:
        coords = self._last_coords
        return CameraPose(
            frame_id="robot_base",
            position=Position(
                x=round(coords[0] / 1000.0, 4),
                y=round(coords[1] / 1000.0, 4),
                z=round(coords[2] / 1000.0, 4),
            ),
            orientation=Orientation(x=0.0, y=0.0, z=0.0, w=1.0),
        )

    def _wait_for_services(self) -> None:
        missing_services = []
        for client in (
            self.set_angles_client,
            self.get_angles_client,
            self.get_coords_client,
        ):
            if not client.wait_for_service(timeout_sec=5.0):
                missing_services.append(client.srv_name)

        if missing_services:
            raise RuntimeError(
                "mycobot ROS2 service node is not running. "
                f"Missing service(s): {', '.join(missing_services)}. "
                "For myCobot 280, start the driver first, for example: "
                "ros2 run mycobot_280 listen_real_service"
            )

    def _send_trajectory(self, angles_deg: List[float]) -> None:
        if not self.trajectory_client.wait_for_server(timeout_sec=10.0):
            raise RuntimeError(
                f"Trajectory action server is not available: {self.trajectory_action}. "
                "Start real_robot.launch.py first."
            )

        from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

        goal = self.FollowJointTrajectory.Goal()
        goal.trajectory = JointTrajectory()
        goal.trajectory.joint_names = MYCOBOT_280_JOINT_NAMES

        point = JointTrajectoryPoint()
        point.positions = [math.radians(value) for value in angles_deg]
        point.time_from_start.sec = int(self.trajectory_time_sec)
        point.time_from_start.nanosec = int(
            (self.trajectory_time_sec % 1.0) * 1_000_000_000
        )
        goal.trajectory.points.append(point)
        goal.goal_time_tolerance.sec = 5

        future = self.trajectory_client.send_goal_async(goal)
        self._wait_for_future(future, timeout_sec=10.0)
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError("FollowJointTrajectory goal rejected")

        result_future = goal_handle.get_result_async()
        self._wait_for_future(result_future, timeout_sec=30.0)
        wrapped_result = result_future.result()
        if wrapped_result is None:
            raise RuntimeError("FollowJointTrajectory result timed out")

        result = wrapped_result.result
        if result.error_code != self.FollowJointTrajectory.Result.SUCCESSFUL:
            raise RuntimeError(
                "FollowJointTrajectory failed: "
                f"error_code={result.error_code}, error_string={result.error_string}"
            )

    def _read_angles(self, fallback_angles: List[float]) -> List[float]:
        request = self.GetAngles.Request()
        future = self.get_angles_client.call_async(request)
        self._wait_for_future(future, timeout_sec=5.0)
        result = future.result()
        if result is None:
            return [float(value) for value in fallback_angles]
        return [float(getattr(result, f"joint_{index}")) for index in range(1, 7)]

    def _read_coords(self) -> List[float]:
        request = self.GetCoords.Request()
        future = self.get_coords_client.call_async(request)
        self._wait_for_future(future, timeout_sec=5.0)
        result = future.result()
        if result is None:
            return self._last_coords
        return [float(getattr(result, axis)) for axis in ("x", "y", "z", "rx", "ry", "rz")]

    def _joint_state_callback(self, msg: JointState) -> None:
        by_name = dict(zip(msg.name, msg.position))
        if all(name in by_name for name in MYCOBOT_280_JOINT_NAMES):
            self._last_angles = [
                round(math.degrees(by_name[name]), 3) for name in MYCOBOT_280_JOINT_NAMES
            ]

    @staticmethod
    def _wait_for_future(future, timeout_sec: float) -> None:
        deadline = time.monotonic() + timeout_sec
        while not future.done():
            if time.monotonic() > deadline:
                return
            time.sleep(0.01)

    @staticmethod
    def _coords_from_angles(angles: List[float]) -> List[float]:
        return [
            320.0,
            round(angles[0] * 1.5, 2),
            round(270.0 + angles[1] * 0.8, 2),
            angles[3],
            angles[4],
            angles[5],
        ]
