"""MQTT to ROS 2 bridge used by the web robot backend.

The MQTT callbacks run in paho's thread.  They never call rclpy directly;
commands are copied to a queue and consumed by a ROS timer instead.
"""

import json
import math
import os
import queue
import threading
import time
from typing import Any, Dict, Optional

import paho.mqtt.client as mqtt
import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Path
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import BatteryState, JointState
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

from interfaces_pkg.action import RailApproach


def _env_int(name: str, default: int) -> int:
    try:
        return int(os.getenv(name, str(default)))
    except ValueError:
        return default


MQTT_BROKER = os.getenv("MQTT_BROKER", "203.251.85.204")
MQTT_PORT = _env_int("MQTT_PORT", 1883)
MQTT_USERNAME = os.getenv("MQTT_USERNAME", "")
MQTT_PASSWORD = os.getenv("MQTT_PASSWORD", "")

DEVICE_TYPE = os.getenv("DEVICE_TYPE", "mobile")
REGION_NAME = os.getenv("REGION_NAME", "sj")
DEVICE_NUMBER = os.getenv("DEVICE_NUMBER", "01")
DEVICE_PATH = f"{DEVICE_TYPE}/{REGION_NAME}/{DEVICE_NUMBER}"
ROBOT_ID = os.getenv("ROBOT_ID", DEVICE_PATH.replace("/", "_"))

TOPIC_STATUS = f"{DEVICE_PATH}/status"
TOPIC_EVENT = f"{DEVICE_PATH}/event"
TOPIC_HEALTH = f"{DEVICE_PATH}/health"
TOPIC_AMR_CMD = f"{DEVICE_PATH}/amr/cmd"
TOPIC_AMR_STATUS = f"{DEVICE_PATH}/amr/status"
TOPIC_AMR_POSE = f"{DEVICE_PATH}/amr/pose"
TOPIC_AMR_PATH = f"{DEVICE_PATH}/amr/path"
TOPIC_AMR_CMD_RESULT = f"{DEVICE_PATH}/amr/cmd_result"
TOPIC_ARM_CMD = f"{DEVICE_PATH}/arm/cmd"
TOPIC_ARM_STATUS = f"{DEVICE_PATH}/arm/status"
TOPIC_ARM_JOINT_STATES = f"{DEVICE_PATH}/arm/joint_states"
TOPIC_ARM_CMD_RESULT = f"{DEVICE_PATH}/arm/cmd_result"
TOPIC_MISSION_CMD = f"{DEVICE_PATH}/mission/cmd"
TOPIC_MISSION_STATUS = f"{DEVICE_PATH}/mission/status"
TOPIC_MISSION_CMD_RESULT = f"{DEVICE_PATH}/mission/cmd_result"
TOPIC_SYSTEM_CMD = f"{DEVICE_PATH}/system/cmd"
TOPIC_SYSTEM_CMD_RESULT = f"{DEVICE_PATH}/system/cmd_result"


def _yaw_to_quaternion(yaw: float):
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def _quaternion_to_yaw(q) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


class RobotAgent(Node):
    """Translate the central server's MQTT protocol to local ROS interfaces."""

    def __init__(self):
        super().__init__("robot_agent")

        self.declare_parameter("map_frame", "map")
        self.declare_parameter("initial_pose_topic", "/initialpose")
        self.declare_parameter("pose_topic", "/amcl_pose")
        self.declare_parameter("plan_topic", "/plan")
        self.declare_parameter("web_cmd_vel_topic", "/cmd_vel_teleop")
        self.declare_parameter("rail_cmd_vel_topic", "/cmd_rail")
        self.declare_parameter("rail_command_topic", "/rail_command")
        self.declare_parameter("arm_command_topic", "/arm_command")
        self.declare_parameter("capture_service", "/capture_image")
        self.declare_parameter("status_period_sec", 1.0)
        self.declare_parameter("jog_max_linear", 0.4)
        self.declare_parameter("jog_max_angular", 1.2)

        self.map_frame = str(self.get_parameter("map_frame").value)
        self.command_queue: queue.Queue = queue.Queue(maxsize=100)
        self.state_lock = threading.Lock()
        self.pose: Optional[Dict[str, float]] = None
        self.path = []
        self.battery: Optional[Dict[str, Any]] = None
        self.joints: Dict[str, float] = {}
        self.mode = "IDLE"
        self.task_state = "READY"
        self.error: Optional[str] = None
        self.emergency = False
        self.mqtt_connected = False
        self.last_command_id: Optional[str] = None
        self.selected_rails = []
        self.mission_state = "IDLE"
        self.arm_state = "IDLE"
        self.nav_goal_handle = None
        self.rail_goal_handle = None
        self.nav_command_id = None
        self.rail_command_id = None
        self.jog_deadline: Optional[float] = None
        self.base_jog_active = False
        self.rail_jog_active = False
        self.mission_trigger_timer = None

        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            str(self.get_parameter("initial_pose_topic").value), 10)
        self.cmd_vel_pub = self.create_publisher(
            Twist, str(self.get_parameter("web_cmd_vel_topic").value), 10)
        self.rail_cmd_vel_pub = self.create_publisher(
            Twist, str(self.get_parameter("rail_cmd_vel_topic").value), 10)
        self.rail_command_pub = self.create_publisher(
            String, str(self.get_parameter("rail_command_topic").value), 10)
        self.arm_command_pub = self.create_publisher(
            String, str(self.get_parameter("arm_command_topic").value), 10)
        self.selected_rails_pub = self.create_publisher(String, "/selected_rails", 10)
        self.mission_trigger_pub = self.create_publisher(Bool, "/mission_trigger", 10)
        self.mission_halt_pub = self.create_publisher(Bool, "/mission_halt", 10)
        self.return_home_pub = self.create_publisher(Bool, "/return_home", 10)
        self.emergency_pub = self.create_publisher(Bool, "/emergency_stop", 10)

        self.create_subscription(
            PoseWithCovarianceStamped,
            str(self.get_parameter("pose_topic").value), self._pose_cb, 10)
        self.create_subscription(
            Path, str(self.get_parameter("plan_topic").value), self._path_cb, 10)
        self.create_subscription(BatteryState, "/battery_state", self._battery_cb, 10)
        self.create_subscription(JointState, "/joint_states", self._joints_cb, 10)
        self.create_subscription(
            String, "/arm_command_result", self._arm_command_result_cb, 10)

        self.nav_client = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self.rail_client = ActionClient(self, RailApproach, "/rail_approach")
        self.capture_client = self.create_client(
            Trigger, str(self.get_parameter("capture_service").value))

        try:
            self.mqtt = mqtt.Client(
                client_id=f"robot-agent-{ROBOT_ID}",
                protocol=mqtt.MQTTv311,
            )
        except TypeError:
            self.mqtt = mqtt.Client(client_id=f"robot-agent-{ROBOT_ID}")
        if MQTT_USERNAME:
            self.mqtt.username_pw_set(MQTT_USERNAME, MQTT_PASSWORD)
        self.mqtt.on_connect = self._mqtt_connect_cb
        self.mqtt.on_disconnect = self._mqtt_disconnect_cb
        self.mqtt.on_message = self._mqtt_message_cb
        self.mqtt.reconnect_delay_set(min_delay=1, max_delay=30)

        self.create_timer(0.05, self._process_commands)
        self.create_timer(0.05, self._safety_tick)
        self.create_timer(
            float(self.get_parameter("status_period_sec").value),
            self._publish_periodic)
        self._start_mqtt()
        self.get_logger().info(
            f"Robot agent ready: mqtt://{MQTT_BROKER}:{MQTT_PORT}/{DEVICE_PATH}")

    # MQTT callbacks -----------------------------------------------------
    def _start_mqtt(self):
        try:
            self.mqtt.connect_async(MQTT_BROKER, MQTT_PORT, keepalive=30)
            self.mqtt.loop_start()
        except Exception as exc:
            self.get_logger().error(f"MQTT start failed: {exc}")

    def _mqtt_connect_cb(self, client, userdata, flags, rc, properties=None):
        self.mqtt_connected = (int(rc) == 0)
        if not self.mqtt_connected:
            self.get_logger().error(f"MQTT connection rejected: rc={rc}")
            return
        for topic in (TOPIC_AMR_CMD, TOPIC_ARM_CMD, TOPIC_MISSION_CMD,
                      TOPIC_SYSTEM_CMD):
            client.subscribe(topic, qos=1)
        self.get_logger().info("MQTT connected and command topics subscribed")
        self._publish_event("ROBOT_AGENT_CONNECTED")

    def _mqtt_disconnect_cb(self, client, userdata, rc, properties=None):
        self.mqtt_connected = False
        self.get_logger().warning(f"MQTT disconnected: rc={rc}")

    def _mqtt_message_cb(self, client, userdata, msg):
        try:
            payload = json.loads(msg.payload.decode("utf-8"))
            if not isinstance(payload, dict):
                raise ValueError("payload must be a JSON object")
            self.command_queue.put_nowait((msg.topic, payload))
        except queue.Full:
            self.get_logger().error("Command queue full; MQTT command dropped")
        except Exception as exc:
            self.get_logger().warning(f"Invalid MQTT command on {msg.topic}: {exc}")

    def _mqtt_publish(self, topic: str, payload: Dict[str, Any], retain=False):
        try:
            self.mqtt.publish(
                topic, json.dumps(payload, ensure_ascii=False, allow_nan=False),
                qos=1, retain=retain)
        except Exception as exc:
            self.get_logger().warning(f"MQTT publish failed ({topic}): {exc}")

    # ROS telemetry ------------------------------------------------------
    def _pose_cb(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        with self.state_lock:
            self.pose = {"x": p.x, "y": p.y, "yaw": _quaternion_to_yaw(q),
                         "frame_id": msg.header.frame_id or self.map_frame}

    def _path_cb(self, msg):
        points = []
        # Bound MQTT packet size while retaining the route shape.
        stride = max(1, len(msg.poses) // 200)
        for stamped in msg.poses[::stride]:
            p = stamped.pose.position
            points.append({"x": p.x, "y": p.y})
        with self.state_lock:
            self.path = points

    def _battery_cb(self, msg):
        pct = float(msg.percentage)
        if math.isfinite(pct) and pct <= 1.0:
            pct *= 100.0
        self.battery = {
            "percentage": pct if math.isfinite(pct) else None,
            "voltage": msg.voltage if math.isfinite(msg.voltage) else None,
            "power_supply_status": int(msg.power_supply_status),
        }

    def _joints_cb(self, msg):
        self.joints = {
            name: float(value) for name, value in zip(msg.name, msg.position)
            if math.isfinite(value)
        }

    def _arm_command_result_cb(self, msg):
        try:
            payload = json.loads(msg.data)
            command_id = str(payload.get("command_id") or "")
            command = str(payload.get("command") or "MOVE_PRESET")
            status = str(payload.get("status") or "FAILED").upper()
            message = str(payload.get("message") or "")
            if not command_id:
                raise ValueError("command_id is required")
        except Exception as exc:
            self.get_logger().warning(f"Invalid /arm_command_result: {exc}")
            return

        self.arm_state = "IDLE" if status == "SUCCEEDED" else status
        self._result(
            TOPIC_ARM_CMD_RESULT,
            command_id,
            command,
            status,
            message,
        )

    # Command routing ----------------------------------------------------
    def _process_commands(self):
        for _ in range(10):
            try:
                topic, payload = self.command_queue.get_nowait()
            except queue.Empty:
                return
            self._handle_command(topic, payload)

    def _handle_command(self, topic: str, payload: Dict[str, Any]):
        command_id = str(payload.get("command_id") or "")
        command = str(payload.get("command") or "").upper()
        params = payload.get("params") or {}
        result_topic = self._result_topic(topic)
        self.last_command_id = command_id

        if not command_id or not command or not isinstance(params, dict):
            self._result(result_topic, command_id, command, "REJECTED",
                         "command_id, command and object params are required")
            return
        timestamp = payload.get("timestamp")
        ttl_ms = payload.get("ttl_ms")
        if ttl_ms is not None and timestamp is not None:
            try:
                if time.time() > float(timestamp) + float(ttl_ms) / 1000.0:
                    self._result(result_topic, command_id, command, "EXPIRED",
                                 "command TTL expired")
                    return
            except (TypeError, ValueError):
                self._result(result_topic, command_id, command, "REJECTED",
                             "invalid timestamp or ttl_ms")
                return
        clear_emergency = (
            topic == TOPIC_SYSTEM_CMD and command == "CLEAR_EMERGENCY"
        )
        if self.emergency and not clear_emergency:
            self._result(result_topic, command_id, command, "REJECTED",
                         "COMMAND_BLOCKED_BY_EMERGENCY")
            return

        try:
            if topic == TOPIC_AMR_CMD:
                self._handle_amr(command_id, command, params, ttl_ms)
            elif topic == TOPIC_MISSION_CMD:
                self._handle_mission(command_id, command, params)
            elif topic == TOPIC_ARM_CMD:
                self._handle_arm(command_id, command, params)
            elif topic == TOPIC_SYSTEM_CMD:
                self._handle_system(command_id, command)
            else:
                self._result(result_topic, command_id, command, "REJECTED",
                             "unknown command topic")
        except Exception as exc:
            self.error = str(exc)
            self.get_logger().error(f"Command {command} failed: {exc}")
            self._result(result_topic, command_id, command, "FAILED", str(exc))

    @staticmethod
    def _result_topic(command_topic: str) -> str:
        return {
            TOPIC_AMR_CMD: TOPIC_AMR_CMD_RESULT,
            TOPIC_ARM_CMD: TOPIC_ARM_CMD_RESULT,
            TOPIC_MISSION_CMD: TOPIC_MISSION_CMD_RESULT,
            TOPIC_SYSTEM_CMD: TOPIC_SYSTEM_CMD_RESULT,
        }.get(command_topic, TOPIC_SYSTEM_CMD_RESULT)

    def _handle_amr(self, cid, command, params, ttl_ms):
        if command == "SET_INITIAL_POSE":
            msg = PoseWithCovarianceStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.map_frame
            msg.pose.pose.position.x = float(params["x"])
            msg.pose.pose.position.y = float(params["y"])
            qx, qy, qz, qw = _yaw_to_quaternion(float(params.get("yaw", 0.0)))
            msg.pose.pose.orientation.x, msg.pose.pose.orientation.y = qx, qy
            msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = qz, qw
            msg.pose.covariance[0] = msg.pose.covariance[7] = 0.25
            msg.pose.covariance[35] = 0.0685
            self.initial_pose_pub.publish(msg)
            self._result(TOPIC_AMR_CMD_RESULT, cid, command, "SUCCEEDED")
        elif command == "NAVIGATE":
            self._navigate(cid, command, params)
        elif command in ("CANCEL_NAVIGATION", "STOP"):
            self._cancel_navigation()
            self._stop_motion()
            self.mode, self.task_state = "IDLE", "STOPPED"
            self._result(TOPIC_AMR_CMD_RESULT, cid, command, "SUCCEEDED")
        elif command == "JOG":
            self._jog(params, ttl_ms)
            self._result(TOPIC_AMR_CMD_RESULT, cid, command, "SUCCEEDED")
        elif command == "RAIL_ENTER":
            self._rail_approach(cid, command, params)
        elif command == "CANCEL_RAIL_APPROACH":
            self._cancel_rail()
            self._stop_motion()
            self._result(TOPIC_AMR_CMD_RESULT, cid, command, "SUCCEEDED")
        elif command == "RAIL_EXIT":
            self.rail_command_pub.publish(String(data="BACK"))
            self.mode, self.task_state = "MANUAL", "RAIL_EXIT"
            self._result(TOPIC_AMR_CMD_RESULT, cid, command, "SUCCEEDED")
        else:
            self._result(TOPIC_AMR_CMD_RESULT, cid, command, "REJECTED",
                         "UNKNOWN_COMMAND")

    def _navigate(self, cid, command, params):
        if not self.nav_client.server_is_ready():
            self.nav_client.wait_for_server(timeout_sec=0.2)
        if not self.nav_client.server_is_ready():
            self._result(TOPIC_AMR_CMD_RESULT, cid, command, "FAILED",
                         "NAV_SERVER_NOT_AVAILABLE")
            return
        goal = NavigateToPose.Goal()
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.header.frame_id = self.map_frame
        goal.pose.pose.position.x = float(params["x"])
        goal.pose.pose.position.y = float(params["y"])
        qx, qy, qz, qw = _yaw_to_quaternion(float(params.get("yaw", 0.0)))
        goal.pose.pose.orientation.x, goal.pose.pose.orientation.y = qx, qy
        goal.pose.pose.orientation.z, goal.pose.pose.orientation.w = qz, qw
        self.mode, self.task_state = "AUTO", "SENDING_NAV_GOAL"
        self.nav_command_id = cid
        future = self.nav_client.send_goal_async(goal, self._nav_feedback)
        future.add_done_callback(self._nav_goal_response)
        self._result(TOPIC_AMR_CMD_RESULT, cid, command, "ACCEPTED")

    def _nav_feedback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.task_state = "NAVIGATING"
        distance = getattr(feedback, "distance_remaining", None)
        if distance is not None:
            self._mqtt_publish(TOPIC_AMR_STATUS, self._amr_status(distance))

    def _nav_goal_response(self, future):
        try:
            handle = future.result()
            if not handle.accepted:
                self._result(TOPIC_AMR_CMD_RESULT, self.nav_command_id,
                             "NAVIGATE", "FAILED", "NAV_GOAL_REJECTED")
                self.mode, self.task_state = "IDLE", "READY"
                return
            self.nav_goal_handle = handle
            handle.get_result_async().add_done_callback(self._nav_result)
        except Exception as exc:
            self._result(TOPIC_AMR_CMD_RESULT, self.nav_command_id,
                         "NAVIGATE", "FAILED", str(exc))

    def _nav_result(self, future):
        wrapped = future.result()
        success = wrapped.status == GoalStatus.STATUS_SUCCEEDED
        self.mode = "IDLE"
        self.task_state = "READY" if success else "ERROR"
        self.nav_goal_handle = None
        self._result(TOPIC_AMR_CMD_RESULT, self.nav_command_id, "NAVIGATE",
                     "SUCCEEDED" if success else "FAILED",
                     "" if success else f"nav status={wrapped.status}")

    def _rail_approach(self, cid, command, params):
        if not self.rail_client.server_is_ready():
            self.rail_client.wait_for_server(timeout_sec=0.2)
        if not self.rail_client.server_is_ready():
            self._result(TOPIC_AMR_CMD_RESULT, cid, command, "FAILED",
                         "RAIL_SERVER_NOT_AVAILABLE")
            return
        goal = RailApproach.Goal()
        goal.timeout_sec = float(params.get("timeout_sec", 60.0))
        goal.x_tolerance = float(params.get("x_tolerance", 0.08))
        goal.angle_tolerance = float(params.get("angle_tolerance", 1.0))
        goal.allow_reverse_align = bool(params.get("allow_reverse_align", True))
        self.mode, self.task_state = "AUTO", "RAIL_APPROACH"
        self.rail_command_id = cid
        future = self.rail_client.send_goal_async(goal, self._rail_feedback)
        future.add_done_callback(self._rail_goal_response)
        self._result(TOPIC_AMR_CMD_RESULT, cid, command, "ACCEPTED")

    def _rail_feedback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.task_state = f"RAIL_{getattr(feedback, 'state', 'RUNNING')}"

    def _rail_goal_response(self, future):
        try:
            handle = future.result()
            if not handle.accepted:
                self._result(TOPIC_AMR_CMD_RESULT, self.rail_command_id,
                             "RAIL_ENTER", "FAILED", "RAIL_GOAL_REJECTED")
                self.mode, self.task_state = "IDLE", "READY"
                return
            self.rail_goal_handle = handle
            handle.get_result_async().add_done_callback(self._rail_result)
        except Exception as exc:
            self._result(TOPIC_AMR_CMD_RESULT, self.rail_command_id,
                         "RAIL_ENTER", "FAILED", str(exc))

    def _rail_result(self, future):
        wrapped = future.result()
        result = wrapped.result
        action_succeeded = wrapped.status == GoalStatus.STATUS_SUCCEEDED
        result_succeeded = bool(getattr(result, "success", True))
        success = action_succeeded and result_succeeded
        reason = str(getattr(result, "reason", ""))
        self.mode, self.task_state = ("IDLE", "READY" if success else "ERROR")
        self.rail_goal_handle = None
        self._result(TOPIC_AMR_CMD_RESULT, self.rail_command_id, "RAIL_ENTER",
                     "SUCCEEDED" if success else "FAILED", reason)

    def _jog(self, params, ttl_ms):
        if str(params.get("mode", "NORMAL_JOG")).upper() == "RAIL_JOG":
            if self.base_jog_active:
                self.cmd_vel_pub.publish(Twist())
                self.base_jog_active = False
            direction = str(params.get("direction", "STOP")).upper()
            direction_sign = {
                "FORWARD": 1.0,
                "BACKWARD": -1.0,
                "BACK": -1.0,
                "STOP": 0.0,
            }.get(direction)
            if direction_sign is None:
                raise ValueError(f"invalid rail direction: {direction}")
            requested_speed = abs(float(params.get("speed", 0.0)))
            rail_twist = Twist()
            rail_twist.linear.x = direction_sign * requested_speed
            self.rail_cmd_vel_pub.publish(rail_twist)
            self.rail_jog_active = direction_sign != 0.0
        else:
            if self.rail_jog_active:
                self.rail_cmd_vel_pub.publish(Twist())
                self.rail_jog_active = False
            max_v = float(self.get_parameter("jog_max_linear").value)
            max_w = float(self.get_parameter("jog_max_angular").value)
            vx = max(-max_v, min(max_v, float(params.get("vx", 0.0))))
            wz = max(-max_w, min(max_w, float(params.get("wz", 0.0))))
            twist = Twist()
            twist.linear.x = vx
            twist.angular.z = wz
            self.cmd_vel_pub.publish(twist)
            self.base_jog_active = vx != 0.0 or wz != 0.0
        duration = max(0.05, min(float(ttl_ms or 500) / 1000.0, 2.0))
        self.jog_deadline = time.monotonic() + duration
        self.mode, self.task_state = "MANUAL", "JOGGING"

    def _handle_mission(self, cid, command, params):
        if command in ("SELECT_RAILS", "START"):
            rails = self._normalise_rails(params.get("rails", []))
            if not rails:
                self._result(TOPIC_MISSION_CMD_RESULT, cid, command, "REJECTED",
                             "selected rails list is empty")
                return
            self.selected_rails = rails
            self.selected_rails_pub.publish(
                String(data=",".join(str(value) for value in rails)))
            if command == "START":
                # Let the BT receive selected rails before its trigger callback.
                self._schedule_mission_trigger()
                self.mission_state = "RUNNING"
            self._result(TOPIC_MISSION_CMD_RESULT, cid, command, "SUCCEEDED")
        elif command == "PAUSE":
            self.mission_halt_pub.publish(Bool(data=True))
            self._stop_motion()
            self.mission_state = "PAUSED"
            self._result(TOPIC_MISSION_CMD_RESULT, cid, command, "SUCCEEDED")
        elif command == "RESUME":
            if not self.selected_rails:
                self._result(TOPIC_MISSION_CMD_RESULT, cid, command, "REJECTED",
                             "no selected rails to resume")
                return
            self.selected_rails_pub.publish(
                String(data=",".join(str(value) for value in self.selected_rails)))
            self._schedule_mission_trigger()
            self.mission_state = "RUNNING"
            self._result(TOPIC_MISSION_CMD_RESULT, cid, command, "SUCCEEDED")
        elif command == "CANCEL":
            self.mission_halt_pub.publish(Bool(data=True))
            self._cancel_navigation()
            self._cancel_rail()
            self._stop_motion()
            self.mission_state = "CANCELLED"
            self._result(TOPIC_MISSION_CMD_RESULT, cid, command, "SUCCEEDED")
        elif command == "RETURN_HOME":
            self.return_home_pub.publish(Bool(data=True))
            self.mission_state = "RETURNING_HOME"
            self._result(TOPIC_MISSION_CMD_RESULT, cid, command, "SUCCEEDED")
        else:
            self._result(TOPIC_MISSION_CMD_RESULT, cid, command, "REJECTED",
                         "UNKNOWN_COMMAND")

    def _schedule_mission_trigger(self):
        if self.mission_trigger_timer is not None:
            self.mission_trigger_timer.cancel()
        self.mission_trigger_timer = self.create_timer(
            0.15, self._publish_mission_trigger_once)

    def _publish_mission_trigger_once(self):
        self.mission_trigger_pub.publish(Bool(data=True))
        if self.mission_trigger_timer is not None:
            self.mission_trigger_timer.cancel()

    @staticmethod
    def _normalise_rails(values):
        if not isinstance(values, list):
            return []
        result = []
        for value in values:
            try:
                rail = int(value)
            except (TypeError, ValueError):
                continue
            if rail > 0 and rail not in result:
                result.append(rail)
        return result

    def _handle_arm(self, cid, command, params):
        if command in ("MOVE_PRESET", "CANCEL"):
            payload = {"command": command, "params": params,
                       "command_id": cid}
            self.arm_command_pub.publish(
                String(data=json.dumps(payload, ensure_ascii=False)))
            self.arm_state = "MOVING" if command == "MOVE_PRESET" else "IDLE"
            self._result(TOPIC_ARM_CMD_RESULT, cid, command, "ACCEPTED")
        elif command == "CAPTURE_TRIGGER":
            if not self.capture_client.service_is_ready():
                self.capture_client.wait_for_service(timeout_sec=0.2)
            if not self.capture_client.service_is_ready():
                self._result(TOPIC_ARM_CMD_RESULT, cid, command, "FAILED",
                             "CAPTURE_SERVER_NOT_AVAILABLE")
                return
            self.arm_state = "CAPTURING"
            future = self.capture_client.call_async(Trigger.Request())
            future.add_done_callback(
                lambda done: self._capture_result(done, cid, command))
            self._result(TOPIC_ARM_CMD_RESULT, cid, command, "ACCEPTED")
        else:
            self._result(TOPIC_ARM_CMD_RESULT, cid, command, "REJECTED",
                         "UNKNOWN_COMMAND")

    def _capture_result(self, future, cid, command):
        try:
            response = future.result()
            success, reason = bool(response.success), str(response.message)
        except Exception as exc:
            success, reason = False, str(exc)
        self.arm_state = "IDLE" if success else "ERROR"
        self._result(TOPIC_ARM_CMD_RESULT, cid, command,
                     "SUCCEEDED" if success else "FAILED", reason)

    def _handle_system(self, cid, command):
        if command == "EMERGENCY_STOP":
            self.emergency = True
            self._cancel_navigation()
            self._cancel_rail()
            self.mission_halt_pub.publish(Bool(data=True))
            self.rail_command_pub.publish(String(data="STOP"))
            self._stop_motion()
            self.emergency_pub.publish(Bool(data=True))
            self.mode, self.task_state = "EMERGENCY_STOP", "STOPPED"
            self.mission_state = "EMERGENCY_STOP"
            self._result(TOPIC_SYSTEM_CMD_RESULT, cid, command, "SUCCEEDED")
        elif command == "CLEAR_EMERGENCY":
            self.emergency = False
            self.emergency_pub.publish(Bool(data=False))
            self.mode, self.task_state, self.error = "IDLE", "READY", None
            self._result(TOPIC_SYSTEM_CMD_RESULT, cid, command, "SUCCEEDED")
        else:
            self._result(TOPIC_SYSTEM_CMD_RESULT, cid, command, "REJECTED",
                         "UNKNOWN_COMMAND")

    # Safety/status ------------------------------------------------------
    def _cancel_navigation(self):
        if self.nav_goal_handle is not None:
            self.nav_goal_handle.cancel_goal_async()

    def _cancel_rail(self):
        if self.rail_goal_handle is not None:
            self.rail_goal_handle.cancel_goal_async()

    def _stop_motion(self):
        self.cmd_vel_pub.publish(Twist())
        self.base_jog_active = False
        if self.rail_jog_active:
            self.rail_cmd_vel_pub.publish(Twist())
            self.rail_jog_active = False
        self.jog_deadline = None

    def _safety_tick(self):
        if self.jog_deadline is not None and time.monotonic() >= self.jog_deadline:
            self._stop_motion()
            if not self.emergency:
                self.mode, self.task_state = "IDLE", "READY"

    def _result(self, topic, cid, command, status, message=""):
        payload = {
            "robot_id": ROBOT_ID,
            "command_id": cid,
            "command": command,
            "status": status,
            "success": status == "SUCCEEDED",
            "message": message,
            "timestamp": time.time(),
        }
        self._mqtt_publish(topic, payload)
        if status in ("FAILED", "REJECTED", "EXPIRED"):
            self._publish_event("COMMAND_FAILED", payload)

    def _amr_status(self, distance_remaining=None):
        with self.state_lock:
            pose = dict(self.pose) if self.pose else None
        return {
            "robot_id": ROBOT_ID,
            "state": self.mode,
            "task_state": self.task_state,
            "pose": pose,
            "distance_remaining": distance_remaining,
            "emergency_stop": self.emergency,
            "error": self.error,
            "timestamp": time.time(),
        }

    def _publish_periodic(self):
        amr = self._amr_status()
        self._mqtt_publish(TOPIC_AMR_STATUS, amr, retain=True)
        if self.pose is not None:
            self._mqtt_publish(TOPIC_AMR_POSE, {
                "robot_id": ROBOT_ID, "pose": self.pose, "timestamp": time.time()})
        if self.path:
            self._mqtt_publish(TOPIC_AMR_PATH, {
                "robot_id": ROBOT_ID, "path": self.path, "timestamp": time.time()})
        arm = {
            "robot_id": ROBOT_ID,
            "connection": "ONLINE" if self.joints else "UNKNOWN",
            "state": self.arm_state,
            "joints": self.joints,
            "timestamp": time.time(),
        }
        self._mqtt_publish(TOPIC_ARM_STATUS, arm, retain=True)
        if self.joints:
            self._mqtt_publish(TOPIC_ARM_JOINT_STATES, {
                "robot_id": ROBOT_ID, "joints": self.joints,
                "timestamp": time.time()})
        mission = {
            "robot_id": ROBOT_ID, "state": self.mission_state,
            "selected_rails": self.selected_rails, "timestamp": time.time()}
        self._mqtt_publish(TOPIC_MISSION_STATUS, mission, retain=True)
        health = {
            "robot_id": ROBOT_ID,
            "mqtt_connected": self.mqtt_connected,
            "ros_ok": rclpy.ok(),
            "battery": self.battery,
            "timestamp": time.time(),
        }
        self._mqtt_publish(TOPIC_HEALTH, health, retain=True)
        self._mqtt_publish(TOPIC_STATUS, {
            "robot_id": ROBOT_ID,
            "device_path": DEVICE_PATH,
            "state": self.mode,
            "task_state": self.task_state,
            "pose": self.pose,
            "amr": amr,
            "arm": arm,
            "routine": mission,
            "health": health,
            "last_seen": time.time(),
        }, retain=True)

    def _publish_event(self, event: str, extra=None):
        payload = {"robot_id": ROBOT_ID, "event": event, "timestamp": time.time()}
        if isinstance(extra, dict):
            payload.update(extra)
        self._mqtt_publish(TOPIC_EVENT, payload)

    def destroy_node(self):
        self._stop_motion()
        self.rail_command_pub.publish(String(data="STOP"))
        self._publish_event("ROBOT_AGENT_SHUTDOWN")
        try:
            self.mqtt.disconnect()
            self.mqtt.loop_stop()
        except Exception:
            pass
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RobotAgent()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
