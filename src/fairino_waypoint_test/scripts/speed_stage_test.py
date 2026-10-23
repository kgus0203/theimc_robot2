#!/usr/bin/env python3
"""Run one bounded Fairino speed-test stage and record its ROS data."""

import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
from datetime import datetime

from action_msgs.msg import GoalStatus
from control_msgs.msg import JointTrajectoryControllerState
import rclpy
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data

from fairino_waypoint_test.action import ScanSide


THRESHOLDS = (0.05, 0.10, 0.15, 0.20)


def percentile(values, percent):
    if not values:
        return None
    ordered = sorted(values)
    index = (len(ordered) - 1) * percent / 100.0
    lower = math.floor(index)
    upper = math.ceil(index)
    if lower == upper:
        return ordered[lower]
    return ordered[lower] * (upper - index) + ordered[upper] * (index - lower)


def unique_bag_path(requested):
    path = Path(requested)
    if not path.exists():
        return path
    suffix = datetime.now().strftime("%Y%m%d_%H%M%S")
    candidate = path.with_name(f"{path.name}_{suffix}")
    sequence = 1
    while candidate.exists():
        candidate = path.with_name(f"{path.name}_{suffix}_{sequence}")
        sequence += 1
    return candidate


class StageRunner:
    def __init__(self, args):
        self.args = args
        self.node = rclpy.create_node("fairino_speed_stage_test")
        self.client = ActionClient(self.node, ScanSide, "/scan_side")
        self.errors = [[] for _ in range(6)]
        self.all_errors = []
        self.threshold_counts = {str(value): 0 for value in THRESHOLDS}
        self.threshold_counts_by_joint = {
            str(value): [0] * 6 for value in THRESHOLDS
        }
        self.run_summaries = []
        self.active_run = None
        self.abort_reason = None
        self.warning_005 = False
        self.cancel_requested = False
        self.goal_handle = None
        self.successful_coordinates = 0
        self.returned_to_standby = False
        self.stage_started_epoch = None
        self.stage_ended_epoch = None
        self.last_sample_time = None
        self.last_sample_moving = False
        self.node.create_subscription(
            JointTrajectoryControllerState,
            "/fairino5_controller/controller_state",
            self.controller_state,
            qos_profile_sensor_data,
        )

    def controller_state(self, message):
        if self.active_run is None:
            return
        positions = list(message.error.positions)
        if len(positions) < 6:
            return
        now = time.monotonic()
        moving = any(abs(value) > 1e-4 for value in message.desired.velocities)
        if (
            self.last_sample_time is not None
            and self.last_sample_moving
            and now - self.last_sample_time < 0.2
        ):
            self.active_run["movement_time_seconds"] += now - self.last_sample_time
        self.last_sample_time = now
        self.last_sample_moving = moving

        sample_max = 0.0
        names = list(message.joint_names)
        for source_index, raw in enumerate(positions[:6]):
            name = names[source_index] if source_index < len(names) else f"j{source_index + 1}"
            if len(name) == 2 and name[0] == "j" and name[1].isdigit():
                index = int(name[1]) - 1
            else:
                index = source_index
            if not 0 <= index < 6:
                continue
            value = abs(raw)
            if not math.isfinite(value):
                self.abort_reason = f"non-finite tracking error on J{index + 1}"
                continue
            self.errors[index].append(value)
            self.all_errors.append(value)
            self.active_run["max_error_by_joint"][index] = max(
                self.active_run["max_error_by_joint"][index], value
            )
            sample_max = max(sample_max, value)
            for threshold in THRESHOLDS:
                if value > threshold:
                    key = str(threshold)
                    self.threshold_counts[key] += 1
                    self.threshold_counts_by_joint[key][index] += 1
        self.active_run["max_error"] = max(self.active_run["max_error"], sample_max)
        if sample_max > 0.05 and not self.warning_005:
            self.warning_005 = True
            print(f"WARNING tracking error exceeded 0.05 rad: {sample_max:.6f}", flush=True)
        if sample_max >= 0.10 and self.abort_reason is None:
            self.abort_reason = f"tracking error reached {sample_max:.6f} rad"
            print(f"STOP {self.abort_reason}", flush=True)

    def feedback(self, run_number, message):
        stage = message.feedback.stage
        if stage.startswith("reached:"):
            self.successful_coordinates += 1
        if stage == "returned to standby":
            self.returned_to_standby = True
        if stage.startswith("reached:") or stage in (
            "returning to standby", "returned to standby"
        ):
            print(
                f"RUN {run_number}/{self.args.repeats} {stage} "
                f"({message.feedback.completed_waypoints}/"
                f"{message.feedback.total_waypoints})",
                flush=True,
            )

    def spin_until(self, future, timeout):
        deadline = time.monotonic() + timeout
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if (
                self.abort_reason
                and self.goal_handle is not None
                and not self.cancel_requested
            ):
                self.cancel_requested = True
                self.goal_handle.cancel_goal_async()
                print("Cancellation requested; waiting for current MoveIt operation", flush=True)
        return future.done()

    def run(self):
        if not self.client.wait_for_server(timeout_sec=30.0):
            self.abort_reason = "/scan_side action unavailable"
            return
        self.stage_started_epoch = time.time()
        for run_number in range(1, self.args.repeats + 1):
            self.returned_to_standby = False
            self.cancel_requested = False
            self.goal_handle = None
            self.last_sample_time = None
            self.last_sample_moving = False
            self.active_run = {
                "run": run_number,
                "action_time_seconds": 0.0,
                "movement_time_seconds": 0.0,
                "max_error": 0.0,
                "max_error_by_joint": [0.0] * 6,
                "successful_coordinates": 0,
                "returned_to_standby": False,
                "success": False,
                "message": "",
            }
            coordinates_before = self.successful_coordinates
            goal = ScanSide.Goal()
            goal.side = self.args.side
            goal.capture_service = ""
            goal.inspection_id = ""
            print(f"RUN {run_number}/{self.args.repeats} START", flush=True)
            started = time.monotonic()
            send_future = self.client.send_goal_async(
                goal,
                feedback_callback=lambda msg, number=run_number: self.feedback(number, msg),
            )
            if not self.spin_until(send_future, 30.0):
                self.abort_reason = self.abort_reason or "goal send timeout"
                self.active_run["message"] = self.abort_reason
            else:
                self.goal_handle = send_future.result()
                if self.goal_handle is None or not self.goal_handle.accepted:
                    self.abort_reason = self.abort_reason or "goal rejected"
                    self.active_run["message"] = self.abort_reason
                else:
                    result_future = self.goal_handle.get_result_async()
                    if not self.spin_until(result_future, self.args.action_timeout):
                        self.abort_reason = self.abort_reason or "action result timeout"
                        self.active_run["message"] = self.abort_reason
                    else:
                        wrapped = result_future.result()
                        result = wrapped.result
                        success = (
                            wrapped.status == GoalStatus.STATUS_SUCCEEDED
                            and result.success
                            and result.completed_waypoints == self.args.waypoints
                            and self.returned_to_standby
                        )
                        self.active_run["success"] = success
                        self.active_run["message"] = result.message
                        if not success:
                            self.abort_reason = self.abort_reason or (
                                f"action failed: status={wrapped.status}, "
                                f"completed={result.completed_waypoints}, message={result.message}"
                            )
            self.active_run["action_time_seconds"] = time.monotonic() - started
            self.active_run["successful_coordinates"] = (
                self.successful_coordinates - coordinates_before
            )
            self.active_run["returned_to_standby"] = self.returned_to_standby
            self.run_summaries.append(self.active_run)
            print(
                f"RUN {run_number} RESULT success={self.active_run['success']} "
                f"coordinates={self.active_run['successful_coordinates']} "
                f"movement={self.active_run['movement_time_seconds']:.3f}s "
                f"max_error={self.active_run['max_error']:.6f} "
                f"standby={self.returned_to_standby}",
                flush=True,
            )
            self.active_run = None
            self.goal_handle = None
            if self.abort_reason:
                break
            time.sleep(1.0)
        self.stage_ended_epoch = time.time()

    def summary(self, bag_path):
        max_by_joint = [max(values, default=0.0) for values in self.errors]
        overall = max(max_by_joint, default=0.0)
        run_maxima = [run["max_error"] for run in self.run_summaries]
        accumulation = False
        if len(run_maxima) >= 2:
            accumulation = run_maxima[-1] > run_maxima[0] + max(
                0.005, run_maxima[0] * 0.20
            )
        return {
            "velocity_scale": self.args.velocity,
            "acceleration_scale": self.args.acceleration,
            "side": self.args.side,
            "requested_repeats": self.args.repeats,
            "successful_repeats": sum(run["success"] for run in self.run_summaries),
            "successful_coordinates": self.successful_coordinates,
            "requested_coordinates": self.args.repeats * self.args.waypoints,
            "bag_path": str(bag_path),
            "stage_started_epoch": self.stage_started_epoch,
            "stage_ended_epoch": self.stage_ended_epoch,
            "run_summaries": self.run_summaries,
            "total_movement_time_seconds": sum(
                run["movement_time_seconds"] for run in self.run_summaries
            ),
            "total_action_time_seconds": sum(
                run["action_time_seconds"] for run in self.run_summaries
            ),
            "max_error_by_joint": max_by_joint,
            "overall_max_error": overall,
            "overall_max_error_joint": (
                f"J{max_by_joint.index(overall) + 1}" if max_by_joint else None
            ),
            "mean_absolute_error": (
                sum(self.all_errors) / len(self.all_errors) if self.all_errors else None
            ),
            "p95_absolute_error": percentile(self.all_errors, 95),
            "p99_absolute_error": percentile(self.all_errors, 99),
            "threshold_exceedance_counts": self.threshold_counts,
            "threshold_exceedance_counts_by_joint": self.threshold_counts_by_joint,
            "warning_over_0_05": self.warning_005,
            "tracking_error_accumulation": accumulation,
            "action_failure": self.abort_reason,
            "all_runs_returned_to_standby": bool(self.run_summaries) and all(
                run["returned_to_standby"] for run in self.run_summaries
            ),
            "servo_timing": None,
            "robot_alarm": None,
            "path_tolerance_violated": None,
            "servo_code_14": None,
            "physical_observation": "pending user confirmation",
            "pass_from_action_and_tracking": (
                self.abort_reason is None
                and len(self.run_summaries) == self.args.repeats
                and all(run["success"] for run in self.run_summaries)
                and overall < 0.10
                and not accumulation
            ),
        }


def start_bag(path):
    command = [
        "ros2", "bag", "record", "-o", str(path),
        "/fairino5_controller/controller_state", "/arm/joint_states",
    ]
    log = open(f"/tmp/{path.name}_rosbag.log", "w", encoding="utf-8")
    process = subprocess.Popen(
        command,
        stdout=log,
        stderr=subprocess.STDOUT,
        start_new_session=True,
    )
    time.sleep(2.0)
    if process.poll() is not None:
        log.close()
        raise RuntimeError(f"rosbag recorder exited with code {process.returncode}")
    return process, log


def stop_bag(process, log):
    if process.poll() is None:
        os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=15.0)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGTERM)
            process.wait(timeout=5.0)
    log.close()


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--velocity", type=float, required=True)
    parser.add_argument("--acceleration", type=float, required=True)
    parser.add_argument("--repeats", type=int, default=2)
    parser.add_argument("--waypoints", type=int, default=3)
    parser.add_argument("--side", choices=("left", "right"), default="right")
    parser.add_argument("--bag", required=True)
    parser.add_argument("--action-timeout", type=float, default=300.0)
    args = parser.parse_args()
    if not 0.0 < args.velocity <= 1.0 or not 0.0 < args.acceleration <= 1.0:
        parser.error("velocity and acceleration must be in (0, 1]")
    if args.repeats < 1 or args.waypoints < 1:
        parser.error("repeats and waypoints must be positive")
    return args


def main():
    args = parse_args()
    bag_path = unique_bag_path(args.bag)
    print(f"BAG {bag_path}", flush=True)
    rclpy.init()
    runner = StageRunner(args)
    bag_process = None
    bag_log = None
    exit_code = 1
    try:
        bag_process, bag_log = start_bag(bag_path)
        runner.run()
        exit_code = 0 if runner.abort_reason is None else 2
    except KeyboardInterrupt:
        runner.abort_reason = runner.abort_reason or "operator interrupted"
        print("STOP operator interrupted", flush=True)
    except Exception as error:  # Keep a machine-readable summary on infrastructure failures.
        runner.abort_reason = runner.abort_reason or str(error)
        print(f"FATAL {error}", flush=True)
    finally:
        if bag_process is not None:
            stop_bag(bag_process, bag_log)
        summary = runner.summary(bag_path)
        if bag_path.exists():
            summary_path = bag_path / "stage_summary.json"
        else:
            summary_path = Path(f"/tmp/{bag_path.name}_stage_summary.json")
        summary_path.write_text(
            json.dumps(summary, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
        print(f"SUMMARY {summary_path}", flush=True)
        print(json.dumps(summary, ensure_ascii=False, indent=2), flush=True)
        runner.node.destroy_node()
        rclpy.shutdown()
    return exit_code


if __name__ == "__main__":
    sys.exit(main())
