#!/usr/bin/env python3

import os
import json
import math
import subprocess
import select
import shutil
import sys
import unicodedata
import time

import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, String
from std_srvs.srv import Trigger, SetBool
from rclpy.qos import QoSProfile

if os.name == 'nt':
    import msvcrt
else:
    import termios
    import tty

MAX_LIN_VEL = 1.0
MAX_ANG_VEL = 1.0
MAX_RAIL_VEL = 0.5

LIN_VEL_STEP_SIZE = 0.1
ANG_VEL_STEP_SIZE = 0.1
# Independent left/right wheel speed controls intentionally removed.
RAIL_VEL_STEP_SIZE = 0.1
RAIL_FIXED_VEL = 0.2

WHEEL_TELEOP_TOPIC = '/cmd_vel_teleop'
RAIL_COMMAND_TOPIC = '/rail_command'
RAIL_SPEED_SETPOINT_TOPIC = '/rail_speed_setpoint'
RAIL_STATE_TOPIC = '/rail_state'
CHARGER_COMMAND_TOPIC = '/charger/command'
RESET_FAULTS_SERVICE = '/agv/reset_faults'
RECOVERY_DIAG_SERVICE = '/agv/recovery_diagnostics'
RECOVER_STALE_SERVICE = '/agv/recover_stale_jobs'
APPROVE_ADS_SERVICE = '/agv/approve_ads_recovery'
APPROVE_POWER_SERVICE = '/agv/approve_power_restore'
# Independent, already-built Beckhoff TwinCAT maintenance executable.
TWINCAT_TOOL = os.environ.get('TWINCAT_TOOL', os.path.expanduser('~/AGVSource/Beckhoff/build/ads_app'))
WHEEL_POWER_SERVICE = '/agv/wheel_power'
RAIL_POWER_SERVICE = '/agv/rail_power'

# Display-width aware terminal layout, preserving the requested keyboard-map design.
# Korean characters, emoji, and terminal-width differences must not ruin columns.
def _display_width(value):
    try:
        from wcwidth import wcswidth
        width = wcswidth(value)
        if width >= 0:
            return width
    except ImportError:
        pass
    # No external dependency required on the robot's ROS2 Python environment.
    return sum(
        0 if unicodedata.combining(c) or c in ("\ufe0e", "\ufe0f", "\u200d")
        else 2 if unicodedata.east_asian_width(c) in ("W", "F") else 1
        for c in value
    )


def _pad_right(value, width):
    return value + " " * max(0, width - _display_width(value))


def _center(value, width):
    padding = max(0, (width - _display_width(value)) // 2)
    return " " * padding + value


def _side_by_side(left_lines, right_lines, left_width=38):
    out = []
    for idx in range(max(len(left_lines), len(right_lines))):
        left = left_lines[idx] if idx < len(left_lines) else ""
        right = right_lines[idx] if idx < len(right_lines) else ""
        out.append(" " + _pad_right(left, left_width) + "  " + right)
    return out


def show_help():
    width = shutil.get_terminal_size(fallback=(80, 24)).columns
    # At least 78 columns: side-by-side descriptions like the requested mockup.
    # Smaller windows retain the actual key drawings but stack descriptions
    # rather than emitting the unreadable, tightly-packed old compact help.
    side_by_side = width >= 78
    rule_width = min(88, max(56, width)) if width >= 56 else max(40, width)

    output = [
        "",
        "=" * rule_width,
        _center("⌨️  ROS2 Teleop Keyboard Map", rule_width),
        "=" * rule_width,
        "",
        " [숫자 키 (1~9)] - 레일 및 상태 제어",
        " ┌───┬───┬───┐     ┌───┬───┬───┬───┬───┬───┐",
        " │ 1 │ 2 │ 3 │     │ 4 │ 5 │ 6 │ 7 │ 8 │ 9 │",
        " └───┴───┴───┘     └───┴───┴───┴───┴───┴───┘",
    ]
    numeric_left = [
        "1 : 레일 전진 (+0.2 m/s)",
        "2 : 레일 후진 (-0.2 m/s)",
        "3 : 레일 정지 (0 m/s)",
    ]
    numeric_right = [
        "4 : RAIL_APPROACH  /  5 : OUT_RAIL",
        "6 : ON_RAIL       /  7 : WORKING",
        "8 : EXITING       /  9 : ENTERING",
    ]
    if side_by_side:
        output.extend(_side_by_side(numeric_left, numeric_right, 34))
    else:
        output.extend("   " + line for line in numeric_left)
        output.extend("   " + line for line in numeric_right)

    output.extend([
        "",
        "-" * rule_width,
        "",
        " [알파벳 키 (QWERTY)] - 바퀴+레일 독립 명령 (개별 바퀴 제어 제거)",
        " ┌───┬───┬───┬───┬───┐   ┌───┬───┬───┬───┬───┐",
        " │   │ W │ E │ R │ T │   │ Y │ U │ I │ O │ P │",
        " ├───┼───┼───┼───┼───┤   ├───┼───┼───┼───┼───┤",
        " │ A │ S │ D │ F │ G │   │ H │ J │ K │ L │   │",
        " ├───┼───┼───┼───┼───┤   ├───┼───┼───┼───┼───┤",
        " │ Z │ X │ C │   │ B │   │ N │ M │   │   │   │",
        " └───┴───┴───┴───┘   └───┴───┴───┴───┴───┘",
        "",
    ])

    ground = [
        "🚗 기본 이동 (Ground Wheel)",
        "  • W     : 직진 속도 +0.1 m/s",
        "  • A / D : 회전 속도 +0.1 / -0.1 rad/s",
        "  • X     : 직진 속도 -0.1 m/s",
        "  • S     : 바퀴만 정지",
        "  • M/Space : 바퀴+레일 동시 정지",
    ]
    power = [
        "⚙️ 모터 Power / 전원 / 리셋",
        "  • E / F : 바퀴 모터 ON / OFF",
        "  • Y / Z : 레일 모터 ON / OFF",
        "  • O / P : 충전기 ON / OFF",
        "  • R     : AGV 에러 리셋 (정지 3초 검증 후 READY)",
        "  • N     : Reset 후 Power 복구 승인",
        "  • V     : TwinCAT 즉시 Restart (모터 상태 검사 없음)",
    ]
    individual = [
        "🔗 바퀴·레일 독립 제어",
        "  • W/A/S/D/X : 바퀴만 제어",
        "  • 1/2/3/T/B/G : 레일만 제어",
        "  • M / Space : 양쪽 모두 정지",
        "  • U/J/I/K/H/L : 사용 안 함",
    ]
    rail = [
        "🚊 레일 미세 속도 (Rail Speed)",
        "  • T     : 레일 속도 +0.1 m/s",
        "  • B     : 레일 속도 -0.1 m/s",
        "  • G     : 레일 정지",
    ]
    if side_by_side:
        output.extend(_side_by_side(ground, power))
        output.append("")
        output.extend(_side_by_side(individual, rail))
    else:
        for block in (ground, power, individual, rail):
            output.extend(" " + line for line in block)
            output.append("")

    output.extend([
        "",
        "-" * rule_width,
        " [?] 단축키 다시 보기  |  [Ctrl+C] 종료",
        " [참고] 바퀴+레일 동시 구동은 AGV의 allow_simultaneous_drive=true 및 두 모터 Power ON 필요",
        " [R] 리셋 직전 STOP 미발행 (PLC 오류 시 정지 동작은 별도 보장 필요)",
        " [V] 즉시 TwinCAT Runtime Restart: 모터 상태/속도 검사 및 입력 확인 없음",
        " [주의] Runtime Restart는 비상정지가 아닙니다. 실제 로봇 정지 및 위험구역 안전 확보 후 V 사용",
        "=" * rule_width,
        "",
    ])
    print("\n".join(output), flush=True)


def get_key(settings):
    if os.name == 'nt':
        if msvcrt.kbhit():
            return msvcrt.getch().decode('utf-8', errors='ignore')
        return ''

    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)

    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def constrain(value, low_bound, high_bound):
    return max(low_bound, min(value, high_bound))


def check_linear_limit_velocity(velocity):
    return constrain(velocity, -MAX_LIN_VEL, MAX_LIN_VEL)


def check_angular_limit_velocity(velocity):
    return constrain(velocity, -MAX_ANG_VEL, MAX_ANG_VEL)


def check_rail_limit_velocity(velocity):
    return constrain(velocity, -MAX_RAIL_VEL, MAX_RAIL_VEL)


def step_linear_velocity(current, delta):
    # Keep wheel linear commands on exact 0.1 m/s increments.
    return round(
        check_linear_limit_velocity(current + delta),
        1
    )


def step_angular_velocity(current, delta):
    # Keep wheel angular commands on exact 0.1 rad/s increments.
    return round(
        check_angular_limit_velocity(current + delta),
        1
    )


def make_simple_profile(output, input_value, slop):
    if input_value > output:
        return min(input_value, output + slop)
    if input_value < output:
        return max(input_value, output - slop)
    return input_value


def print_vels(linear, angular, rail):
    print(
        f'\rlinear={linear:+.2f} m/s  '
        f'angular={angular:+.2f} rad/s  '
        f'rail={rail:+.2f} m/s      ',
        end='',
        flush=True
    )


def publish_wheel_stop(pub):
    pub.publish(Twist())


def publish_rail_command(pub, command):
    msg = String()
    msg.data = str(command).strip().upper()
    pub.publish(msg)


def publish_rail_speed_setpoint(pub, speed):
    msg = Float32()
    msg.data = float(speed)
    pub.publish(msg)


def publish_rail_state(pub, state):
    msg = String()
    msg.data = state
    pub.publish(msg)


def publish_charger_command(pub, command_str):
    """Send the String commands expected by bringup: ON / OFF."""
    msg = String()
    msg.data = str(command_str).strip().upper()
    pub.publish(msg)



def call_set_bool_service(node, client, service_name, enabled):
    if not client.wait_for_service(timeout_sec=1.0):
        print(f"\n[POWER][ERROR] {service_name} service is not available.")
        return

    request = SetBool.Request()
    request.data = bool(enabled)
    future = client.call_async(request)

    rclpy.spin_until_future_complete(node, future, timeout_sec=15.0)

    if not future.done():
        print(f"\n[POWER][ERROR] {service_name} timed out after 15 sec.")
        return

    try:
        response = future.result()
        state = "ON" if enabled else "OFF"
        if response is None:
            print(f"\n[POWER][ERROR] {service_name} returned an empty response.")
        elif response.success:
            print(f"\n[POWER][OK] {service_name} -> {state}: {response.message}")
        else:
            print(f"\n[POWER][FAIL] {service_name} -> {state}: {response.message}")
    except Exception as exc:
        print(f"\n[POWER][ERROR] {service_name} call failed: {exc}")




def _run_twincat_tool(*args, timeout=12):
    """Call only the known local executable; never use shell=True."""
    tool = os.path.realpath(os.path.expanduser(TWINCAT_TOOL))
    if not os.path.isfile(tool) or not os.access(tool, os.X_OK):
        print(f"\n[TWINCAT][ERROR] 실행 파일 없음/실행 불가: {tool}")
        return None
    try:
        result = subprocess.run([tool, *args], capture_output=True,
                                text=True, timeout=timeout, check=False)
    except subprocess.TimeoutExpired:
        print(f"\n[TWINCAT][ERROR] 시간 초과 ({timeout}s). PLC 상태를 별도 확인하세요.")
        return None
    except OSError as exc:
        print(f"\n[TWINCAT][ERROR] 실행 실패: {exc}")
        return None
    # The Beckhoff tool can produce hundreds of ADS trace lines; show only failures.
    if result.returncode != 0:
        detail = (result.stderr or result.stdout or '').strip().splitlines()
        for line in detail[-8:]:
            print(f'[TWINCAT][DETAIL] {line}')
        print(f'[TWINCAT][FAIL] exit={result.returncode}')
    return result.returncode


def _axes_safe_for_restart(last_axis_snapshot):
    """Fail closed. Require a recent full 3-axis *PLC* snapshot, all powered OFF."""
    if not last_axis_snapshot or time.monotonic() - last_axis_snapshot['time'] > 2.0:
        return False, '최신 PLC 축 진단이 없습니다(2초 이내 데이터 필요).'
    axes = last_axis_snapshot['axes']
    if len(axes) != 3 or {a.get('axis') for a in axes} != {1, 2, 3}:
        return False, 'Axis1/2/3 상태가 모두 유효하지 않습니다.'
    for a in axes:
        axis = a['axis']
        velocity = a.get('velocity_mps')
        if not isinstance(velocity, (float, int)) or isinstance(velocity, bool) or not math.isfinite(velocity) or not (-0.001 <= velocity <= 0.001):
            return False, f'Axis{axis} 측정 속도가 0이 아니거나 불명확합니다.'
        if a.get('ready') is not False:
            return False, f'Axis{axis} Power가 OFF로 확인되지 않았습니다.'
        if a.get('not_moving') is not True or a.get('has_job') is not False:
            return False, f'Axis{axis} NC 정지/Job 해제가 확인되지 않았습니다.'
        if a.get('moving_fw') is not False or a.get('moving_bw') is not False:
            return False, f'Axis{axis} 방향 플래그가 정지 상태가 아닙니다.'
        if a.get('error') is not False:
            return False, f'Axis{axis} Error가 남아 있습니다.'
    return True, 'Axis1/2/3 전원 OFF, NC 정지, Job 없음 확인'


def _refresh_axis_snapshot(node, snapshot_ref, duration=1.2):
    started = time.monotonic()
    deadline = started + duration
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        snapshot = snapshot_ref.get('data')
        if snapshot and snapshot['time'] > started:
            return _axes_safe_for_restart(snapshot)
    return False, '새 PLC 축 진단 메시지를 수신하지 못했습니다.'


def _call_trigger_service(node, service_name, timeout=8.0):
    """Response success acknowledges a request; do not confuse it with READY."""
    client = node.create_client(Trigger, service_name)
    try:
        if not client.wait_for_service(timeout_sec=1.5):
            print(f'[RECOVERY][FAIL] {service_name}: 서비스 연결 불가')
            return False
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
        if not future.done():
            print(f'[RECOVERY][FAIL] {service_name}: {timeout:.0f}초 타임아웃')
            return False
        response = future.result()
        if response is None:
            print(f'[RECOVERY][FAIL] {service_name}: 빈 응답')
            return False
        tag = 'ACK' if response.success else 'DENIED'
        print(f'[RECOVERY][{tag}] {service_name}: {response.message}')
        print('[RECOVERY] ACK는 READY 또는 안전 정지 완료를 뜻하지 않습니다.')
        return bool(response.success)
    except Exception as exc:
        print(f'[RECOVERY][ERROR] {service_name}: {exc}')
        return False
    finally:
        node.destroy_client(client)


def _print_axis_snapshot(snapshot_ref):
    snapshot = snapshot_ref.get('data')
    if not snapshot or time.monotonic() - snapshot['time'] > 2.0:
        print('[RECOVERY] 최근 2초 이내 PLC AxisStatus 없음')
        return
    for a in sorted(snapshot['axes'], key=lambda x: x.get('axis', 99)):
        print(' Axis{axis}: speed={velocity_mps} ready={ready} NM={not_moving} '
              'HasJob={has_job} FW={moving_fw} BW={moving_bw} '
              'Error={error} Code={error_code}'.format(**a))


def twincat_maintenance_menu(node, snapshot_ref, pub_wheel,
                             pub_rail_command, pub_rail_speed_setpoint,
                             agv_status=None, direct_restart=False):
    """V directly requests TwinCAT restart without PLC axis-state prechecks."""
    if direct_restart:
        # Maintenance only: TwinCAT Runtime restart is NOT an emergency stop.
        # Do not wait for velocity=0, Power OFF, bNotMoving or HasJob clearance.
        # Stop the robot and secure the work area independently before pressing V.
        print('\n[TWINCAT] V -> 즉시 Restart 요청 (모터 상태 검사 없음)')
        rc = _run_twincat_tool('restart', '--confirmed', timeout=90)
        if rc == 0:
            print('[TWINCAT][OK] Restart 명령 완료. ADS 재연결 및 PLC READY 확인 후 운전하세요.')
        else:
            print('[TWINCAT][FAIL] Restart 실패/불확실. PLC Runtime 상태를 확인하세요.')
        return
    else:
        print('\n' + '=' * 58)
        print('Beckhoff TwinCAT / AGV 오류 복구 (V)')
        print('1: TwinCAT 상태 조회 (읽기)')
        print('2: TwinCAT Runtime 재시작 (기존 엄격한 안전 조건 유지)')
        print('3: AGV/PLC 오류 및 잠금 원인 진단 (읽기)')
        print('4: 잔류 NC Job 복구 요청 (실제 정지 확인 후)')
        print('5: ADS 재연결 이후 운영자 승인')
        print('6: Reset 이후 Power 복구 승인')
        print('7: 오류 Reset 요청 (정지 확인 후)')
        print('0: 취소')
        print('=' * 58)
        choice = input('선택: ').strip()
    if choice in ('0', ''):
        return
    if choice == '1':
        _run_twincat_tool('state', timeout=12)
        return
    if choice == '3':
        if agv_status is not None:
            print(f"[RECOVERY] 현재 AGV 상태: {agv_status.get('state', 'UNKNOWN')}")
        _print_axis_snapshot(snapshot_ref)
        _call_trigger_service(node, RECOVERY_DIAG_SERVICE)
        return
    if choice in ('4', '5', '6', '7'):
        names = {'4': RECOVER_STALE_SERVICE, '5': APPROVE_ADS_SERVICE,
                 '6': APPROVE_POWER_SERVICE, '7': RESET_FAULTS_SERVICE}
        _print_axis_snapshot(snapshot_ref)
        if choice == '4':
            print('[주의] 잔류 Job 복구는 PLC에 STOP을 1회 보냅니다.')
            print('[주의] 실제 움직임과 위험 영역, 비상정지 가능 여부를 현장에서 확인하세요.')
            if input('확인 후 STALE 입력 (취소: Enter): ').strip() != 'STALE':
                print('[RECOVERY] 취소됨')
                return
        elif choice in ('5', '6', '7'):
            print('[주의] 서비스는 요청 승인/Reset일 뿐 자동 주행 명령은 보낼 수 없습니다.')
            if input('현장 확인 후 CONFIRM 입력 (취소: Enter): ').strip() != 'CONFIRM':
                print('[RECOVERY] 취소됨')
                return
        _call_trigger_service(node, names[choice])
        return
    if choice != '2':
        print('[TWINCAT] 잘못된 선택')
        return

    # This is NOT an E-stop. Do not send a PLC StopMove as a side effect
    # of opening the restart menu while the driver reports a fault latch.
    inhibited = bool(agv_status and agv_status.get('inhibited'))
    if not inhibited:
        publish_wheel_stop(pub_wheel)
        publish_rail_command(pub_rail_command, 'STOP')
        publish_rail_speed_setpoint(pub_rail_speed_setpoint, 0.0)
    # Fault-latched axes must not receive a redundant STOP.

    # Keep strict restart gating: this software has no independent STO
    # feedback. A manual text confirmation is not proof of STO activation.
    ok, reason = _refresh_axis_snapshot(node, snapshot_ref)
    if not ok:
        print(f'[TWINCAT][BLOCKED] {reason}')
        print('[TWINCAT] 모터 안전 정지·Power OFF 확인 후 재시도하세요.')
        return
    # One-key request: no interactive menu or typing. The PLC state is rechecked.
    ok, reason = _refresh_axis_snapshot(node, snapshot_ref)
    if not ok:
        print(f'[TWINCAT][BLOCKED] 재확인 실패: {reason}')
        return
    print('[TWINCAT] Restart 요청 중...')
    rc = _run_twincat_tool('restart', '--confirmed', timeout=90)
    if rc == 0:
        print('[TWINCAT][OK] Restart 요청 완료. ADS/PLC READY 확인 후 운전하세요.')
    else:
        print('[TWINCAT][FAIL] Runtime 상태를 확인하세요.')


def main():
    settings = None
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)

    rclpy.init()
    node = rclpy.create_node('teleop_keyboard_ads')
    qos = QoSProfile(depth=10)

    # Wheel path intentionally goes through twist_mux so E-stop/priority logic is preserved.
    pub_wheel = node.create_publisher(Twist, WHEEL_TELEOP_TOPIC, qos)

    # Rail command goes through bringup_node.
    # bringup_node is the only node that publishes /rail/cmd_speed.
    pub_rail_command = node.create_publisher(String, RAIL_COMMAND_TOPIC, qos)
    pub_rail_speed_setpoint = node.create_publisher(
        Float32, RAIL_SPEED_SETPOINT_TOPIC, qos)
    pub_rail_state = node.create_publisher(String, RAIL_STATE_TOPIC, qos)

    # WPT charger ON/OFF command.
    pub_charger = node.create_publisher(String, CHARGER_COMMAND_TOPIC, qos)

    # Driver status is used to revoke any stale 10Hz wheel heartbeat after a fault.
    # Only a fresh explicit READY message permits motion; all other statuses
    # (including ADS RECONNECTED / WAITING OPERATOR APPROVAL) block motion.
    agv_status = {'inhibited': True, 'clear_pending': False,
                  'last_rx': 0.0, 'state': 'WAITING FOR AGV STATUS'}
    agv_status_timeout_sec = 3.0
    last_interlock_warning_time = 0.0

    def on_agv_status(msg):
        state = msg.data.upper().strip()
        agv_status['state'] = msg.data
        agv_status['last_rx'] = time.monotonic()
        # Positively identify READY, rather than maintaining a fragile
        # blacklist of future fault/recovery state names.
        is_ready = state.startswith('AGV ADS DRIVER: READY /')
        if not is_ready and not agv_status['inhibited']:
            agv_status['clear_pending'] = True
        agv_status['inhibited'] = not is_ready

    node.create_subscription(String, '/agv_status', on_agv_status, qos)

    # Capture fresh PLC axis status for manual TwinCAT restart gating.
    axis_snapshot_ref = {'data': None}

    def on_axis_diagnostics(msg):
        try:
            value = json.loads(msg.data)
            axes = value.get('axes')
            if isinstance(axes, list):
                axis_snapshot_ref['data'] = {'time': time.monotonic(), 'axes': axes}
        except (ValueError, TypeError, AttributeError):
            axis_snapshot_ref['data'] = None

    node.create_subscription(String, '/agv/axis_diagnostics', on_axis_diagnostics, qos)


    # AGV fault reset service.
    reset_faults_client = node.create_client(Trigger, RESET_FAULTS_SERVICE)

    # Manual motor power services.
    wheel_power_client = node.create_client(SetBool, WHEEL_POWER_SERVICE)
    rail_power_client = node.create_client(SetBool, RAIL_POWER_SERVICE)

    target_linear_velocity = 0.0
    target_angular_velocity = 0.0
    target_rail_velocity = 0.0

    control_linear_velocity = 0.0
    control_angular_velocity = 0.0

    # No mutually-exclusive wheel/rail control mode: both requested targets
    # may stay active at the same time. Wheel heartbeat is needed because
    # twist_mux times out /cmd_vel_teleop after 0.5s.
    wheel_heartbeat_period_sec = 0.10  # 10Hz while wheel command is non-zero
    last_wheel_publish_mono = 0.0
    wheel_teleop_claimed = False  # Keep zero command alive after S to prevent Nav2 resuming.
    last_rail_state = ''

    show_help()

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.0)
            # A lost/stalled status publisher must never leave stale wheel
            # heartbeat commands executing. Fail closed and clear targets.
            if time.monotonic() - agv_status['last_rx'] > agv_status_timeout_sec:
                if not agv_status['inhibited']:
                    print('\n[AGV INTERLOCK] /agv_status timeout; motion blocked.')
                    agv_status['clear_pending'] = True
                agv_status['inhibited'] = True
                agv_status['state'] = 'STATUS TIMEOUT / NOT READY'
            if agv_status['clear_pending']:
                agv_status['clear_pending'] = False
                wheel_teleop_claimed = True
                target_linear_velocity = 0.0
                target_angular_velocity = 0.0
                target_rail_velocity = 0.0
                control_linear_velocity = 0.0
                control_angular_velocity = 0.0
                publish_wheel_stop(pub_wheel)
                publish_rail_command(pub_rail_command, 'STOP')
                publish_rail_speed_setpoint(pub_rail_speed_setpoint, 0.0)
                print('\n[AGV FAULT] Previous teleop targets CLEARED; fresh input required after READY.')
            key = get_key(settings)
            # Accept Caps Lock / uppercase keys without changing Ctrl+C or space.
            if len(key) == 1:
                key = key.lower()
            if agv_status['inhibited'] and key in ('w','x','a','d','t','b','1','2'):
                if time.monotonic() - last_interlock_warning_time >= 3.0:
                    print('\n[AGV] 주행 잠금: READY 대기 중')
                    last_interlock_warning_time = time.monotonic()
                key = ''

            # '?' only reprints the key guide; it does not send a motion command.
            if key == '?':
                print()
                show_help()
                continue

            # ---------------- Wheel speed: NEVER cancel rail velocity ----------------
            if key in ('w', 'x', 'a', 'd', 's'):
                wheel_teleop_claimed = True
                if key == 'w':
                    target_linear_velocity = step_linear_velocity(
                        target_linear_velocity, +LIN_VEL_STEP_SIZE)
                elif key == 'x':
                    target_linear_velocity = step_linear_velocity(
                        target_linear_velocity, -LIN_VEL_STEP_SIZE)
                elif key == 'a':
                    target_angular_velocity = step_angular_velocity(
                        target_angular_velocity, +ANG_VEL_STEP_SIZE)
                elif key == 'd':
                    target_angular_velocity = step_angular_velocity(
                        target_angular_velocity, -ANG_VEL_STEP_SIZE)
                elif key == 's':
                    target_linear_velocity = 0.0
                    target_angular_velocity = 0.0

                control_linear_velocity = target_linear_velocity
                control_angular_velocity = target_angular_velocity
                twist = Twist()
                twist.linear.x = control_linear_velocity
                twist.angular.z = control_angular_velocity
                pub_wheel.publish(twist)
                last_wheel_publish_mono = time.monotonic()
                print_vels(target_linear_velocity, target_angular_velocity,
                           target_rail_velocity)

            # ---------------- Rail speed: NEVER cancel wheel velocity ----------------
            elif key in ('t', 'b', 'g'):
                if key == 't':
                    target_rail_velocity = check_rail_limit_velocity(
                        target_rail_velocity + RAIL_VEL_STEP_SIZE)
                elif key == 'b':
                    target_rail_velocity = check_rail_limit_velocity(
                        target_rail_velocity - RAIL_VEL_STEP_SIZE)
                else:  # G: rail only STOP
                    target_rail_velocity = 0.0
                    publish_rail_command(pub_rail_command, 'STOP')
                publish_rail_speed_setpoint(pub_rail_speed_setpoint,
                                            target_rail_velocity)
                print_vels(target_linear_velocity, target_angular_velocity,
                           target_rail_velocity)

            # ---------------- Fixed rail commands: wheels remain active ----------------
            elif key in ('1', '2', '3'):
                if key == '1':
                    target_rail_velocity = +RAIL_FIXED_VEL
                    publish_rail_command(pub_rail_command, 'FORWARD')
                    publish_rail_speed_setpoint(pub_rail_speed_setpoint,
                                                +RAIL_FIXED_VEL)
                    print('\nRail command: FORWARD (+0.2 m/s)')
                elif key == '2':
                    target_rail_velocity = -RAIL_FIXED_VEL
                    publish_rail_command(pub_rail_command, 'BACK')
                    publish_rail_speed_setpoint(pub_rail_speed_setpoint,
                                                -RAIL_FIXED_VEL)
                    print('\nRail command: BACK (-0.2 m/s)')
                else:
                    target_rail_velocity = 0.0
                    publish_rail_command(pub_rail_command, 'STOP')
                    publish_rail_speed_setpoint(pub_rail_speed_setpoint, 0.0)
                    print('\nRail command: STOP (wheel command unchanged)')

            # ---------------- Explicit software stop of BOTH axes ----------------
            elif key in (' ', 'm'):
                wheel_teleop_claimed = True
                target_linear_velocity = 0.0
                target_angular_velocity = 0.0
                target_rail_velocity = 0.0
                control_linear_velocity = 0.0
                control_angular_velocity = 0.0
                publish_wheel_stop(pub_wheel)
                publish_rail_command(pub_rail_command, 'STOP')
                publish_rail_speed_setpoint(pub_rail_speed_setpoint, 0.0)
                last_wheel_publish_mono = time.monotonic()
                print('\n[STOP] Wheel + Rail STOP requested (software stop; not E-stop)')
                print_vels(0.0, 0.0, 0.0)

            # ---------------- ADS rail state ----------------
            elif key in ('4', '5', '6', '7', '8', '9'):
                state_map = {
                    '4': 'RAIL_APPROACH',
                    '5': 'OUT_RAIL',
                    '6': 'ON_RAIL',
                    '7': 'WORKING',
                    '8': 'EXITING',
                    '9': 'ENTERING',
                }
                state = state_map[key]

                # Clear stale motion before changing mechanical mode.
                wheel_teleop_claimed = True
                publish_wheel_stop(pub_wheel)
                publish_rail_command(pub_rail_command, 'STOP')
                publish_rail_speed_setpoint(pub_rail_speed_setpoint, 0.0)

                target_linear_velocity = 0.0
                target_angular_velocity = 0.0
                target_rail_velocity = 0.0
                control_linear_velocity = 0.0
                control_angular_velocity = 0.0

                if state != last_rail_state:
                    publish_rail_state(pub_rail_state, state)
                    last_rail_state = state
                    print(f'\nADS rail state -> {state}')

            # ---------------- Manual motor power ----------------
            elif key in ('e', 'f', 'y', 'z'):
                # Clear all motion commands before changing servo power.
                wheel_teleop_claimed = True
                target_linear_velocity = 0.0
                target_angular_velocity = 0.0
                target_rail_velocity = 0.0
                control_linear_velocity = 0.0
                control_angular_velocity = 0.0

                publish_wheel_stop(pub_wheel)
                publish_rail_command(pub_rail_command, 'STOP')
                publish_rail_speed_setpoint(pub_rail_speed_setpoint, 0.0)

                if key == 'e':
                    call_set_bool_service(
                        node, wheel_power_client, WHEEL_POWER_SERVICE, True)
                elif key == 'f':
                    call_set_bool_service(
                        node, wheel_power_client, WHEEL_POWER_SERVICE, False)
                elif key == 'y':
                    call_set_bool_service(
                        node, rail_power_client, RAIL_POWER_SERVICE, True)
                elif key == 'z':
                    call_set_bool_service(
                        node, rail_power_client, RAIL_POWER_SERVICE, False)

                print_vels(
                    target_linear_velocity,
                    target_angular_velocity,
                    target_rail_velocity
                )

            # ---------------- AGV error/fault reset ----------------
            elif key == 'r':
                # IMPORTANT (vendor-specific Beckhoff fault recovery):
                wheel_teleop_claimed = True
                # Clear only local teleop targets. Do NOT publish wheel/rail STOP
                # immediately before Reset: downstream PLC StopMove can stay BUSY
                # when a drive fault is present. The AGV driver must block motion
                # throughout recovery and the PLC must enforce safe stopping.
                target_linear_velocity = 0.0
                target_angular_velocity = 0.0
                target_rail_velocity = 0.0
                control_linear_velocity = 0.0
                control_angular_velocity = 0.0

                # No STOP publication here. The fault-aware AGV driver will
                # request Reset without StopMove, then verify actual standstill.
                print("\n[RESET] Local targets cleared (no STOP published). Calling /agv/reset_faults ...")

                if not reset_faults_client.wait_for_service(timeout_sec=1.0):
                    print("[RESET][ERROR] /agv/reset_faults service is not available.")
                else:
                    request = Trigger.Request()
                    future = reset_faults_client.call_async(request)

                    # The driver reset sequence can wait for physical stop/fault clear,
                    # so allow enough time for the service callback to finish.
                    rclpy.spin_until_future_complete(
                        node,
                        future,
                        timeout_sec=15.0
                    )

                    if not future.done():
                        print("[RESET][ERROR] Reset service timed out after 15 sec.")
                    else:
                        try:
                            response = future.result()
                            if response is None:
                                print("[RESET][ERROR] Empty response from reset service.")
                            elif response.success:
                                print(f"[RESET][ACK / NOT READY YET] {response.message}")
                                print("[RESET] PLC 정지 및 FB idle 3초 검증 후 자동 READY (전원 구성 정상 시).")
                                print("[RESET] Power OFF 축은 N 승인 필요. 시간 초과시 자동 TwinCAT Restart 없음.")
                            else:
                                print(f"[RESET][FAIL] {response.message}")
                        except Exception as exc:
                            print(f"[RESET][ERROR] Service call failed: {exc}")

                print_vels(
                    target_linear_velocity,
                    target_angular_velocity,
                    target_rail_velocity
                )

            # ---------------- Read-only recovery diagnostics ----------------
            elif key == 'v':
                # Direct TwinCAT restart; no motor-zero/PLC-state gating.
                wheel_teleop_claimed = True
                target_linear_velocity = 0.0
                target_angular_velocity = 0.0
                target_rail_velocity = 0.0
                control_linear_velocity = 0.0
                control_angular_velocity = 0.0
                twincat_maintenance_menu(
                    node, axis_snapshot_ref, pub_wheel,
                    pub_rail_command, pub_rail_speed_setpoint,
                    agv_status=agv_status, direct_restart=True)
                # Never restore pre-maintenance velocities. The normal 10Hz zero
                # heartbeat resumes after the menu closes.
                last_wheel_publish_mono = 0.0
                print_vels(0.0, 0.0, 0.0)

            # ---------------- Operator approval after Reset ----------------
            elif key == 'n':
                print('\n[POWER APPROVAL] 현장에서 실제 정지와 위험 구역을 확인해야 합니다.')
                if input('승인하려면 CONFIRM 입력: ').strip() == 'CONFIRM':
                    _call_trigger_service(node, APPROVE_POWER_SERVICE)
                else:
                    print('[POWER APPROVAL] 취소')

            # ---------------- WPT charger control ----------------
            elif key in ('o', 'p'):
                # Charger command does not change wheel/rail teleop mode.
                if key == 'o':
                    publish_charger_command(pub_charger, 'ON')
                    print('\nCharger command: ON')
                else:
                    publish_charger_command(pub_charger, 'OFF')
                    print('\nCharger command: OFF')

            elif key == '\x03':
                break

            # Maintain teleop priority after *either* wheel motion or a STOP.
            # If we let teleop's zero expire, twist_mux may unexpectedly resume
            # Nav2 after operator presses S. Release teleop control by exiting.
            if (wheel_teleop_claimed and
                    time.monotonic() - last_wheel_publish_mono >= wheel_heartbeat_period_sec):
                twist = Twist()
                twist.linear.x = 0.0 if agv_status['inhibited'] else target_linear_velocity
                twist.angular.z = 0.0 if agv_status['inhibited'] else target_angular_velocity
                pub_wheel.publish(twist)
                last_wheel_publish_mono = time.monotonic()

    except Exception as exc:
        print(f'\nTeleop error: {exc}')

    finally:
        # Teleop releases its velocity request, but must NOT unilaterally
        # PowerDisable axes shared with Nav2/BT. PLC shutdown belongs to
        # agv_ads_driver when the full bringup is stopped.
        if rclpy.ok():
            try:
                for _ in range(3):
                    publish_wheel_stop(pub_wheel)
                    publish_rail_command(pub_rail_command, 'STOP')
                    publish_rail_speed_setpoint(pub_rail_speed_setpoint, 0.0)
                    time.sleep(0.05)
            except Exception as exc:
                print(f'\n[TELEOP EXIT] STOP publish failed: {exc}')
        else:
            print('\n[TELEOP EXIT] ROS context invalid; STOP publish unavailable. '
                  'Confirm motor state using PLC/AGV diagnostics.')

        if os.name != 'nt' and settings is not None:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

        try:
            node.destroy_node()
        except Exception:
            pass

        if rclpy.ok():
            rclpy.shutdown()

        print('\nTeleop stopped.')


if __name__ == '__main__':
    main()