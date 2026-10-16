#!/usr/bin/env python3

import os
import select
import sys

import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float32, String
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

LIN_VEL_STEP_SIZE = 0.01
ANG_VEL_STEP_SIZE = 0.01
INDIVIDUAL_WHEEL_STEP_SIZE = 0.01
WHEEL_SEPARATION = 1.0  # Must match agv_node wheel_separation
RAIL_VEL_STEP_SIZE = 0.1
RAIL_FIXED_VEL = 0.2

WHEEL_TELEOP_TOPIC = '/cmd_vel_teleop'
RAIL_COMMAND_TOPIC = '/rail_command'
RAIL_SPEED_SETPOINT_TOPIC = '/rail_speed_setpoint'
RAIL_STATE_TOPIC = '/rail_state'
CHARGER_COMMAND_TOPIC = '/charger/command'
RESET_FAULTS_SERVICE = '/agv/reset_faults'
WHEEL_POWER_SERVICE = '/agv/wheel_power'
RAIL_POWER_SERVICE = '/agv/rail_power'

HELP = """
Control Your robot - ADS version
================================

Ground wheel control
--------------------
        w
   a    s    d
        x

w/x : increase/decrease linear velocity
a/d : increase/decrease angular velocity
space, s : wheel STOP

Individual wheel test
---------------------
u / j : LEFT wheel  +0.01 / -0.01 m/s
i / k : RIGHT wheel +0.01 / -0.01 m/s
h     : LEFT wheel STOP
l     : RIGHT wheel STOP
m     : BOTH wheels STOP

NOTE:
Individual wheel commands are converted to /cmd_vel_teleop.
WHEEL_SEPARATION must match agv_node.

Rail speed control
------------------
t : increase rail speed by +0.1 m/s
b : decrease rail speed by -0.1 m/s
g : rail STOP

Fixed rail commands
-------------------
1 : rail FORWARD  +0.2 m/s
2 : rail BACK     -0.2 m/s
3 : rail STOP      0.0 m/s

NOTE:
t/b/g publish a signed speed setpoint through bringup_node.
1/2/3 publish FORWARD/BACK/STOP commands.

ADS rail state
--------------
4 : RAIL_APPROACH
5 : OUT_RAIL
6 : ON_RAIL
7 : WORKING
8 : EXITING
9 : ENTERING

Charger control
---------------
o : Charger ON
p : Charger OFF

AGV error reset
---------------
r : Reset AGV faults (/agv/reset_faults)

Motor power control
-------------------
e : Wheel motors ON
f : Wheel motors OFF
y : Rail motor ON
z : Rail motor OFF

IMPORTANT
---------
OUT_RAIL / MOVING_TO_RAIL / RAIL_APPROACH / ENTERING:
    Wheels ENABLE, Rail DISABLE

ON_RAIL / WORKING / EXITING:
    Wheels DISABLE, Rail ENABLE

CTRL-C to quit
"""


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


def wheel_speeds_to_twist(left_mps, right_mps):
    # agv_node:
    #   left  = linear - angular * wheel_separation / 2
    #   right = linear + angular * wheel_separation / 2
    # Therefore:
    #   linear  = (left + right) / 2
    #   angular = (right - left) / wheel_separation
    twist = Twist()
    twist.linear.x = (left_mps + right_mps) / 2.0
    twist.angular.z = (right_mps - left_mps) / WHEEL_SEPARATION
    return twist


def print_individual_wheels(left_mps, right_mps):
    linear = (left_mps + right_mps) / 2.0
    angular = (right_mps - left_mps) / WHEEL_SEPARATION
    print(
        f'\rLEFT={left_mps:+.2f} m/s  RIGHT={right_mps:+.2f} m/s  '
        f'-> linear={linear:+.2f}  angular={angular:+.2f}      ',
        end='',
        flush=True
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


def publish_charger_command(pub, enabled):
    msg = Bool()
    msg.data = bool(enabled)
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
    pub_charger = node.create_publisher(Bool, CHARGER_COMMAND_TOPIC, qos)

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

    target_left_wheel_velocity = 0.0
    target_right_wheel_velocity = 0.0

    current_control_mode = 'none'  # wheel / individual_wheel / rail / none
    last_rail_state = ''

    print(HELP)

    try:
        while rclpy.ok():
            key = get_key(settings)

            # ---------------- Wheel control ----------------
            if key in ('w', 'x', 'a', 'd', ' ', 's'):
                if current_control_mode != 'wheel':
                    target_rail_velocity = 0.0
                    publish_rail_command(pub_rail_command, 'STOP')
                    publish_rail_speed_setpoint(pub_rail_speed_setpoint, 0.0)

                if current_control_mode != 'wheel':
                    target_left_wheel_velocity = 0.0
                    target_right_wheel_velocity = 0.0

                current_control_mode = 'wheel'

                if key == 'w':
                    target_linear_velocity = check_linear_limit_velocity(
                        target_linear_velocity + LIN_VEL_STEP_SIZE)
                elif key == 'x':
                    target_linear_velocity = check_linear_limit_velocity(
                        target_linear_velocity - LIN_VEL_STEP_SIZE)
                elif key == 'a':
                    target_angular_velocity = check_angular_limit_velocity(
                        target_angular_velocity + ANG_VEL_STEP_SIZE)
                elif key == 'd':
                    target_angular_velocity = check_angular_limit_velocity(
                        target_angular_velocity - ANG_VEL_STEP_SIZE)
                elif key in (' ', 's'):
                    target_linear_velocity = 0.0
                    target_angular_velocity = 0.0
                    control_linear_velocity = 0.0
                    control_angular_velocity = 0.0

                print_vels(
                    target_linear_velocity,
                    target_angular_velocity,
                    target_rail_velocity)

            # ---------------- Individual wheel control ----------------
            elif key in ('u', 'j', 'i', 'k', 'h', 'l', 'm'):
                if current_control_mode != 'individual_wheel':
                    target_rail_velocity = 0.0
                    publish_rail_command(pub_rail_command, 'STOP')
                    publish_rail_speed_setpoint(pub_rail_speed_setpoint, 0.0)

                    # Enter individual-wheel mode from a known safe zero command.
                    target_linear_velocity = 0.0
                    target_angular_velocity = 0.0
                    control_linear_velocity = 0.0
                    control_angular_velocity = 0.0
                    target_left_wheel_velocity = 0.0
                    target_right_wheel_velocity = 0.0
                    publish_wheel_stop(pub_wheel)

                current_control_mode = 'individual_wheel'

                if key == 'u':
                    target_left_wheel_velocity = check_linear_limit_velocity(
                        target_left_wheel_velocity + INDIVIDUAL_WHEEL_STEP_SIZE)
                elif key == 'j':
                    target_left_wheel_velocity = check_linear_limit_velocity(
                        target_left_wheel_velocity - INDIVIDUAL_WHEEL_STEP_SIZE)
                elif key == 'i':
                    target_right_wheel_velocity = check_linear_limit_velocity(
                        target_right_wheel_velocity + INDIVIDUAL_WHEEL_STEP_SIZE)
                elif key == 'k':
                    target_right_wheel_velocity = check_linear_limit_velocity(
                        target_right_wheel_velocity - INDIVIDUAL_WHEEL_STEP_SIZE)
                elif key == 'h':
                    target_left_wheel_velocity = 0.0
                elif key == 'l':
                    target_right_wheel_velocity = 0.0
                elif key == 'm':
                    target_left_wheel_velocity = 0.0
                    target_right_wheel_velocity = 0.0

                print_individual_wheels(
                    target_left_wheel_velocity,
                    target_right_wheel_velocity
                )

            # ---------------- Adjustable rail speed via bringup_node ----------------
            elif key in ('t', 'b', 'g'):
                if current_control_mode != 'rail':
                    target_linear_velocity = 0.0
                    target_angular_velocity = 0.0
                    control_linear_velocity = 0.0
                    control_angular_velocity = 0.0
                    target_left_wheel_velocity = 0.0
                    target_right_wheel_velocity = 0.0
                    publish_wheel_stop(pub_wheel)

                current_control_mode = 'rail'

                if key == 't':
                    target_rail_velocity = check_rail_limit_velocity(
                        target_rail_velocity + RAIL_VEL_STEP_SIZE)
                elif key == 'b':
                    target_rail_velocity = check_rail_limit_velocity(
                        target_rail_velocity - RAIL_VEL_STEP_SIZE)
                elif key == 'g':
                    target_rail_velocity = 0.0

                publish_rail_speed_setpoint(
                    pub_rail_speed_setpoint,
                    target_rail_velocity
                )

                print_vels(
                    target_linear_velocity,
                    target_angular_velocity,
                    target_rail_velocity
                )

            # ---------------- Rail command via bringup_node ----------------
            elif key in ('1', '2', '3'):
                if current_control_mode != 'rail':
                    target_linear_velocity = 0.0
                    target_angular_velocity = 0.0
                    control_linear_velocity = 0.0
                    control_angular_velocity = 0.0
                    target_left_wheel_velocity = 0.0
                    target_right_wheel_velocity = 0.0
                    publish_wheel_stop(pub_wheel)

                current_control_mode = 'rail'

                if key == '1':
                    target_rail_velocity = +RAIL_FIXED_VEL
                    publish_rail_command(pub_rail_command, 'FORWARD')
                    publish_rail_speed_setpoint(
                        pub_rail_speed_setpoint, +RAIL_FIXED_VEL)
                    print('\nRail command: FORWARD (+0.2 m/s)')
                elif key == '2':
                    target_rail_velocity = -RAIL_FIXED_VEL
                    publish_rail_command(pub_rail_command, 'BACK')
                    publish_rail_speed_setpoint(
                        pub_rail_speed_setpoint, -RAIL_FIXED_VEL)
                    print('\nRail command: BACK (-0.2 m/s)')
                else:
                    target_rail_velocity = 0.0
                    publish_rail_command(pub_rail_command, 'STOP')
                    publish_rail_speed_setpoint(
                        pub_rail_speed_setpoint, 0.0)
                    print('\nRail command: STOP')

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
                publish_wheel_stop(pub_wheel)
                publish_rail_command(pub_rail_command, 'STOP')
                publish_rail_speed_setpoint(pub_rail_speed_setpoint, 0.0)

                target_linear_velocity = 0.0
                target_angular_velocity = 0.0
                target_rail_velocity = 0.0
                control_linear_velocity = 0.0
                control_angular_velocity = 0.0
                target_left_wheel_velocity = 0.0
                target_right_wheel_velocity = 0.0
                current_control_mode = 'none'

                if state != last_rail_state:
                    publish_rail_state(pub_rail_state, state)
                    last_rail_state = state
                    print(f'\nADS rail state -> {state}')

            # ---------------- Manual motor power ----------------
            elif key in ('e', 'f', 'y', 'z'):
                # Clear all motion commands before changing servo power.
                target_linear_velocity = 0.0
                target_angular_velocity = 0.0
                target_rail_velocity = 0.0
                control_linear_velocity = 0.0
                control_angular_velocity = 0.0
                target_left_wheel_velocity = 0.0
                target_right_wheel_velocity = 0.0
                current_control_mode = 'none'

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
                # Safety: clear all local motion targets and publish STOP first.
                target_linear_velocity = 0.0
                target_angular_velocity = 0.0
                target_rail_velocity = 0.0
                control_linear_velocity = 0.0
                control_angular_velocity = 0.0
                target_left_wheel_velocity = 0.0
                target_right_wheel_velocity = 0.0
                current_control_mode = 'none'

                publish_wheel_stop(pub_wheel)
                publish_rail_command(pub_rail_command, 'STOP')
                publish_rail_speed_setpoint(pub_rail_speed_setpoint, 0.0)

                print("\n[RESET] Wheel/Rail STOP sent. Calling /agv/reset_faults ...")

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
                                print(f"[RESET][OK] {response.message}")
                            else:
                                print(f"[RESET][FAIL] {response.message}")
                        except Exception as exc:
                            print(f"[RESET][ERROR] Service call failed: {exc}")

                print_vels(
                    target_linear_velocity,
                    target_angular_velocity,
                    target_rail_velocity
                )

            # ---------------- WPT charger control ----------------
            elif key in ('o', 'p'):
                # Charger command does not change wheel/rail teleop mode.
                if key == 'o':
                    publish_charger_command(pub_charger, True)
                    print('\nCharger command: ON')
                else:
                    publish_charger_command(pub_charger, False)
                    print('\nCharger command: OFF')

            elif key == '\x03':
                break

            # ---------------- One-shot motion command ----------------
            # Wheel/rail velocity is sent only when a control key is pressed.
            # No periodic velocity publishing is performed.
            if key in ('w', 'x', 'a', 'd', ' ', 's'):
                control_linear_velocity = target_linear_velocity
                control_angular_velocity = target_angular_velocity

                twist = Twist()
                twist.linear.x = control_linear_velocity
                twist.angular.z = control_angular_velocity
                pub_wheel.publish(twist)

            elif key in ('u', 'j', 'i', 'k', 'h', 'l', 'm'):
                twist = wheel_speeds_to_twist(
                    target_left_wheel_velocity,
                    target_right_wheel_velocity
                )
                pub_wheel.publish(twist)

            # Rail 1/2/3 commands are already published above via /rail_command.

    except Exception as exc:
        print(f'\nTeleop error: {exc}')

    finally:
        try:
            publish_wheel_stop(pub_wheel)
            publish_rail_command(pub_rail_command, 'STOP')
            publish_rail_speed_setpoint(pub_rail_speed_setpoint, 0.0)
        except Exception:
            pass

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