#!/usr/bin/env python3

import os
import select
import sys

import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, String
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
RAIL_VEL_STEP_SIZE = 0.1
RAIL_FIXED_VEL = 0.2

WHEEL_TELEOP_TOPIC = '/cmd_vel_teleop'
RAIL_SPEED_TOPIC = '/rail/cmd_speed'
RAIL_STATE_TOPIC = '/rail_state'

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

ADS rail state
--------------
4 : RAIL_APPROACH
5 : OUT_RAIL
6 : ON_RAIL
7 : WORKING
8 : EXITING
9 : ENTERING

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


def publish_rail_speed(pub, speed):
    msg = Float32()
    msg.data = float(speed)
    pub.publish(msg)


def publish_rail_state(pub, state):
    msg = String()
    msg.data = state
    pub.publish(msg)


def main():
    settings = None
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)

    rclpy.init()
    node = rclpy.create_node('teleop_keyboard_ads')
    qos = QoSProfile(depth=10)

    # Wheel path intentionally goes through twist_mux so E-stop/priority logic is preserved.
    pub_wheel = node.create_publisher(Twist, WHEEL_TELEOP_TOPIC, qos)

    # Direct ADS-driver interfaces for rail.
    pub_rail_speed = node.create_publisher(Float32, RAIL_SPEED_TOPIC, qos)
    pub_rail_state = node.create_publisher(String, RAIL_STATE_TOPIC, qos)

    target_linear_velocity = 0.0
    target_angular_velocity = 0.0
    target_rail_velocity = 0.0

    control_linear_velocity = 0.0
    control_angular_velocity = 0.0

    current_control_mode = 'none'  # wheel / rail / none
    last_rail_state = ''

    print(HELP)

    try:
        while rclpy.ok():
            key = get_key(settings)

            # ---------------- Wheel control ----------------
            if key in ('w', 'x', 'a', 'd', ' ', 's'):
                if current_control_mode != 'wheel':
                    target_rail_velocity = 0.0
                    publish_rail_speed(pub_rail_speed, 0.0)

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

            # ---------------- Adjustable rail speed ----------------
            elif key in ('t', 'b', 'g'):
                if current_control_mode != 'rail':
                    target_linear_velocity = 0.0
                    target_angular_velocity = 0.0
                    control_linear_velocity = 0.0
                    control_angular_velocity = 0.0
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

                print_vels(
                    target_linear_velocity,
                    target_angular_velocity,
                    target_rail_velocity)

            # ---------------- Fixed rail command ----------------
            elif key in ('1', '2', '3'):
                if current_control_mode != 'rail':
                    target_linear_velocity = 0.0
                    target_angular_velocity = 0.0
                    control_linear_velocity = 0.0
                    control_angular_velocity = 0.0
                    publish_wheel_stop(pub_wheel)

                current_control_mode = 'rail'

                if key == '1':
                    target_rail_velocity = +RAIL_FIXED_VEL
                    print('\nRail command: FORWARD (+0.2 m/s)')
                elif key == '2':
                    target_rail_velocity = -RAIL_FIXED_VEL
                    print('\nRail command: BACK (-0.2 m/s)')
                else:
                    target_rail_velocity = 0.0
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
                publish_rail_speed(pub_rail_speed, 0.0)

                target_linear_velocity = 0.0
                target_angular_velocity = 0.0
                target_rail_velocity = 0.0
                control_linear_velocity = 0.0
                control_angular_velocity = 0.0
                current_control_mode = 'none'

                if state != last_rail_state:
                    publish_rail_state(pub_rail_state, state)
                    last_rail_state = state
                    print(f'\nADS rail state -> {state}')

            elif key == '\x03':
                break

            # ---------------- Heartbeat publishing ----------------
            # get_key() loops every ~0.1 s, so this continuously feeds watchdogs.
            if current_control_mode == 'wheel':
                control_linear_velocity = make_simple_profile(
                    control_linear_velocity,
                    target_linear_velocity,
                    LIN_VEL_STEP_SIZE / 2.0)

                control_angular_velocity = make_simple_profile(
                    control_angular_velocity,
                    target_angular_velocity,
                    ANG_VEL_STEP_SIZE / 2.0)

                twist = Twist()
                twist.linear.x = control_linear_velocity
                twist.angular.z = control_angular_velocity
                pub_wheel.publish(twist)

            elif current_control_mode == 'rail':
                publish_rail_speed(pub_rail_speed, target_rail_velocity)

    except Exception as exc:
        print(f'\nTeleop error: {exc}')

    finally:
        try:
            publish_wheel_stop(pub_wheel)
            publish_rail_speed(pub_rail_speed, 0.0)
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