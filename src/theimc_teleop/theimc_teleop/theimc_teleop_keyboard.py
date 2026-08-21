#!/usr/bin/env python
#
# Copyright (c) 2011, Willow Garage, Inc.
# All rights reserved.
#
# (라이선스 생략)
#
# Author: Darby Lim

import os
import select
import sys
import rclpy

from geometry_msgs.msg import Twist
from std_msgs.msg import String  # String 메시지 타입 추가
from rclpy.qos import QoSProfile

if os.name == 'nt':
    import msvcrt # windows
else:
    import termios
    import tty

MAX_LIN_VEL = 1.0
MAX_ANG_VEL = 1.0
MAX_RAIL_VEL = 0.5

LIN_VEL_STEP_SIZE = 0.01
ANG_VEL_STEP_SIZE = 0.01
RAIL_VEL_STEP_SIZE = 0.1

msg = """
Control Your robot !
---------------------------
Moving around:
        w
   a    s    d
        x

w/x : increase/decrease linear velocity 
a/d : increase/decrease angular velocity 

space key, s : force main motor stop

Rail Motor Control (Step: 0.1):
        t
        g
        b

t/b : increase/decrease rail velocity
g   : force stop rail motor

Rail String Commands:
        1 : FORWARD
        2 : BACK
        3 : STOP
        4 : DETECTED
        5 : OUT
        6 : ON

CTRL-C to quit
"""

e = """
Communications Failed
"""

def get_key(settings):
    if os.name == 'nt':
        return msvcrt.getch().decode('utf-8')
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def print_vels(target_linear_velocity, target_angular_velocity, target_rail_velocity):
    print('currently:\tlinear vel {0:.2f}\t angular vel {1:.2f}\t rail vel {2:.1f}'.format(
        target_linear_velocity,
        target_angular_velocity,
        target_rail_velocity))

def make_simple_profile(output, input, slop):
    if input > output:
        output = min(input, output + slop)
    elif input < output:
        output = max(input, output - slop)
    else:
        output = input

    return output

def constrain(input_vel, low_bound, high_bound):
    if input_vel < low_bound:
        input_vel = low_bound
    elif input_vel > high_bound:
        input_vel = high_bound
    else:
        input_vel = input_vel

    return input_vel

def check_linear_limit_velocity(velocity):
    return constrain(velocity, -MAX_LIN_VEL, MAX_LIN_VEL)

def check_angular_limit_velocity(velocity):
    return constrain(velocity, -MAX_ANG_VEL, MAX_ANG_VEL)

def check_rail_limit_velocity(velocity):
    return constrain(velocity, -MAX_RAIL_VEL, MAX_RAIL_VEL)

def main():
    settings = None
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)

    rclpy.init()

    qos = QoSProfile(depth=10)
    node = rclpy.create_node('teleop_keyboard')
    
    # 퍼블리셔 선언
    pub = node.create_publisher(Twist, 'cmd_vel_teleop', qos)
    pub_cmd_rail = node.create_publisher(Twist, 'cmd_rail', qos)
    pub_rail_cmd = node.create_publisher(String, 'rail_command', qos) # String 퍼블리셔 추가

    status = 0
    target_linear_velocity = 0.0
    target_angular_velocity = 0.0
    target_rail_velocity = 0.0

    control_linear_velocity = 0.0
    control_angular_velocity = 0.0
    control_rail_velocity = 0.0

    current_control_mode = 'none' # 현재 제어 상태 ('cmd' 또는 'rail')

    # [추가] 레일 명령 중복 퍼블리시 방지용 변수
    last_pub_rail_vel = -999.0
    last_rail_cmd = ""

    try:
        print(msg)
        while(1):
            key = get_key(settings)

            # 키 입력에 따라 전송 모드 분리
            if key in ['w', 'x', 'a', 'd', ' ', 's']:
                current_control_mode = 'cmd'
                last_rail_cmd = ""  # 주행 시 이전 String 명령 초기화 (재입력 가능하게 함)
            elif key in ['t', 'b', 'g']:
                current_control_mode = 'rail'
                last_rail_cmd = ""  # 주행 시 이전 String 명령 초기화 

            # ---------------- 주행 명령 ----------------
            if key == 'w':
                target_linear_velocity = check_linear_limit_velocity(target_linear_velocity + LIN_VEL_STEP_SIZE)
                status = status + 1
                print_vels(target_linear_velocity, target_angular_velocity, target_rail_velocity)
            elif key == 'x':
                target_linear_velocity = check_linear_limit_velocity(target_linear_velocity - LIN_VEL_STEP_SIZE)
                status = status + 1
                print_vels(target_linear_velocity, target_angular_velocity, target_rail_velocity)
            elif key == 'a':
                target_angular_velocity = check_angular_limit_velocity(target_angular_velocity + ANG_VEL_STEP_SIZE)
                status = status + 1
                print_vels(target_linear_velocity, target_angular_velocity, target_rail_velocity)
            elif key == 'd':
                target_angular_velocity = check_angular_limit_velocity(target_angular_velocity - ANG_VEL_STEP_SIZE)
                status = status + 1
                print_vels(target_linear_velocity, target_angular_velocity, target_rail_velocity)
            elif key == ' ' or key == 's':
                target_linear_velocity = 0.0
                control_linear_velocity = 0.0
                target_angular_velocity = 0.0
                control_angular_velocity = 0.0
                print_vels(target_linear_velocity, target_angular_velocity, target_rail_velocity)

            # ---------------- 레일 모터 제어 명령 ----------------
            elif key == 't':
                target_rail_velocity = check_rail_limit_velocity(target_rail_velocity + RAIL_VEL_STEP_SIZE)
                status = status + 1
                print_vels(target_linear_velocity, target_angular_velocity, target_rail_velocity)
            elif key == 'b':
                target_rail_velocity = check_rail_limit_velocity(target_rail_velocity - RAIL_VEL_STEP_SIZE)
                status = status + 1
                print_vels(target_linear_velocity, target_angular_velocity, target_rail_velocity)
            elif key == 'g':
                target_rail_velocity = 0.0
                control_rail_velocity = 0.0
                print_vels(target_linear_velocity, target_angular_velocity, target_rail_velocity)

            # ---------------- 레일 상태 제어 (String) 명령 ----------------
            elif key in ['1', '2', '3', '4', '5', '6']:
                rail_msg = String()
                if key == '1':
                    rail_msg.data = 'FORWARD'
                elif key == '2':
                    rail_msg.data = 'BACK'
                elif key == '3':
                    rail_msg.data = 'STOP'
                elif key == '4':
                    rail_msg.data = 'DETECTED'
                elif key == '5':
                    rail_msg.data = 'OUT'
                elif key == '6':
                    rail_msg.data = 'ON'
                
                # [수정] DETECTED 이거나 이전 명령과 다를 때만 퍼블리시
                if rail_msg.data == 'DETECTED' or rail_msg.data != last_rail_cmd:
                    pub_rail_cmd.publish(rail_msg)
                    print(f'Sent Rail String Command: [{rail_msg.data}]')
                    last_rail_cmd = rail_msg.data  # 전송 후 현재 명령 저장
                
                status = status + 1

            else:
                if (key == '\x03'): # '\x03' = ctl+c
                    break

            if status == 20:
                print(msg)
                status = 0

            # 모드에 따라 Twist 전송 분리 발행
            if current_control_mode == 'cmd':
                twist = Twist()

                control_linear_velocity = make_simple_profile(
                    control_linear_velocity,
                    target_linear_velocity,
                    (LIN_VEL_STEP_SIZE / 2.0))

                twist.linear.x = control_linear_velocity
                twist.linear.y = 0.0
                twist.linear.z = 0.0

                control_angular_velocity = make_simple_profile(
                    control_angular_velocity,
                    target_angular_velocity,
                    (ANG_VEL_STEP_SIZE / 2.0))

                twist.angular.x = 0.0
                twist.angular.y = 0.0
                twist.angular.z = control_angular_velocity

                pub.publish(twist)

            elif current_control_mode == 'rail':
                twist_rail = Twist()
                
                control_rail_velocity = make_simple_profile(
                    control_rail_velocity,
                    target_rail_velocity,
                    (RAIL_VEL_STEP_SIZE / 2.0))
                
    
                # 속도값이 이전과 달라졌을 때만 퍼블리시
                if control_rail_velocity != last_pub_rail_vel:
                    twist_rail.linear.x = control_rail_velocity
                    twist_rail.linear.y = 0.0
                    twist_rail.linear.z = 0.0

                    twist_rail.angular.x = 0.0
                    twist_rail.angular.y = 0.0
                    twist_rail.angular.z = 0.0
                    
                    pub_cmd_rail.publish(twist_rail)
                    last_pub_rail_vel = control_rail_velocity  # 전송 후 현재 속도 저장


    except Exception as e:
        print(e)

    finally:
        # 종료 시 모든 구동계 0으로 정지
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        pub.publish(twist)

        rail_stop_msg = String()
        rail_stop_msg.data = 'STOP'
        pub_rail_cmd.publish(rail_stop_msg)

        if os.name != 'nt':
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

if __name__ == '__main__':
    main()