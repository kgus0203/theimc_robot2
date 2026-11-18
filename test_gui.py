#!/usr/bin/env python3
"""AGV monitoring and dead-man control GUI for the supplied ROS 2 bringup/ADS driver.

The GUI is NOT a functional-safety device or an emergency-stop replacement.
Keep the physical E-stop, independent PLC interlocks and PLC command-loss watchdog.
"""
import json
import math
import sys
import time
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool, Float32, Float32MultiArray, String
from std_srvs.srv import Trigger, SetBool

from PyQt5.QtCore import Qt, QThread, QTimer, pyqtSignal
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QLabel, QPushButton, QVBoxLayout,
    QHBoxLayout, QGridLayout, QGroupBox, QTabWidget, QScrollArea, QDoubleSpinBox,
    QTextEdit, QMessageBox, QCheckBox, QSizePolicy
)


class RosBridge(QThread):
    measurement = pyqtSignal(str, object)
    service_reply = pyqtSignal(str, bool, str)
    worker_error = pyqtSignal(str)

    # Name -> ROS type; these exactly match uploaded bringup/driver files.
    SUBSCRIPTIONS = {
        '/agv_status': String,
        '/agv/axis_diagnostics': String,
        '/rail_state': String,
        '/rail/tof_state': String,
        '/rail_command_state': String,
        '/rail/speed_feedback': Float32MultiArray,
        '/rail/cmd_speed': Float32,
        '/rail/odom': Odometry,
        '/rail_obstacle_front': Bool,
        '/rail_obstacle_rear': Bool,
        '/rail_end_reached': Bool,
        '/tof_distance': Float32,
        '/wheel/speed_feedback': Float32MultiArray,
        '/wheel/odom': Odometry,
        '/battery/state': BatteryState,
        '/battery/soh': Float32,
        '/battery/error_code': String,
        '/battery/status': String,
        '/charger/state': String,
        '/charger/voltage': Float32,
        '/charger/current': Float32,
        '/charger/set_voltage': Float32,
        '/charger/set_current': Float32,
        '/charger/ready': Bool,
        '/charger/contact': Bool,
        '/charger/charging': Bool,
        '/charger/completed': Bool,
        '/charger/error': Bool,
        '/charger/error_code': String,
        '/charger/ack': String,
        '/environment/data': Float32MultiArray,
    }

    def __init__(self):
        super().__init__()
        self.node = Node('agv_control_dashboard')
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.subs = []
        for topic, msg_type in self.SUBSCRIPTIONS.items():
            self.subs.append(self.node.create_subscription(
                msg_type, topic, lambda msg, t=topic: self.on_message(t, msg), 10))
        self.wheel_pub = self.node.create_publisher(Twist, '/cmd_vel', 10)
        self.rail_pub = self.node.create_publisher(String, '/rail_command', 10)
        self.rail_speed_pub = self.node.create_publisher(Float32, '/rail_speed_setpoint', 10)
        self.charger_pub = self.node.create_publisher(String, '/charger/command', 10)
        self.services = {
            '/agv/reset_faults': self.node.create_client(Trigger, '/agv/reset_faults'),
            '/agv/approve_power_restore': self.node.create_client(Trigger, '/agv/approve_power_restore'),
            '/agv/reconnect_ads': self.node.create_client(Trigger, '/agv/reconnect_ads'),
            '/agv/recover_stale_jobs': self.node.create_client(Trigger, '/agv/recover_stale_jobs'),
            '/agv/wheel_power': self.node.create_client(SetBool, '/agv/wheel_power'),
            '/agv/rail_power': self.node.create_client(SetBool, '/agv/rail_power'),
        }
        self.running = True

    def on_message(self, topic, msg):
        try:
            if topic == '/agv/axis_diagnostics':
                payload = json.loads(msg.data)
            elif topic == '/battery/state':
                payload = {'voltage': msg.voltage, 'current': msg.current,
                           'percentage': msg.percentage, 'temperature': msg.temperature,
                           'power_supply_status': msg.power_supply_status}
            elif isinstance(msg, Odometry):
                payload = {'x': msg.pose.pose.position.x, 'y': msg.pose.pose.position.y,
                           'velocity': msg.twist.twist.linear.x,
                           'angular': msg.twist.twist.angular.z}
            elif hasattr(msg, 'data'):
                payload = list(msg.data) if isinstance(msg.data, (list, tuple)) else msg.data
            else:
                return
            self.measurement.emit(topic, payload)
        except Exception as error:
            self.worker_error.emit(f'{topic}: {error}')

    def publish_wheel(self, linear=0.0, angular=0.0):
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self.wheel_pub.publish(msg)

    def publish_rail_speed(self, speed):
        msg = Float32()
        msg.data = float(speed)
        self.rail_speed_pub.publish(msg)

    def publish_rail_stop(self):
        self.publish_rail_speed(0.0)
        msg = String()
        msg.data = 'STOP'
        self.rail_pub.publish(msg)

    def publish_charger(self, command):
        msg = String()
        msg.data = command
        self.charger_pub.publish(msg)

    def request(self, name, value=None):
        client = self.services[name]
        if not client.service_is_ready():
            self.service_reply.emit(name, False, '서비스 연결되지 않음')
            return
        req = Trigger.Request() if value is None else SetBool.Request()
        if value is not None:
            req.data = bool(value)
        try:
            future = client.call_async(req)
            future.add_done_callback(lambda f, n=name: self.on_service_reply(n, f))
        except Exception as error:
            self.service_reply.emit(name, False, str(error))

    def on_service_reply(self, name, future):
        try:
            result = future.result()
            self.service_reply.emit(name, bool(result.success), result.message)
        except Exception as error:
            self.service_reply.emit(name, False, str(error))

    def run(self):
        while self.running and rclpy.ok():
            try:
                self.executor.spin_once(timeout_sec=0.10)
            except Exception as error:
                self.worker_error.emit(str(error))
                break

    def stop(self):
        self.running = False

    def cleanup(self):
        self.executor.remove_node(self.node)
        self.executor.shutdown()
        self.node.destroy_node()


def field(label, value='--'):
    k = QLabel(label)
    k.setObjectName('fieldName')
    v = QLabel(value)
    v.setObjectName('fieldValue')
    v.setWordWrap(True)
    return k, v


def group(title):
    g = QGroupBox(title)
    layout = QGridLayout(g)
    layout.setVerticalSpacing(10)
    return g, layout


class Dashboard(QMainWindow):
    def __init__(self, bridge):
        super().__init__()
        self.bridge = bridge
        self.data = {}
        self.stamps = {}
        self.values = {}
        self.hold = None
        self.last_status = None
        self.events = deque(maxlen=200)
        self.setWindowTitle('AGV | Beckhoff · Rail · BMU · WPT')
        self.resize(1180, 850)
        self.build_ui()
        self.bridge.measurement.connect(self.on_measurement)
        self.bridge.service_reply.connect(self.on_service_reply)
        self.bridge.worker_error.connect(self.append_log)
        self.tick = QTimer(self)
        self.tick.timeout.connect(self.refresh)
        self.tick.start(100)
        self.append_log('모니터링 시작. GUI STOP은 물리적 비상정지가 아닙니다.')

    def add_fields(self, layout, names):
        for row, (key, title) in enumerate(names):
            k, v = field(title)
            layout.addWidget(k, row, 0)
            layout.addWidget(v, row, 1)
            self.values[key] = v

    def add_button(self, layout, text, callback, danger=False):
        b = QPushButton(text)
        if danger:
            b.setProperty('danger', True)
        b.clicked.connect(callback)
        layout.addWidget(b)
        return b

    def build_ui(self):
        self.setStyleSheet('''
            QWidget { background: #101826; color: #e9eff9; font-size: 13px; }
            QGroupBox { border: 1px solid #344257; border-radius: 9px; margin-top: 14px;
                        padding: 11px; font-weight: 700; }
            QGroupBox::title { subcontrol-origin: margin; left: 13px; padding: 0 5px; }
            QLabel#fieldName { color: #a7b6cd; min-width: 135px; }
            QLabel#fieldValue { font-weight: 650; }
            QPushButton { background: #283e59; border: 1px solid #4b6686; border-radius: 7px;
                          min-height: 36px; padding: 4px 12px; font-weight: 700; }
            QPushButton:hover { background: #3d5976; }
            QPushButton:pressed { background: #567698; }
            QPushButton[danger="true"] { background: #963b3b; border-color: #d85e5e; }
            QPushButton:disabled { color: #7b8798; background: #242b37; }
            QDoubleSpinBox { padding: 5px; color: white; background: #202e42; }
            QTextEdit { background: #0c1420; }
            QTabWidget::pane { border: 1px solid #344257; }
            QTabBar::tab { padding: 10px 18px; background: #243449; }
            QTabBar::tab:selected { background: #436185; }
            QCheckBox { padding: 5px; }
        ''')
        root = QWidget()
        self.setCentralWidget(root)
        main = QVBoxLayout(root)
        main.setSpacing(8)
        banner = QHBoxLayout()
        self.top_status = QLabel('AGV 상태 수신 대기')
        self.top_status.setStyleSheet('font-size: 18px; font-weight: 800;')
        self.top_fresh = QLabel('데이터 연결 확인 중')
        banner.addWidget(self.top_status, 1)
        banner.addWidget(self.top_fresh)
        main.addLayout(banner)
        self.tabs = QTabWidget()
        main.addWidget(self.tabs, 1)
        self.make_monitor_tab()
        self.make_control_tab()
        self.make_energy_tab()
        self.make_logs_tab()
        self.statusBar().showMessage('PLC/ADS 상태를 반드시 현장에서 확인하세요.')

    def tab_layout(self, name):
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        page = QWidget()
        layout = QVBoxLayout(page)
        layout.setContentsMargins(15, 15, 15, 15)
        layout.setSpacing(12)
        scroll.setWidget(page)
        self.tabs.addTab(scroll, name)
        return layout

    def make_monitor_tab(self):
        layout = self.tab_layout('모터 · 레일 상태')
        system, l = group('AGV 드라이버')
        self.add_fields(l, [('agv', 'ADS/모션 상태'), ('source_age', '상태 데이터 수신'),
                            ('wheel', '휠 실측 속도 (L/R)'), ('wheel_odom', '휠 Odometry'),
                            ('rail_fb', '레일 실제 속도'), ('rail_cmd', '레일 적용 지령')])
        layout.addWidget(system)
        axis, l = group('Beckhoff 실제 AxisStatus · /agv/axis_diagnostics (최대 5 Hz)')
        self.add_fields(l, [('axis1', 'Axis 1 · 왼쪽 휠'), ('axis2', 'Axis 2 · 오른쪽 휠'),
                            ('axis3', 'Axis 3 · 레일'), ('axis_age', 'PLC 축 진단 수신')])
        layout.addWidget(axis)
        rail, l = group('레일 · 센서 · 장애물')
        self.add_fields(l, [('rail_state', '운용 레일 상태'), ('tof_state', 'STM TOF 상태'),
                            ('tof', 'TOF 거리'), ('rail_odom', '레일 주행 거리'),
                            ('front', '전방 장애물'), ('rear', '후방 장애물'),
                            ('rail_end', '레일 끝 도달'), ('rail_command_state', '유효 레일 명령')])
        layout.addWidget(rail)
        layout.addStretch(1)

    def make_control_tab(self):
        layout = self.tab_layout('조작 · 복구')
        tip = QLabel('이 탭은 ROS 명령을 전송합니다. 휠/레일 버튼은 누르는 동안만 속도 명령을 송신하고, '
                     '손을 떼면 0을 송신합니다. 명령 송신은 실제 모터 동작 성공을 의미하지 않습니다.')
        tip.setWordWrap(True)
        layout.addWidget(tip)
        self.control_enable = QCheckBox('현장 안전 확인 후 GUI 수동 조작 활성화')
        self.control_enable.stateChanged.connect(lambda _: self.release_motion())
        layout.addWidget(self.control_enable)

        wheel, l = group('휠 조그 — /cmd_vel (누르고 유지, 10 Hz)')
        self.wheel_speed = QDoubleSpinBox()
        self.wheel_speed.setRange(0.01, 0.30)
        self.wheel_speed.setSingleStep(0.01)
        self.wheel_speed.setDecimals(2)
        self.wheel_speed.setSuffix(' m/s')
        self.wheel_speed.setValue(0.05)
        self.turn_speed = QDoubleSpinBox()
        self.turn_speed.setRange(0.05, 0.40)
        self.turn_speed.setSingleStep(0.05)
        self.turn_speed.setValue(0.10)
        self.turn_speed.setSuffix(' rad/s')
        l.addWidget(QLabel('직진 속도'), 0, 0)
        l.addWidget(self.wheel_speed, 0, 1)
        l.addWidget(QLabel('회전 속도'), 1, 0)
        l.addWidget(self.turn_speed, 1, 1)
        wheel_buttons = QHBoxLayout()
        for name, motion in [('◀ 좌회전', 'left'), ('▲ 전진', 'forward'),
                             ('▼ 후진', 'back'), ('우회전 ▶', 'right')]:
            b = QPushButton(name)
            b.pressed.connect(lambda m=motion: self.press_motion('wheel', m))
            b.released.connect(self.release_motion)
            wheel_buttons.addWidget(b)
        l.addLayout(wheel_buttons, 2, 0, 1, 2)
        stop = QPushButton('휠 STOP (0 명령)')
        stop.setProperty('danger', True)
        stop.clicked.connect(self.stop_all)
        l.addWidget(stop, 3, 0, 1, 2)
        layout.addWidget(wheel)

        rail, l = group('레일 조그 — /rail_speed_setpoint → bringup → /rail/cmd_speed')
        self.rail_speed = QDoubleSpinBox()
        self.rail_speed.setRange(0.01, 0.50)
        self.rail_speed.setDecimals(2)
        self.rail_speed.setSingleStep(0.05)
        self.rail_speed.setSuffix(' m/s')
        self.rail_speed.setValue(0.10)
        l.addWidget(QLabel('절대속도'), 0, 0)
        l.addWidget(self.rail_speed, 0, 1)
        rr = QHBoxLayout()
        for label, direction in [('레일 전진 (+)', 'forward'), ('레일 후진 (-)', 'back')]:
            b = QPushButton(label)
            b.pressed.connect(lambda d=direction: self.press_motion('rail', d))
            b.released.connect(self.release_motion)
            rr.addWidget(b)
        l.addLayout(rr, 1, 0, 1, 2)
        b = QPushButton('레일 STOP')
        b.setProperty('danger', True)
        b.clicked.connect(self.stop_all)
        l.addWidget(b, 2, 0, 1, 2)
        layout.addWidget(rail)

        recovery, l = group('드라이버 복구 · 전원 (요청 결과와 실제 READY 별도 확인)')
        actions = [
            ('FAULT RESET', '/agv/reset_faults', None),
            ('전원 복구 승인', '/agv/approve_power_restore', None),
            ('ADS 재연결', '/agv/reconnect_ads', None),
            ('Stale NC Job 복구', '/agv/recover_stale_jobs', None),
            ('휠 Power ON', '/agv/wheel_power', True),
            ('휠 Power OFF', '/agv/wheel_power', False),
            ('레일 Power ON', '/agv/rail_power', True),
            ('레일 Power OFF', '/agv/rail_power', False),
        ]
        for i, (label, name, value) in enumerate(actions):
            b = QPushButton(label)
            b.clicked.connect(lambda _=False, n=name, v=value, s=label: self.ask_service(n, v, s))
            l.addWidget(b, i // 2, i % 2)
        layout.addWidget(recovery)
        layout.addStretch(1)

    def make_energy_tab(self):
        layout = self.tab_layout('배터리 · WPT · 환경')
        battery, l = group('BMU / BMS')
        self.add_fields(l, [('battery', '전압 / SOC / 전류 / 온도'),
                            ('battery_soh', 'SOH'), ('battery_err', 'BMS 상태 코드'),
                            ('battery_raw', 'BMS 원문 상태')])
        layout.addWidget(battery)
        charger, l = group('무선 충전기 WPT')
        self.add_fields(l, [('charger', '충전기 상태'), ('charger_vi', '실측 V / I'),
                            ('charger_set', '설정 V / I'), ('charger_ready', 'READY / CONTACT'),
                            ('charger_flags', 'CHARGING / COMPLETED / ERROR'),
                            ('charger_error', '오류 코드'), ('charger_ack', 'WPT ACK')])
        layout.addWidget(charger)
        commands = QHBoxLayout()
        for label, cmd in [('충전 ON', 'ON'), ('충전 OFF', 'OFF'), ('충전기 RESET', 'RESET')]:
            b = QPushButton(label)
            b.clicked.connect(lambda _=False, c=cmd: self.ask_charger(c))
            commands.addWidget(b)
        layout.addLayout(commands)
        env, l = group('Beckhoff 환경 센서')
        self.add_fields(l, [('environment', '일사 / 온도 / 습도 / CO₂')])
        layout.addWidget(env)
        layout.addStretch(1)

    def make_logs_tab(self):
        layout = self.tab_layout('이벤트 기록')
        self.log = QTextEdit()
        self.log.setReadOnly(True)
        layout.addWidget(self.log)
        self.add_button(layout, '표시 기록 지우기', self.log.clear)

    def append_log(self, text):
        timestamp = time.strftime('%H:%M:%S')
        self.events.append(f'[{timestamp}] {text}')
        if hasattr(self, 'log'):
            self.log.append(f'[{timestamp}] {text}')

    def on_measurement(self, topic, data):
        old = self.data.get(topic)
        self.data[topic] = data
        self.stamps[topic] = time.monotonic()
        if topic in ('/agv_status', '/rail_state', '/rail/tof_state', '/charger/state') and old != data:
            self.append_log(f'{topic}: {str(data)[:230]}')

    def on_service_reply(self, name, success, message):
        result = '요청 수락' if success else '요청 실패'
        self.append_log(f'{name}: {result} · {message}')
        self.statusBar().showMessage(f'{name}: {result} · {message}', 10000)

    def alive(self, topic, seconds=2.5):
        return time.monotonic() - self.stamps.get(topic, -1.e9) < seconds

    def is_ready(self):
        text = str(self.data.get('/agv_status', '')).upper()
        return (self.alive('/agv_status', 2.5) and
                'DRIVER: READY /' in text and 'MOCK MODE' not in text)

    def is_axis_fresh(self):
        return self.alive('/agv/axis_diagnostics', 1.5)

    def motion_permitted(self, drive):
        if not self.control_enable.isChecked() or not self.is_ready() or not self.is_axis_fresh():
            return False
        axes = self.data.get('/agv/axis_diagnostics', {}).get('axes', [])
        if len(axes) < 3 or any(a.get('error', True) for a in axes):
            return False
        if drive == 'wheel':
            return all(axes[i].get('ready') for i in (0, 1)) and self.alive('/wheel/speed_feedback', 1.5)
        return (axes[2].get('ready') and self.alive('/rail_state', 2.5) and
                self.alive('/rail/speed_feedback', 1.5) and
                self.data.get('/rail_state') in ('ON_RAIL', 'WORKING', 'EXITING') and
                self.data.get('/rail/tof_state') == 'ON_RAIL' and
                self.alive('/rail/tof_state', 2.5) and
                not self.data.get('/rail_end_reached', False))

    def press_motion(self, drive, direction):
        if not self.motion_permitted(drive):
            self.append_log(f'{drive} 조작 차단: 조작 활성화/READY/축 진단/레일 상태 확인 필요')
            return
        if self.hold:
            self.release_motion()
        self.hold = (drive, direction)
        self.send_held_motion()

    def release_motion(self):
        previous = self.hold
        self.hold = None
        if previous:
            if previous[0] == 'wheel':
                self.bridge.publish_wheel(0.0, 0.0)
            else:
                self.bridge.publish_rail_stop()

    def send_held_motion(self):
        if not self.hold:
            return
        drive, direction = self.hold
        if not self.motion_permitted(drive):
            self.release_motion()
            self.append_log('조작 중단: 수신 상태/인터록 조건 변화')
            return
        if drive == 'wheel':
            speed = self.wheel_speed.value()
            turn = self.turn_speed.value()
            lin = speed if direction == 'forward' else -speed if direction == 'back' else 0.0
            angular = turn if direction == 'left' else -turn if direction == 'right' else 0.0
            self.bridge.publish_wheel(lin, angular)
        else:
            speed = self.rail_speed.value() * (1 if direction == 'forward' else -1)
            # Directional limit check at command time.
            if (speed > 0 and (self.data.get('/rail_obstacle_front', True) or
                               self.data.get('/rail_end_reached', True))) or (
                    speed < 0 and self.data.get('/rail_obstacle_rear', True)):
                self.release_motion()
                self.append_log('레일 명령 차단: 장애물 또는 끝단')
                return
            self.bridge.publish_rail_speed(speed)

    def stop_all(self):
        self.release_motion()
        self.bridge.publish_wheel(0.0, 0.0)
        self.bridge.publish_rail_stop()
        self.append_log('휠/레일 속도 0 및 레일 STOP 요청 (물리적 E-stop 아님)')

    def ask_service(self, name, value, description):
        self.stop_all()
        if not self.control_enable.isChecked():
            self.append_log('서비스 조작 차단: 현장 확인 체크 필요')
            return
        if not self.alive('/agv_status', 2.5) or not self.is_axis_fresh():
            self.append_log('서비스 조작 차단: AGV 상태/AxisStatus 수신 불량')
            return
        # Never treat a GUI-side velocity display of zero as proof of PLC standstill.
        axes = self.data.get('/agv/axis_diagnostics', {}).get('axes', [])
        if name != '/agv/reset_faults' and not (
                len(axes) == 3 and all(
                    a.get('not_moving') and not a.get('has_job') and
                    abs(float(a.get('velocity_mps', 1))) <= 0.01 for a in axes)):
            self.append_log('서비스 조작 차단: PLC 3축 정지/NC Job 없음 미확인')
            return
        reply = QMessageBox.question(
            self, '현장 안전 확인',
            f'{description} 요청을 전송합니까?\n\nPLC/현장 확인이 필요하며, 서비스 성공은 모터 준비 완료를 뜻하지 않습니다.',
            QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
        if reply == QMessageBox.Yes:
            self.bridge.request(name, value)
            self.append_log(f'{description} 요청 전송: {name}')

    def ask_charger(self, command):
        reply = QMessageBox.question(self, 'WPT 명령 확인',
            f'무선 충전기에 {command} 명령을 전송합니까?\nWPT ACK와 실제 전류를 확인하세요.',
            QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
        if reply == QMessageBox.Yes:
            self.bridge.publish_charger(command)
            self.append_log(f'WPT {command} 명령 발행 (ACK/작동 미확인)')

    def show_value(self, key, text, warn=False):
        label = self.values.get(key)
        if label:
            label.setText(str(text))
            label.setStyleSheet('color: #ff9696;' if warn else 'color: #e9eff9;')

    def read(self, topic, default='--'):
        return self.data.get(topic, default) if self.alive(topic, 3.0) else default

    def fmt(self, x, suffix='', digits=2):
        try:
            return f'{float(x):.{digits}f}{suffix}' if math.isfinite(float(x)) else '--'
        except (ValueError, TypeError):
            return '--'

    def refresh(self):
        if self.hold:
            self.send_held_motion()
        status = self.read('/agv_status')
        self.top_status.setText(str(status)[:160])
        ready = self.is_ready()
        self.top_status.setStyleSheet('font-size: 17px; font-weight: 800; color: ' +
                                      ('#76dbaa;' if ready else '#ffb279;'))
        age = time.monotonic() - self.stamps.get('/agv_status', -1e9)
        self.top_fresh.setText('ADS STATUS: ' + (f'{age:.1f}s 전' if age < 999 else 'NO DATA'))
        self.show_value('agv', status, not ready)
        self.show_value('source_age', f'{age:.1f}초' if age < 999 else '미수신', age > 2.5)
        self.show_value('axis_age', ('정상' if self.is_axis_fresh() else '미수신 / 지연'), not self.is_axis_fresh())
        axes = self.read('/agv/axis_diagnostics', {})
        if isinstance(axes, dict):
            for i, axis in enumerate(axes.get('axes', [])[:3]):
                code = axis.get('error_code', 0)
                desc = (f"v={self.fmt(axis.get('velocity_mps'), ' m/s', 3)}  "
                        f"pos={self.fmt(axis.get('position_mm'), ' mm', 1)}  "
                        f"Ready={int(bool(axis.get('ready')))}  NM={int(bool(axis.get('not_moving')))}  "
                        f"Job={int(bool(axis.get('has_job')))}  "
                        f"Error={int(bool(axis.get('error')))}  Code=0x{int(code):X}")
                self.show_value(f'axis{i+1}', desc, axis.get('error', False))
        w = self.read('/wheel/speed_feedback', [])
        if isinstance(w, list) and len(w) >= 2:
            self.show_value('wheel', f'L={self.fmt(w[0], " m/s", 3)} / R={self.fmt(w[1], " m/s", 3)}')
        wo = self.read('/wheel/odom', {})
        if isinstance(wo, dict):
            self.show_value('wheel_odom', f"x={self.fmt(wo.get('x'), ' m')} y={self.fmt(wo.get('y'), ' m')} "
                           f"v={self.fmt(wo.get('velocity'), ' m/s', 3)}")
        rf = self.read('/rail/speed_feedback', [])
        if isinstance(rf, list) and rf:
            self.show_value('rail_fb', self.fmt(rf[0], ' m/s', 3))
        self.show_value('rail_cmd', self.fmt(self.read('/rail/cmd_speed'), ' m/s', 3))
        self.show_value('rail_state', self.read('/rail_state'))
        self.show_value('tof_state', self.read('/rail/tof_state'))
        self.show_value('tof', self.fmt(self.read('/tof_distance'), ' mm', 1))
        ro = self.read('/rail/odom', {})
        if isinstance(ro, dict):
            self.show_value('rail_odom', self.fmt(ro.get('x'), ' m', 3))
        for key, topic in [('front', '/rail_obstacle_front'), ('rear', '/rail_obstacle_rear'),
                           ('rail_end', '/rail_end_reached')]:
            value = self.read(topic)
            self.show_value(key, '감지 / 제한' if value is True else '정상' if value is False else '--', value is True)
        self.show_value('rail_command_state', self.read('/rail_command_state'))

        batt = self.read('/battery/state', {})
        if isinstance(batt, dict):
            self.show_value('battery',
                f"{self.fmt(batt.get('voltage'), ' V')} / "
                f"{self.fmt(batt.get('percentage', float('nan')) * 100, '%', 1)} / "
                f"{self.fmt(batt.get('current'), ' A')} / "
                f"{self.fmt(batt.get('temperature'), ' °C', 1)}")
        self.show_value('battery_soh', self.fmt(self.read('/battery/soh'), '%', 1))
        self.show_value('battery_err', self.read('/battery/error_code'))
        self.show_value('battery_raw', str(self.read('/battery/status'))[:180])
        self.show_value('charger', self.read('/charger/state'))
        self.show_value('charger_vi', f"{self.fmt(self.read('/charger/voltage'), ' V')} / "
                        f"{self.fmt(self.read('/charger/current'), ' A')}")
        self.show_value('charger_set', f"{self.fmt(self.read('/charger/set_voltage'), ' V')} / "
                        f"{self.fmt(self.read('/charger/set_current'), ' A')}")
        self.show_value('charger_ready', f"{self.read('/charger/ready')} / {self.read('/charger/contact')}")
        self.show_value('charger_flags', f"{self.read('/charger/charging')} / "
                        f"{self.read('/charger/completed')} / {self.read('/charger/error')}",
                        self.read('/charger/error') is True)
        self.show_value('charger_error', self.read('/charger/error_code'))
        self.show_value('charger_ack', self.read('/charger/ack'))
        env = self.read('/environment/data', [])
        if isinstance(env, list) and len(env) >= 4:
            self.show_value('environment', f'{self.fmt(env[0], " W/m²")} / '
                            f'{self.fmt(env[1], " °C")} / '
                            f'{self.fmt(env[2], " %")} / '
                            f'{self.fmt(env[3], " ppm", 0)}')

    def closeEvent(self, event):
        self.stop_all()
        self.tick.stop()
        self.bridge.stop()
        if not self.bridge.wait(2000):
            self.append_log('ROS 스레드 종료 지연: GUI 종료만으로 모터 안전정지를 보장할 수 없음')
        else:
            self.bridge.cleanup()
        event.accept()


def main():
    rclpy.init()
    app = QApplication(sys.argv)
    bridge = RosBridge()
    dash = Dashboard(bridge)
    bridge.start()
    dash.show()
    try:
        return app.exec_()
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
