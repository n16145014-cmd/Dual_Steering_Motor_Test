#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Shared Side A/Side B steering bridge implementation (based on validated V4).

All command, status, legacy command and feedback topics are side-scoped.
Safety: bench-only; firmware uses integer output-degree targets and an
unverified 20000 position-steps/degree assumption. STOP_ALL is not an E-stop.
"""
import math
import re
from collections import deque

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Float64MultiArray, String
import serial

DEFAULT_PORT = '/dev/serial/by-id/usb-1a86_USB2.0-Serial-if00-port0'
STEPS_PER_DEG = 20000.0  # Verify against real output shaft movement!
ANGLE_LIMIT = 180
MAX_RPM = 1000  # STM32 bench firmware limit; actual 0x6081 units unverified
DEFAULT_RPM = 100
MAX_JOBS = 30
# Time after a complete UART transaction to let the blocking MCU return to RX.
UART_GUARD_NS = 120_000_000


class CommandError(Exception):
    pass


def is_integer(value):
    return math.isfinite(value) and abs(value - round(value)) < 1e-7


def parse_angles(token, action):
    """Parse A90, a-10, A[90,-90]. Case-sensitive ONLY for A/a."""
    if not token or token[0] != action:
        raise CommandError('角度格式錯誤')
    arg = token[1:].strip()
    if arg.startswith('[') and arg.endswith(']'):
        pieces = [p.strip() for p in arg[1:-1].split(',')]
        if len(pieces) != 2:
            raise CommandError('all 陣列必須是 [M1,M2] 兩個角度')
        try:
            values = [float(x) for x in pieces]
        except ValueError:
            raise CommandError('角度必須是數字')
    else:
        try:
            values = [float(arg)]
        except ValueError:
            raise CommandError('角度必須是數字')
    if any(not is_integer(x) for x in values):
        raise CommandError('目前 STM32 僅支援整數度；請輸入整數角度')
    return [int(round(x)) for x in values]


def parse_speed(token):
    if not token or token[0].lower() != 's':
        raise CommandError('速度格式應為 S100 或 s100')
    arg = token[1:].strip()
    if arg.startswith('[') and arg.endswith(']'):
        parts = [p.strip() for p in arg[1:-1].split(',')]
        if len(parts) != 2:
            raise CommandError('速度陣列必須是 [M1,M2]')
    else:
        parts = [arg]
    try:
        floats = [float(x) for x in parts]
    except ValueError:
        raise CommandError('速度必須是數字')
    if any(not is_integer(x) or not 1 <= x <= MAX_RPM for x in floats):
        raise CommandError(f'速度必須是 1～{MAX_RPM} 的整數設定值')
    return [int(round(x)) for x in floats]


def expand(values, motors):
    if len(values) == 1:
        return [values[0]] * len(motors)
    if motors != [1, 2]:
        raise CommandError('陣列形式僅適用於 all')
    return values


def parse_user_command(text):
    """Parse user-friendly syntax, return an immutable job description."""
    parts = [p.strip() for p in text.strip().split(';')]
    if len(parts) < 2 or not all(parts):
        raise CommandError('格式：m1|m2|all;action[;S速度]')
    target = parts[0].lower()
    if target not in ('m1', 'm2', 'all'):
        raise CommandError('目標只接受 m1、m2、all')
    motors = [1, 2] if target == 'all' else [int(target[1])]
    action = parts[1]
    low = action.lower()

    if low in ('brakeon', 'brakeoff'):
        raise CommandError('目前 main.c 無獨立電磁煞車控制；不可把 enable/disable 視為 brakeoff/brakeon')
    if low in ('jog', 'jogstop'):
        raise CommandError('目前 main.c 不支援安全的按住式 Jog；須先實作 MCU Jog watchdog')
    if low in ('disable',):
        raise CommandError('目前 main.c 無獨立 disable；請使用 all;stop (Quick Stop)')
    if low == 'stop':
        if len(parts) != 2:
            raise CommandError('stop 不接受其他參數')
        if target != 'all':
            raise CommandError('目前韌體只能 STOP_ALL，單顆 stop 尚不支援')
        return ('stop', motors, None, None)
    if low in ('position', 'status', 'sethome', 'enable', 'probe'):
        if len(parts) != 2:
            raise CommandError(f'{low} 不接受其他參數')
        return (low, motors, None, None)
    if low == 'home':
        if len(parts) > 3:
            raise CommandError('home 用法：all;home;S100')
        speed = parse_speed(parts[2]) if len(parts) == 3 else [DEFAULT_RPM]
        return ('absolute', motors, [0] * len(motors), expand(speed, motors))
    if action[0] in ('A', 'a') and len(action) > 1:
        if len(parts) > 3:
            raise CommandError('角度用法：m1;A90;S100 或 all;a[10,-10];S100')
        angle = expand(parse_angles(action, action[0]), motors)
        speed = expand(parse_speed(parts[2]) if len(parts) == 3 else [DEFAULT_RPM], motors)
        if action[0] == 'A' and any(abs(a) > ANGLE_LIMIT for a in angle):
            raise CommandError(f'絕對目標超出 ±{ANGLE_LIMIT}°')
        return ('absolute' if action[0] == 'A' else 'relative', motors, angle, speed)
    raise CommandError('未知指令；A=絕對、a=相對、S/s=速度；請參閱指令.txt')


class MotorBridge(Node):
    def __init__(self, configured_side='side_a'):
        super().__init__('dual_steering_bridge')
        self.side = configured_side
        if self.side not in ('side_a', 'side_b'):
            raise ValueError('side must be side_a or side_b')
        self.prefix = '/steering/' + self.side
        self.port = self.declare_parameter('port', DEFAULT_PORT).value
        node_ids_text = self.declare_parameter('node_ids', '1,2' if self.side == 'side_a' else '3,4').value
        try:
            ids = [int(x.strip()) for x in node_ids_text.split(',')]
            if len(ids) != 2 or len(set(ids)) != 2 or not all(1 <= i <= 127 for i in ids):
                raise ValueError('expected two distinct CANopen Node IDs in 1..127')
        except (ValueError, AttributeError) as ex:
            raise ValueError('node_ids must be a string such as 3,4') from ex
        if ids not in ([1, 2], [3, 4]):
            raise ValueError('node_ids must be 1,2 or 3,4, in that order; set the matching IDs in V5')
        self.reply_to_logical = {ids[0]: 1, ids[1]: 2}
        self.ser = serial.Serial(self.port, 115200, timeout=0, write_timeout=0.1)
        self.get_logger().info('Connected: ' + self.port + ' | side=' + self.side + ' | UART M1/M2 map to CANopen ' + str(ids))
        self.rx = bytearray()
        self.job_queue = deque()
        self.active_job = None
        self.pending = None  # (sent_cmd, expected_kind, expected_motor, start_ns)
        self.next_req = None
        self.last_hb_ns = 0
        self.last_pos_ns = 0
        self.poll_motor = 1
        self.next_tx_ns = 0
        self.active_job_meta = None
        self.suspend_polling = False
        self.pos = {1: None, 2: None}  # cache used ONLY for feedback output
        self.status_pub = self.create_publisher(String, self.prefix + '/status', 30)
        self.result_pub = self.create_publisher(String, self.prefix + '/motor/result', 20)
        self.feedback = {
            1: self.create_publisher(Float64, self.prefix + '/left_steering/feedback_deg', 10),
            2: self.create_publisher(Float64, self.prefix + '/right_steering/feedback_deg', 10),
        }
        self.create_subscription(String, self.prefix + '/motor/command', self.user_command, 10)
        self.create_subscription(String, self.prefix + '/control', self.legacy_control, 10)
        self.create_subscription(Float64, self.prefix + '/left_steering/command_deg',
                                 lambda m: self.legacy_angle(1, m.data), 10)
        self.create_subscription(Float64, self.prefix + '/right_steering/command_deg',
                                 lambda m: self.legacy_angle(2, m.data), 10)
        self.create_subscription(Float64MultiArray, self.prefix + '/command', self.legacy_dual, 10)
        self.create_timer(0.02, self.tick)
        self.enqueue(('probe', [1, 2], None, None))
        self.get_logger().info('Motor Command V1 ready: ' + self.prefix + '/motor/command (std_msgs/String)')

    def publish_result(self, message):
        m = String()
        m.data = message
        self.result_pub.publish(m)
        if message.startswith('ERR'):
            self.get_logger().error(message)
        else:
            self.get_logger().info(message)

    def enqueue(self, job):
        if len(self.job_queue) >= MAX_JOBS:
            self.publish_result('ERR QUEUE_FULL')
            return
        self.job_queue.append(job)
        self.publish_result(f'QUEUED {job[0].upper()} motors={job[1]}')

    def user_command(self, msg):
        try:
            job = parse_user_command(msg.data)
            if job[0] == 'stop':
                self.request_stop()
            else:
                self.enqueue(job)
        except CommandError as ex:
            self.publish_result('ERR COMMAND ' + str(ex))

    def legacy_control(self, msg):
        mappings = {
            'ZERO_M1': ('sethome', [1], None, None),
            'ZERO_M2': ('sethome', [2], None, None),
            'ENABLE_M1': ('enable', [1], None, None),
            'ENABLE_M2': ('enable', [2], None, None),
            'PROBE_M1': ('probe', [1], None, None),
            'PROBE_M2': ('probe', [2], None, None),
            'STATUS': ('status', [1, 2], None, None),
        }
        name = msg.data.strip().upper()
        if name == 'STOP_ALL':
            self.request_stop()
        elif name in mappings:
            self.enqueue(mappings[name])
        else:
            self.publish_result('ERR legacy control: ' + name)

    def legacy_angle(self, motor, angle):
        if not is_integer(angle) or not -ANGLE_LIMIT <= angle <= ANGLE_LIMIT:
            self.publish_result('ERR legacy angle must be integer degrees inside ±180')
            return
        self.enqueue(('absolute', [motor], [int(round(angle))], [DEFAULT_RPM]))

    def legacy_dual(self, msg):
        vals = list(msg.data)
        if len(vals) != 2 or any(not is_integer(x) or abs(x) > ANGLE_LIMIT for x in vals):
            self.publish_result('ERR dual requires two integer angles within ±180')
            return
        self.enqueue(('absolute', [1, 2], [int(round(x)) for x in vals],
                      [DEFAULT_RPM, DEFAULT_RPM]))

    def request_stop(self):
        self.job_queue.clear()
        self.active_job = None
        self.active_job_meta = None
        self.pending = None
        self.next_req = None
        self._write('STOP_ALL')
        self.publish_result('STOP_ALL SENT (request only; not certified E-stop)')

    def _write(self, command):
        # All UART writes go through this one ROS executor callback/thread.
        self.ser.write((command + '\n').encode('ascii'))
        if command != 'HB':
            self.get_logger().info('UART TX ' + command)

    def _fail_job(self, reason):
        # If ALL enable was partially successful, request a quick stop of both.
        is_all_enable = (self.active_job_meta is not None and
                         self.active_job_meta[0] == 'enable' and
                         self.active_job_meta[1] == [1, 2])
        self.active_job = None
        self.active_job_meta = None
        self.next_req = None
        self.pending = None
        self.publish_result('ERR ' + reason)
        if is_all_enable:
            self.job_queue.clear()
            self._write('STOP_ALL')
            self.publish_result('WARN ALL ENABLE FAILED: STOP_ALL REQUESTED; check hardware state')
        self.next_tx_ns = self.get_clock().now().nanoseconds + UART_GUARD_NS

    @staticmethod
    def _parse_position(line):
        m = re.search(r'^OK M([12]) POS ACT_POS=(-?\d+) REL_STEPS=(-?\d+) REF_VALID=([01])(?: MOVING=([01]))?', line)
        if not m:
            return None
        return {'motor': int(m.group(1)), 'actual': int(m.group(2)),
                'steps': int(m.group(3)), 'valid': m.group(4) == '1',
                'moving': m.group(5) == '1' if m.group(5) else False}

    def _handle_line(self, line):
        m = String()
        m.data = line
        self.status_pub.publish(m)
        # Keep the raw firmware response in /steering/status; translate only
        # the leading motor ID for internal V3 parsing and transaction matching.
        head = re.match(r'^(OK|ERR) M(\d+)(?=\s|$)', line)
        if head:
            physical_id = int(head.group(2))
            if physical_id in self.reply_to_logical:
                logical_id = self.reply_to_logical[physical_id]
                line = line[:head.start(2)] + str(logical_id) + line[head.end(2):]
        position = self._parse_position(line)
        if position:
            self.pos[position['motor']] = position
            if position['valid']:
                value = Float64()
                value.data = position['steps'] / STEPS_PER_DEG
                self.feedback[position['motor']].publish(value)
        # Heartbeat has no response; polling responses are ignored by jobs unless
        # a matching request was explicitly pending.
        if not self.pending:
            if line.startswith('ERR'):
                self.get_logger().warning('MCU: ' + line)
            return
        sent, kind, motor, started = self.pending
        matched = False
        if kind == 'position':
            matched = position is not None and position['motor'] == motor
        elif kind == 'init':
            matched = (line.startswith(f'OK M{motor} ENABLED') or
                       line.startswith(f'ERR M{motor} INIT') or
                       line.startswith('ERR INIT:'))
        elif kind == 'zero':
            matched = line.startswith(f'OK M{motor} ZERO') or line.startswith(f'ERR M{motor} ZERO')
        elif kind == 'probe':
            matched = line.startswith(f'OK M{motor} ONLINE') or line.startswith(f'ERR M{motor} PROBE')
        elif kind == 'motion':
            matched = (line.startswith(f'OK M{motor} ACCEPT') or
                       line.startswith(f'ERR M{motor}'))
        elif kind == 'status':
            matched = line.startswith('STATUS ')
        elif kind == 'poll':
            matched = position is not None and position['motor'] == motor
        if not matched:
            if line.startswith('ERR'):
                self.get_logger().warning('MCU unsolicited: ' + line)
            return
        self.pending = None
        self.next_tx_ns = self.get_clock().now().nanoseconds + UART_GUARD_NS
        if kind == 'poll':
            return
        if line.startswith('ERR'):
            self._fail_job(f'MCU for {sent}: {line}')
            return
        if kind == 'init':
            self.publish_result(f'OK M{motor} ENABLE ACKNOWLEDGED (brake state not independently sensed)')
        try:
            self.next_req = self.active_job.send(position if kind == 'position' else line)
        except StopIteration:
            self.active_job = None
            self.active_job_meta = None
            self.next_req = None
            self.publish_result('OK COMMAND COMPLETED (accepted; motion may still be running)')
        except CommandError as ex:
            self._fail_job(str(ex))

    def _program(self, job):
        """Generator: yield (UART command, response kind, motor).

        For relative commands, ALWAYS request a fresh actual position, and
        reject motion while moving or before ZERO. Preflight both motors
        before dispatching an ALL motion; dispatch isn't atomic.
        """
        action, motors, values, speeds = job
        if action == 'status':
            yield ('STATUS', 'status', 0)
            for motor in motors:
                yield (f'M{motor}:PROBE', 'probe', motor)
            return
        if action in ('position', 'probe', 'sethome'):
            kinds = {'position': ('POS', 'position'), 'probe': ('PROBE', 'probe'),
                     'sethome': ('ZERO', 'zero')}
            suffix, kind = kinds[action]
            for motor in motors:
                yield (f'M{motor}:{suffix}', kind, motor)
            return
        if action == 'enable':
            # Do not dispatch M2:INIT until STM32 has explicitly completed M1:INIT.
            # INIT blocks the STM32 UART receive loop; TX HB must be suppressed
            # during these transactions to avoid partial/garbled UART frames.
            for motor in motors:
                yield (f'M{motor}:INIT', 'init', motor)
            status = yield ('STATUS', 'status', 0)
            for motor in motors:
                match = re.search(rf'M{motor}_READY=([01])', status)
                if not match or match.group(1) != '1':
                    raise CommandError(f'M{motor} READY is not 1 after INIT: {status}')
            for motor in motors:
                probe = yield (f'M{motor}:PROBE', 'probe', motor)
                match = re.search(r'STATUSWORD=0x([0-9A-Fa-f]+)', probe)
                if not match or (int(match.group(1), 16) & 0x006F) != 0x0027:
                    raise CommandError(f'M{motor} not Operation Enabled: {probe}')
            self.publish_result('OK ENABLE VERIFIED for motors=' + str(motors) +
                                ' (CiA402 status only; physical brake unverified)')
            return
        if action in ('absolute', 'relative'):
            targets = []
            # Query each motor before any motion, to avoid partially dispatching
            # an ALL command with invalid/missing reference or limits.
            for index, motor in enumerate(motors):
                pos = yield (f'M{motor}:POS', 'position', motor)
                if not pos or not pos['valid']:
                    raise CommandError(f'M{motor} 尚未設定 sethome (REF_VALID=0)')
                if pos['moving']:
                    raise CommandError(f'M{motor} MOVING=1，拒絕重疊指令')
                if action == 'relative':
                    target_float = pos['steps'] / STEPS_PER_DEG + values[index]
                    target = round(target_float)
                    # Firmware accepts integer targets only; explicit quantization
                    if abs(target_float - target) > 0.05:
                        self.publish_result(
                            f'WARN M{motor} relative target {target_float:.4f}° '
                            f'quantized to {target}° (MCU integer-degree protocol)')
                else:
                    target = values[index]
                if not -ANGLE_LIMIT <= target <= ANGLE_LIMIT:
                    raise CommandError(f'M{motor} target {target}° exceeds ±{ANGLE_LIMIT}°')
                targets.append((motor, target, speeds[index]))
            for motor, angle, speed in targets:
                yield (f'M{motor}:A{angle},S{speed}', 'motion', motor)
            return
        raise CommandError('unsupported action')

    def _advance_program(self):
        if self.pending:
            return
        if self.active_job is None:
            if not self.job_queue:
                return
            self.active_job_meta = self.job_queue.popleft()
            self.active_job = self._program(self.active_job_meta)
        try:
            if self.next_req is not None:
                command, kind, motor = self.next_req
                self.next_req = None
            else:
                command, kind, motor = next(self.active_job)
        except StopIteration:
            self.active_job = None
            self.active_job_meta = None
            return
        except CommandError as ex:
            self._fail_job(str(ex))
            return
        self._write(command)
        self.pending = (command, kind, motor, self.get_clock().now().nanoseconds)

    def tick(self):
        try:
            data = self.ser.read(4096)
            if data:
                self.rx.extend(data)
                if len(self.rx) > 8192:
                    self.rx.clear()
                    self.publish_result('ERR UART RX OVERFLOW')
                while b'\n' in self.rx:
                    raw, _, remaining = self.rx.partition(b'\n')
                    self.rx = bytearray(remaining)
                    line = raw.decode('utf-8', errors='replace').strip()
                    if line:
                        self._handle_line(line)
            now = self.get_clock().now().nanoseconds
            if self.pending:
                # Critical: NEVER send HB while MCU is in blocking INIT / SDO.
                # Those bytes can be dropped and corrupt subsequent UART lines.
                limit_ns = 12_000_000_000 if self.pending[1] == 'init' else 3_000_000_000
                if now - self.pending[3] > limit_ns:
                    self._fail_job('UART RESPONSE TIMEOUT: ' + self.pending[0])
                return
            if now < self.next_tx_ns:
                return
            # Issue ONE heartbeat between transactions, not during them.
            if now - self.last_hb_ns >= 250_000_000:
                self._write('HB')
                self.last_hb_ns = now
                self.next_tx_ns = now + UART_GUARD_NS
                return
            if self.active_job is not None or self.job_queue:
                self._advance_program()
                return
            # Track polling requests too, preventing interleaved/unknown replies.
            if now - self.last_pos_ns >= 400_000_000:
                motor = self.poll_motor
                cmd = f'M{motor}:POS'
                self._write(cmd)
                self.pending = (cmd, 'poll', motor, now)
                self.poll_motor = 2 if motor == 1 else 1
                self.last_pos_ns = now
        except (serial.SerialException, OSError) as ex:
            self.get_logger().fatal('UART fault, firmware watchdog should stop: ' + str(ex))
            raise

    def destroy_node(self):
        try:
            self._write('STOP_ALL')
            self.ser.flush()
            self.ser.close()
        except (serial.SerialException, OSError):
            pass
        super().destroy_node()


def main(side='side_a'):
    rclpy.init()
    node = None
    try:
        node = MotorBridge(configured_side=side)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main('side_a')
