#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Simple ROS 2 four-motor command router for two existing V4 isolated bridges.

No UART, CAN, heartbeat, polling, timers, preflight, or automatic retry here.
V4/STM32 own the actual control and safety checks. Bench use only.
Software all;stop is NOT a safety-rated emergency stop.
"""
import re
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

SIDES = ('side_a', 'side_b')
MAX_ANGLE = 180
MAX_SPEED = 1000


def number(token, name, minimum, maximum):
    token = token.strip()
    if not re.fullmatch(r'[+-]?\d+', token):
        raise ValueError(f'{name} must be an integer')
    value = int(token)
    if not minimum <= value <= maximum:
        raise ValueError(f'{name} must be in {minimum}..{maximum}')
    return value


def parse_values(raw, count, name, low, high):
    if raw.startswith('[') and raw.endswith(']'):
        chunks = raw[1:-1].split(',')
        if len(chunks) != count:
            raise ValueError(f'{name} array must contain {count} values')
        return [number(x, name, low, high) for x in chunks]
    value = number(raw, name, low, high)
    return [value] * count


def route_command(command, side_a_nodes=(1, 2), side_b_nodes=(3, 4)):
    """Pure parser. Return {side: V4 command} or raise ValueError."""
    if sorted(side_a_nodes + side_b_nodes) != [1, 2, 3, 4] or (tuple(side_a_nodes) not in ((1,2),(3,4))) or (tuple(side_b_nodes) not in ((1,2),(3,4))):
        raise ValueError('node mapping must be complementary pairs 1,2 and 3,4')
    side_of = {m: side for side, ids in (('side_a', side_a_nodes), ('side_b', side_b_nodes)) for m in ids}
    local_of = {m: i+1 for ids in (side_a_nodes, side_b_nodes) for i,m in enumerate(ids)}
    parts = [x.strip() for x in command.split(';')]
    if len(parts) not in (2, 3) or any(not x for x in parts):
        raise ValueError('expected m1|m2|m3|m4|all;action[;Sspeed]')
    target = parts[0].lower()
    if target == 'all':
        motors = [1, 2, 3, 4]
    elif target in ('m1', 'm2', 'm3', 'm4'):
        motors = [int(target[1])]
    else:
        raise ValueError('target must be m1,m2,m3,m4,all')
    action = parts[1]
    action_l = action.lower()

    if action_l in ('status', 'position', 'probe', 'sethome', 'enable', 'stop'):
        if len(parts) != 2:
            raise ValueError(f'{action_l} does not accept speed')
        if action_l == 'stop' and target != 'all':
            raise ValueError('only all;stop is supported by current V4 firmware')
        if target == 'all':
            return {s: f'all;{action_l}' for s in SIDES}
        motor = motors[0]
        return {side_of[motor]: f'm{local_of[motor]};{action_l}'}

    if action_l in ('disable', 'jog', 'jogstop', 'brakeon', 'brakeoff'):
        raise ValueError(f'{action_l} unsupported by current V4/STM32')

    if action_l == 'home':
        action_kind = 'A'
        angles = [0] * len(motors)
    elif len(action) >= 2 and action[0] in ('A', 'a'):
        action_kind = action[0]
        angles = parse_values(action[1:], len(motors), 'angle', -MAX_ANGLE, MAX_ANGLE)
    else:
        raise ValueError('unknown action')

    if len(parts) == 3:
        speed_token = parts[2]
        if not speed_token or speed_token[0].lower() != 's':
            raise ValueError('speed must be S100 or S[100,100,100,100]')
        speeds = parse_values(speed_token[1:], len(motors), 'speed', 1, MAX_SPEED)
    else:
        speeds = [100] * len(motors)

    if target == 'all':
        settings = {motor: (angles[i], speeds[i]) for i, motor in enumerate(motors)}
        return {side: (f'all;{action_kind}[{settings[ids[0]][0]},{settings[ids[1]][0]}];'
                       f'S[{settings[ids[0]][1]},{settings[ids[1]][1]}]')
                for side, ids in (('side_a', side_a_nodes), ('side_b', side_b_nodes))}
    motor = motors[0]
    local = local_of[motor]
    return {side_of[motor]: f'm{local};{action_kind}{angles[0]};S{speeds[0]}'}


class FourMotorRouter(Node):
    def __init__(self):
        super().__init__('four_motor_router_v5')
        def parse_pair(name, default):
            raw = self.declare_parameter(name, default).value
            try:
                values = tuple(int(p.strip()) for p in raw.split(','))
            except (ValueError, AttributeError) as exc:
                raise ValueError(f'{name} must be 1,2 or 3,4') from exc
            if values not in ((1, 2), (3, 4)):
                raise ValueError(f'{name} must be 1,2 or 3,4')
            return values
        self.side_a_nodes = parse_pair('side_a_nodes', '1,2')
        self.side_b_nodes = parse_pair('side_b_nodes', '3,4')
        if self.side_a_nodes == self.side_b_nodes:
            raise ValueError('side A/B must have different node pairs')
        self.commands = {
            s: self.create_publisher(String, f'/steering/{s}/motor/command', 10)
            for s in SIDES
        }
        self.result_pub = self.create_publisher(String, '/steering/four/result', 30)
        self.status_pub = self.create_publisher(String, '/steering/four/status', 50)
        for side in SIDES:
            self.create_subscription(
                String, f'/steering/{side}/motor/result',
                lambda msg, s=side: self.forward_result(s, msg.data), 30)
            self.create_subscription(
                String, f'/steering/{side}/status',
                lambda msg, s=side: self.forward_status(s, msg.data), 50)
        self.create_subscription(String, '/steering/four/command', self.on_command, 10)
        self.publish_result(f'READY V5 SIMPLE ROUTER; A={self.side_a_nodes} B={self.side_b_nodes}; V4/STM32 execute')

    def publish_result(self, message):
        self.result_pub.publish(String(data=message))
        if message.startswith('ERR'):
            self.get_logger().error(message)
        else:
            self.get_logger().info(message)

    def forward_result(self, side, line):
        self.result_pub.publish(String(data=f'{side}: {line}'))

    def forward_status(self, side, line):
        self.status_pub.publish(String(data=f'{side}: {line}'))

    def on_command(self, message):
        try:
            commands = route_command(message.data, self.side_a_nodes, self.side_b_nodes)
        except ValueError as exc:
            self.publish_result(f'ERR V5 ROUTER: {exc}')
            return
        for side, command in commands.items():
            self.commands[side].publish(String(data=command))
            self.publish_result(f'ROUTED {side}: {command}')


def main():
    rclpy.init()
    node = FourMotorRouter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
