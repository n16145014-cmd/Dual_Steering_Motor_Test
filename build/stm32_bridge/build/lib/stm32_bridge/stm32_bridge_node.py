import re
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial


OUTPUT_STEPS_PER_DEG = 20000.0


class STM32Bridge(Node):
    def __init__(self):
        super().__init__('stm32_bridge')

        self.ser = serial.Serial(
            '/dev/ttyUSB0',
            115200,
            timeout=0.2
        )

        self.subscription = self.create_subscription(
            String,
            '/steering_cmd',
            self.command_callback,
            10
        )

        self.feedback_pub = self.create_publisher(
            String,
            '/steering_feedback',
            10
        )

        self.get_logger().info(
            'STM32 steering bridge started: '
            'Axx,Sxx | HOME,Sxx | POS | SAVE'
        )

    def command_callback(self, msg):
        command = msg.data.strip()

        self.ser.reset_input_buffer()
        self.ser.write((command + '\n').encode())
        self.ser.flush()

        self.get_logger().info(f'Sent: {command}')

        deadline = time.monotonic() + 35.0
        final_line = None

        while time.monotonic() < deadline:
            raw = self.ser.readline()

            if not raw:
                continue

            line = raw.decode(errors='ignore').strip()

            if not line:
                continue

            self.get_logger().info(f'STM32: {line}')

            if line.startswith('OK '):
                final_line = line
                self.publish_feedback(line)
                break

            if line.startswith('ERR '):
                final_line = line

                feedback = String()
                feedback.data = line
                self.feedback_pub.publish(feedback)
                break

        if final_line is None:
            self.get_logger().error(
                f'Timeout waiting for STM32 reply: {command}'
            )

            feedback = String()
            feedback.data = f'ERR TIMEOUT CMD={command}'
            self.feedback_pub.publish(feedback)

    def publish_feedback(self, line):
        feedback = String()

        # Normal angle command:
        # OK CMD=10 SPEED=300 ACT_POS=123456 REL_STEPS=199800
        match_cmd = re.search(
            r'CMD=(-?\d+)\s+'
            r'SPEED=(-?\d+)\s+'
            r'ACT_POS=(-?\d+)\s+'
            r'REL_STEPS=(-?\d+)',
            line
        )

        if match_cmd:
            cmd_deg = int(match_cmd.group(1))
            speed_rpm = int(match_cmd.group(2))
            actual_position = int(match_cmd.group(3))
            relative_steps = int(match_cmd.group(4))

            actual_deg = relative_steps / OUTPUT_STEPS_PER_DEG
            error_deg = cmd_deg - actual_deg

            feedback.data = (
                f'STATUS=OK '
                f'MODE=MOVE '
                f'CMD_DEG={cmd_deg} '
                f'ACT_DEG={actual_deg:.3f} '
                f'ERROR_DEG={error_deg:.3f} '
                f'SPEED_RPM={speed_rpm} '
                f'ACT_POS={actual_position} '
                f'REL_STEPS={relative_steps}'
            )

            self.feedback_pub.publish(feedback)

            self.get_logger().info(
                f'Feedback: cmd={cmd_deg:.3f} deg, '
                f'actual={actual_deg:.3f} deg, '
                f'error={error_deg:.3f} deg'
            )
            return

        # HOME:
        # OK HOME ACT_POS=123456 REL_STEPS=5
        match_home = re.search(
            r'HOME\s+ACT_POS=(-?\d+)\s+REL_STEPS=(-?\d+)',
            line
        )

        if match_home:
            actual_position = int(match_home.group(1))
            relative_steps = int(match_home.group(2))
            actual_deg = relative_steps / OUTPUT_STEPS_PER_DEG

            feedback.data = (
                f'STATUS=OK '
                f'MODE=HOME '
                f'ACT_DEG={actual_deg:.3f} '
                f'ACT_POS={actual_position} '
                f'REL_STEPS={relative_steps}'
            )

            self.feedback_pub.publish(feedback)

            self.get_logger().info(
                f'HOME complete: actual={actual_deg:.3f} deg'
            )
            return

        # POS:
        # OK POS ACT_POS=123456 REL_STEPS=200000
        match_pos = re.search(
            r'POS\s+ACT_POS=(-?\d+)\s+REL_STEPS=(-?\d+)',
            line
        )

        if match_pos:
            actual_position = int(match_pos.group(1))
            relative_steps = int(match_pos.group(2))
            actual_deg = relative_steps / OUTPUT_STEPS_PER_DEG

            feedback.data = (
                f'STATUS=OK '
                f'MODE=POS '
                f'ACT_DEG={actual_deg:.3f} '
                f'ACT_POS={actual_position} '
                f'REL_STEPS={relative_steps}'
            )

            self.feedback_pub.publish(feedback)

            self.get_logger().info(
                f'Position: {actual_deg:.3f} deg'
            )
            return

        # SAVE or any other OK line.
        feedback.data = line
        self.feedback_pub.publish(feedback)


def main(args=None):
    rclpy.init(args=args)

    node = STM32Bridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.ser.is_open:
            node.ser.close()

        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

