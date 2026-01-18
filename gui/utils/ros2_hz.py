#!/usr/bin/env python3
"""ROS2 토픽 Hz 측정 헬퍼 스크립트 - rclpy로 직접 측정"""
import sys
import os
import subprocess
import time

DEBUG = os.environ.get('HZ_DEBUG', '0') == '1'

def debug_print(msg):
    if DEBUG:
        print(f"[DEBUG] {msg}", file=sys.stderr)

def setup_ros2_env():
    """ROS2 환경 변수 설정"""
    ros2_setup = '/opt/ros/humble/setup.bash'
    if os.path.exists(ros2_setup):
        result = subprocess.run(
            ['bash', '-c', f'source {ros2_setup} && env'],
            capture_output=True, text=True
        )
        for line in result.stdout.split('\n'):
            if '=' in line:
                key, _, value = line.partition('=')
                os.environ[key] = value
        debug_print(f"ROS2 env loaded, AMENT_PREFIX_PATH={os.environ.get('AMENT_PREFIX_PATH', 'NOT SET')}")
    else:
        debug_print(f"ROS2 setup file not found: {ros2_setup}")

# ROS2 환경 설정
setup_ros2_env()

# rclpy import
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    HAS_RCLPY = True
    debug_print("rclpy imported successfully")
except ImportError as e:
    HAS_RCLPY = False
    Node = object  # Fallback for class definition
    debug_print(f"rclpy import failed: {e}")


class HzMeasurer(Node):
    def __init__(self, topic_name, duration=2.0):
        if not HAS_RCLPY:
            self.rate = None
            return

        super().__init__('hz_measurer')
        self.timestamps = []
        self.duration = duration
        self.start_time = time.time()
        self.topic_name = topic_name

        # QoS 설정 - Best Effort로 FC 토픽 수신
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # 동적으로 토픽 타입 가져오기
        topic_types = self.get_topic_names_and_types()
        debug_print(f"Found {len(topic_types)} topics")
        topic_type = None
        for name, types in topic_types:
            if name == topic_name and types:
                topic_type = types[0]
                break

        if not topic_type:
            debug_print(f"Topic {topic_name} not found in topic list")
            self.rate = None
            return

        debug_print(f"Topic type: {topic_type}")

        # 메시지 타입 import
        try:
            parts = topic_type.split('/')
            if len(parts) == 3:
                module = __import__(f'{parts[0]}.{parts[1]}', fromlist=[parts[2]])
                msg_class = getattr(module, parts[2])

                self.subscription = self.create_subscription(
                    msg_class,
                    topic_name,
                    self.callback,
                    qos
                )
                debug_print(f"Subscription created for {topic_name}")
                self.rate = None
            else:
                self.rate = None
        except Exception as e:
            debug_print(f"Failed to create subscription: {e}")
            self.rate = None

    def callback(self, msg):
        self.timestamps.append(time.time())

    def calculate_rate(self):
        debug_print(f"Received {len(self.timestamps)} messages")
        if len(self.timestamps) >= 2:
            total_time = self.timestamps[-1] - self.timestamps[0]
            if total_time > 0:
                return (len(self.timestamps) - 1) / total_time
        return None


def measure_hz(topic_name, duration=2.0):
    if not HAS_RCLPY:
        debug_print("rclpy not available")
        return None

    try:
        rclpy.init()
        debug_print("rclpy initialized")
        node = HzMeasurer(topic_name, duration)

        start = time.time()
        while time.time() - start < duration:
            rclpy.spin_once(node, timeout_sec=0.1)

        rate = node.calculate_rate()
        node.destroy_node()
        rclpy.shutdown()
        debug_print(f"Measured rate: {rate}")
        return rate
    except Exception as e:
        debug_print(f"Exception: {e}")
        try:
            rclpy.shutdown()
        except:
            pass
        return None


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("null")
        sys.exit(1)

    topic = sys.argv[1]
    duration = float(sys.argv[2]) if len(sys.argv) > 2 else 2.0

    rate = measure_hz(topic, duration)

    if rate is not None:
        print(f"{rate:.3f}")
    else:
        print("null")
