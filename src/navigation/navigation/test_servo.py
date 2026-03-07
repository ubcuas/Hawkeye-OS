#!/usr/bin/env python3
"""Test script for /drone/set_servo topic"""

import rclpy
from std_msgs.msg import Float32MultiArray
import time


def test_servo():
    rclpy.init()
    node = rclpy.create_node('servo_test')
    pub = node.create_publisher(Float32MultiArray, '/drone/set_servo', 10)

    test_cases = [
        (9, 1000, "Servo 9 - PWM 1000 (min)"),
        (9, 1500, "Servo 9 - PWM 1500 (neutral)"),
        (9, 2000, "Servo 9 - PWM 2000 (max)"),
        (10, 1500, "Servo 10 - PWM 1500"),
    ]

    print("Testing /drone/set_servo topic...")
    print("Make sure navigation node is running!")
    time.sleep(2)

    for servo_num, pwm, description in test_cases:
        msg = Float32MultiArray(data=[float(servo_num), float(pwm)])

        print(f"Publishing: {description}")
        pub.publish(msg)
        time.sleep(1)

    print("Done!")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    test_servo()
