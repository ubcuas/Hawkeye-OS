#!/usr/bin/env python3
"""Test script for /drone/set_servo topic"""

import rclpy
from geometry_msgs.msg import Point
import time


def test_servo():
    rclpy.init()
    node = rclpy.create_node('servo_test')
    pub = node.create_publisher(Point, '/drone/set_servo', 10)
    
    test_cases = [
        (9, 1000.0, "Servo 9 - PWM 1000 (min)"),
        (9, 1500.0, "Servo 9 - PWM 1500 (neutral)"),
        (9, 2000.0, "Servo 9 - PWM 2000 (max)"),
        (10, 1500.0, "Servo 10 - PWM 1500"),
    ]
    
    print("Testing /drone/set_servo topic...")
    print("Make sure navigation node is running!")
    time.sleep(2)
    
    for servo_num, pwm, description in test_cases:
        msg = Point()
        msg.x = float(servo_num)
        msg.y = float(pwm)
        
        print(f"Publishing: {description}")
        pub.publish(msg)
        time.sleep(1)
    
    print("Done!")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    test_servo()
