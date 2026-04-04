#!/usr/bin/env python3
# Publishes sample /drone/set_servo messages. ros2 run drone_control test_servo
# Requires drone_control (or equivalent) subscribed to /drone/set_servo.

import time

import rclpy
from std_msgs.msg import Float32MultiArray

_LOG_RULE = "=================================================="


def test_servo():
    rclpy.init()
    node = rclpy.create_node("servo_test")
    log = node.get_logger()

    # ==========================
    # Tunable parameters
    # ==========================
    test_cases = [
        (9, 1000, "Servo 9 PWM 1000 (min)"),
        (9, 1500, "Servo 9 PWM 1500 (neutral)"),
        (9, 2000, "Servo 9 PWM 2000 (max)"),
        (10, 1500, "Servo 10 PWM 1500"),
    ]

    # Command Publisher
    pub = node.create_publisher(Float32MultiArray, "/drone/set_servo", 10)

    log.info(
        "servo_test: started. Publishing /drone/set_servo cases. Ensure drone_control is running."
    )
    time.sleep(2)

    n = len(test_cases)
    for i, (servo_num, pwm, description) in enumerate(test_cases, start=1):
        log.info(_LOG_RULE)
        log.info(f"TEST STEP {i}/{n} STARTED (servo)")
        log.info(f"Name: {description}")
        log.info(f"Expect: drone_control receives [servo={servo_num}, pwm={pwm}]")
        log.info(_LOG_RULE)
        pub.publish(Float32MultiArray(data=[float(servo_num), float(pwm)]))
        time.sleep(1)
        log.info(f"TEST PASS: step {i}/{n} — published")

    log.info(_LOG_RULE)
    log.info(f"TEST PASS: completed all {n} servo publish steps.")
    log.info(_LOG_RULE)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    test_servo()
