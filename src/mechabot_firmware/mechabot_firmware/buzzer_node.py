#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
from geometry_msgs.msg import Twist
import RPi.GPIO as GPIO
from collections import deque

BUZZER_PIN = 18


class BuzzerNode(Node):
    def __init__(self):
        super().__init__('buzzer_node')

        # GPIO Setup
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(BUZZER_PIN, GPIO.OUT)
        self.pwm = GPIO.PWM(BUZZER_PIN, 1000)
        self.pwm.start(0)

        # Queue: (type, freq, duration, volume)
        self.tone_queue = deque()

        # Timer for non-blocking playback
        self.timer = self.create_timer(0.05, self.process_queue)

        # Playback state
        self.current_item = None
        self.time_left = 0.0

        # Subscriptions
        self.create_subscription(String, '/buzzer_command', self.cmd_callback, 10)
        self.create_subscription(Float32, '/battery_status', self.battery_callback, 10)
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        # Internal states
        self.low_battery_triggered = False
        self.last_motion_state = False

        self.get_logger().info("🔊 Smart Buzzer Node Started")

    # =========================
    # 🔊 Playback Engine
    # =========================

    def process_queue(self):
        # Load next item
        if self.current_item is None and self.tone_queue:
            self.current_item = self.tone_queue.popleft()
            item_type, freq, duration, volume = self.current_item

            if item_type == "tone":
                self.pwm.ChangeFrequency(freq)
                self.pwm.ChangeDutyCycle(volume)
            elif item_type == "pause":
                self.pwm.ChangeDutyCycle(0)

            self.time_left = duration

        # Process current item
        elif self.current_item:
            self.time_left -= 0.05

            if self.time_left <= 0:
                self.pwm.ChangeDutyCycle(0)
                self.current_item = None

    def add_tone(self, freq, duration, volume=50):
        self.tone_queue.append(("tone", freq, duration, volume))

    def add_pause(self, duration):
        self.tone_queue.append(("pause", 0, duration, 0))

    def clear_queue(self):
        self.tone_queue.clear()
        self.current_item = None
        self.pwm.ChangeDutyCycle(0)

    # =========================
    # 🎼 Sound Patterns
    # =========================

    def startup_melody(self):
        self.clear_queue()
        self.add_tone(800, 0.15)
        self.add_tone(1200, 0.15)
        self.add_tone(1500, 0.25)

    def low_battery_alert(self):
        self.clear_queue()
        for _ in range(2):
            self.add_tone(600, 0.3, 40)
            self.add_pause(0.4)

    def critical_battery_alert(self):
        self.clear_queue()
        for _ in range(5):
            self.add_tone(400, 0.15, 80)
            self.add_pause(0.15)

    def goal_reached(self):
        self.clear_queue()
        self.add_tone(1200, 0.1)
        self.add_tone(1500, 0.1)

    def motion_beep(self):
        # Do NOT clear queue → low priority
        self.add_tone(1000, 0.05, 20)

    def mario_tune(self):
        self.clear_queue()
        notes = [660, 660, 0, 660, 0, 520, 660, 0, 780]

        for n in notes:
            if n == 0:
                self.add_pause(0.1)
            else:
                self.add_tone(n, 0.1)

    # =========================
    # 📡 Callbacks
    # =========================

    def cmd_callback(self, msg):
        cmd = msg.data

        if cmd == "startup":
            self.startup_melody()

        elif cmd == "low_battery":
            self.low_battery_alert()

        elif cmd == "critical":
            self.critical_battery_alert()

        elif cmd == "goal":
            self.goal_reached()

        elif cmd == "mario":
            self.mario_tune()

        elif cmd == "stop":
            self.clear_queue()

    def battery_callback(self, msg):
        voltage = msg.data

        # Tune these thresholds for your battery
        if voltage < 10.5:
            self.critical_battery_alert()

        elif voltage < 11.2:
            if not self.low_battery_triggered:
                self.low_battery_alert()
                self.low_battery_triggered = True

        else:
            self.low_battery_triggered = False

    def cmd_vel_callback(self, msg):
        moving = abs(msg.linear.x) > 0.01 or abs(msg.angular.z) > 0.01

        if moving and not self.last_motion_state:
            self.motion_beep()

        self.last_motion_state = moving

    # =========================

    def destroy_node(self):
        GPIO.cleanup()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = BuzzerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()