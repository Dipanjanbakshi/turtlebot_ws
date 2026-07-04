#!/usr/bin/env python3


from collections import deque
from std_msgs.msg import Bool
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from rclpy.duration import Duration


class BatteryAverageNode(Node):

    def __init__(self):
        super().__init__('battery_average_node')

        self.avg_history = deque(maxlen=100)
        self.threshold_cross_time = None
        self.charging = False

        # Number of samples in moving average
        self.window_size = 150

        self.samples = deque(maxlen=self.window_size)

        self.subscription = self.create_subscription(
            Float32,
            '/battery_percentage',
            self.battery_callback,
            10
        )

        self.publisher = self.create_publisher(
            Float32,
            '/battery_percentage_avg',
            10
        )

        self.charging_pub = self.create_publisher(
            Bool,
            '/battery_status',
            10
        )

        self.get_logger().info(
            f'Moving average window size: {self.window_size}'
        )

    def battery_callback(self, battery_msg):

        self.samples.append(battery_msg.data)

        avg = sum(self.samples) / len(self.samples)

        # Store moving average history
        if not hasattr(self, 'avg_history'):
            self.avg_history = deque(maxlen=100)

        self.avg_history.append(avg)

        avg_msg = Float32()
        avg_msg.data = float(avg)
        self.publisher.publish(avg_msg)

        # Wait until history fills up
        if len(self.avg_history) >= self.avg_history.maxlen:

            old_avg = self.avg_history[0]
            delta = avg - old_avg

            # Charger connected
            if not self.charging and delta >= 3.0:
                self.charging = True
                self.get_logger().info(
                    f"CHARGER CONNECTED | "
                    f"Old Avg: {old_avg:.2f}% | "
                    f"Current Avg: {avg:.2f}% | "
                    f"Delta: {delta:.2f}%"
                )

            # Charger disconnected
            elif self.charging and delta <= -3.0:
                self.charging = False
                self.get_logger().info(
                    f"CHARGER DISCONNECTED | "
                    f"Old Avg: {old_avg:.2f}% | "
                    f"Current Avg: {avg:.2f}% | "
                    f"Delta: {delta:.2f}%"
                )

        charging_msg = Bool()
        charging_msg.data = self.charging
        self.charging_pub.publish(charging_msg)

        self.get_logger().info(
            f'Current: {battery_msg.data:.2f}% | '
            f'Average: {avg:.2f}% | '
            f'Charging: {self.charging}'
        )

def main(args=None):
    rclpy.init(args=args)

    node = BatteryAverageNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()