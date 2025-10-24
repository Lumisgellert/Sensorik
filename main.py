#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, String
import serial


class ServoController(Node):
    def __init__(self):
        super().__init__('servo_controller')

        # Serial-Verbindung (Port anpassen!)
        self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)

        # Subscriber für Servo-Befehle
        self.subscription = self.create_subscription(
            Int32,
            'servo_angle',
            self.angle_callback,
            10)

        # Publisher für Feedback
        self.publisher = self.create_publisher(String, 'servo_feedback', 10)

        # Timer zum Lesen von Arduino-Nachrichten
        self.timer = self.create_timer(0.1, self.read_serial)

    def angle_callback(self, msg):
        angle = msg.data
        self.ser.write(f"{angle}\n".encode())
        self.get_logger().info(f'Sende Winkel: {angle}°')

    def read_serial(self):
        if self.ser.in_waiting > 0:
            feedback = self.ser.readline().decode().strip()
            msg = String()
            msg.data = feedback
            self.publisher.publish(msg)
            self.get_logger().info(f'Arduino: {feedback}')


def main(args=None):
    rclpy.init(args=args)
    node = ServoController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()