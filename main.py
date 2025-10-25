#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, String
import serial
import time


class ServoController(Node):
    def __init__(self):
        super().__init__('servo_controller')

        # Serial-Verbindung mit kürzerem timeout
        self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=0.1)
        time.sleep(2)  # Warte bis Arduino bereit ist
        self.ser.reset_input_buffer()  # Buffer leeren

        # Subscriber für Servo-Befehle
        self.subscription = self.create_subscription(
            Int32,
            'servo_angle',
            self.angle_callback,
            10)

        # Publisher für Feedback
        self.publisher = self.create_publisher(String, 'servo_feedback', 10)

        # Schnellerer Timer (20ms statt 100ms)
        self.timer = self.create_timer(0.02, self.read_serial)

        self.get_logger().info('Servo Controller bereit')

    def angle_callback(self, msg):
        t1 = time.time()
        angle = msg.data
        self.ser.write(f"{angle}\n".encode())
        self.ser.flush()  # Sende sofort
        t2 = time.time()
        self.get_logger().info(f'Sende Winkel: {angle}° (Latenz: {(t2 - t1) * 1000:.1f}ms)')

    def read_serial(self):
        while self.ser.in_waiting > 0:  # Alle verfügbaren Nachrichten lesen
            try:
                feedback = self.ser.readline().decode().strip()
                if feedback:
                    msg = String()
                    msg.data = feedback
                    self.publisher.publish(msg)
                    self.get_logger().info(f'Arduino: {feedback}')
            except:
                pass


def main(args=None):
    rclpy.init(args=args)
    node = ServoController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()