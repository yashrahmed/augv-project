#!/usr/bin/env python3

from time import sleep

import rclpy
from rclpy.node import Node
from serial import Serial
from std_msgs.msg import String


def get_serial_conn():
    conn = Serial('/dev/ttyACM0', baudrate=9600, timeout=3)
    sleep(3)
    return conn

class MotorVelCmdSubscriber(Node):
    def __init__(self, serial_conn):
        super().__init__('motor_vel_cmd_subscriber')
        self.serial_conn = serial_conn
        self.subscription = self.create_subscription(String, '/driver/motor_vel_cmd', self.listener_cb, 100)
    
    def listener_cb(self, msg):
        print(msg.data)
        self.serial_conn.write(f'{msg.data}'.encode())


if __name__ == '__main__':
    conn = get_serial_conn()
    rclpy.init(args=None)
    
    motor_vel_cmd_subscriber = MotorVelCmdSubscriber(conn)
    
    rclpy.spin(motor_vel_cmd_subscriber)

    motor_vel_cmd_subscriber.destroy_node()
    rclpy.shutdown()

