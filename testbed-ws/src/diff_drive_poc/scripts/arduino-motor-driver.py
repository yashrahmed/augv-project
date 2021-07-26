#!/usr/bin/env python3

from math import pi
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
        self.subscription = self.create_subscription(String, '/driver/motor_vel_cmd', self.listener_cb, 10)
    
    def listener_cb(self, msg):
        left_vel, right_vel = msg.data.split(' ')
        left_vel = (float(left_vel) / (2 * pi)) * 408
        right_vel = -(float(right_vel) / (2 * pi)) * 408
        self.serial_conn.write(f'{left_vel} {right_vel}\n'.encode())


if __name__ == '__main__':
    conn = get_serial_conn()
    rclpy.init(args=None)
    
    motor_vel_cmd_subscriber = MotorVelCmdSubscriber(conn)
    
    rclpy.spin(motor_vel_cmd_subscriber)

    motor_vel_cmd_subscriber.destroy_node()
    rclpy.shutdown()

