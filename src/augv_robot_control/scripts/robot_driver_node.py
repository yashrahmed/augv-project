import time

import rospy
from serial import Serial
from std_msgs.msg import String

ROS_RATE_HZ = 50
GPS_OUTPUT_TOPIC = '/my/raw/gps'
IMU_OUTPUT_TOPIC = '/my/raw/imu'
MOTOR_STATE_OUTPUT_TOPIC = '/my/raw/mot_state'
MOTOR_CMD_INPUT_TOPIC = '/my/cmd/motor'

def get_serial_conn():
    conn = Serial('/dev/ttyACM0', baudrate=9600)
    time.sleep(3)
    return conn


def route_serial_data(conn, imu_pub, gps_pub, mot_pub):
    line = conn.readline()
    try:
        payload = line.strip().decode('utf-8').replace(',nan', '')
        if len(payload) > 3:
            msg_type = payload[0:3]
            msg_body = payload[4:]
            if msg_type == 'IMU':
                imu_pub.publish(msg_body)
            elif msg_type == 'GPS':
                gps_pub.publish(msg_body)
            elif msg_type == 'MOT':
                mot_pub.publish(msg_body)
    except (UnicodeDecodeError, ValueError):
        return None

def send_cmd_to_robot(message, args):
    serial_conn = args
    #rospy.loginfo(f'robot_driver_node received cmd {message.data.encode()}')
    serial_conn.write(f'{message.data}\n'.encode())


def start_node():
    conn = get_serial_conn()
    rospy.init_node('demux_node', anonymous=True)

    gps_topic = rospy.get_param(
        f'{rospy.get_name()}/gps_output_topic', GPS_OUTPUT_TOPIC)
    gps_publisher = rospy.Publisher(gps_topic, String, queue_size=1000)

    imu_topic = rospy.get_param(
        f'{rospy.get_name()}/imu_output_topic', IMU_OUTPUT_TOPIC)
    imu_publisher = rospy.Publisher(imu_topic, String, queue_size=1000)

    motor_state_topic = rospy.get_param(
        f'{rospy.get_name()}/motor_state_output_topic', MOTOR_STATE_OUTPUT_TOPIC)
    motor_state_publisher = rospy.Publisher(motor_state_topic, String, queue_size=1000)

    motor_cmd_input_topic = rospy.get_param(
        f'{rospy.get_name()}/motor_cmd_input_topic', MOTOR_CMD_INPUT_TOPIC)
    rospy.Subscriber(motor_cmd_input_topic, String,
                     callback=send_cmd_to_robot, callback_args=(conn))

    rate_limiter = rospy.Rate(ROS_RATE_HZ)
    while not rospy.is_shutdown():
        route_serial_data(conn, imu_publisher, gps_publisher, motor_state_publisher)
        rate_limiter.sleep()


if __name__ == '__main__':
    try:
        start_node()
    except rospy.ROSInterruptException:
        pass
