import sys
import time

import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import String

LIN_ACC_COV = 1e-4 ** 2  # 0.01m/s^2
# errors have to be made high to compensate for sensor lag
ANG_VEL_COV = 0.025 ** 2
ORIENT_COV = 0.025 ** 2
ROS_RATE_HZ = 50

INPUT_TOPIC = '/input'
OUTPUT_TOPIC = '/data'
FRAME_ID = 'my_base_link'


def extract_imu_data(message):
    try:
        # First part of the message is "IMU "
        vals = message.split(',')
        if len(vals) == 10:
            return [float(v) for v in vals]
        else:
            return None
    except (UnicodeDecodeError, IndexError, ValueError):
        return None


def init_imu_message(frame_id) -> Imu:
    imu_msg = Imu()
    imu_msg.header.frame_id = frame_id
    imu_msg.header.stamp = rospy.get_rostime()

    imu_msg.linear_acceleration_covariance[0] = LIN_ACC_COV
    imu_msg.linear_acceleration_covariance[4] = LIN_ACC_COV
    imu_msg.linear_acceleration_covariance[8] = LIN_ACC_COV

    imu_msg.angular_velocity_covariance[0] = ANG_VEL_COV
    imu_msg.angular_velocity_covariance[4] = ANG_VEL_COV
    imu_msg.angular_velocity_covariance[8] = ANG_VEL_COV

    imu_msg.orientation_covariance[0] = ORIENT_COV
    imu_msg.orientation_covariance[4] = ORIENT_COV
    imu_msg.orientation_covariance[8] = ORIENT_COV

    return imu_msg


def create_imu_message(orientation_data, frame_id):
    message = init_imu_message(frame_id)
    x, y, z, w, ang_x_vel, ang_y_vel, ang_z_vel, acc_x_lin, acc_y_lin, acc_z_lin = orientation_data
    message.orientation.x = x
    message.orientation.y = y
    message.orientation.z = z
    message.orientation.w = w
    message.angular_velocity.x = ang_x_vel
    message.angular_velocity.y = ang_y_vel
    message.angular_velocity.z = ang_z_vel
    message.linear_acceleration.x = acc_x_lin
    message.linear_acceleration.y = acc_y_lin
    message.linear_acceleration.z = acc_z_lin
    return message


def publish_imu_data(message, args):
    publisher, frame_id = args
    orientation_data = extract_imu_data(message.data)
    if orientation_data:
        publisher.publish(create_imu_message(orientation_data, frame_id))


def start_node():
    rospy.init_node('imu_sensor_node', anonymous=True)
    frame_id = rospy.get_param(
        f'{rospy.get_name()}/frame_id', FRAME_ID)
    input_topic = rospy.get_param(
        f'{rospy.get_name()}/input_topic', INPUT_TOPIC)
    output_topic = rospy.get_param(
        f'{rospy.get_name()}/output_topic', OUTPUT_TOPIC)
    publisher = rospy.Publisher(output_topic, Imu, queue_size=1000)
    rospy.Subscriber(input_topic, String,
                     callback=publish_imu_data, callback_args=(publisher, frame_id))
    rospy.spin()


if __name__ == '__main__':
    try:
        start_node()
    except rospy.ROSInterruptException:
        pass
