import sys
import time
from math import pi

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from tf_conversions import transformations as txms

INPUT_TOPIC = '/sensor/imu'
OUTPUT_TOPIC = '/mobile_base_controller/cmd_vel'
DEFAULT_THETA_Z_TOLERANCE = 0.005

set_point = pi / 2


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


def create_and_run_plan(imu_message, args):
    publisher, set_point, angle_tolerance = args
    quat = [imu_message.orientation.x,
            imu_message.orientation.y,
            imu_message.orientation.z,
            imu_message.orientation.w]
    _, _, az = txms.euler_from_quaternion(quat)
    out_msg = Twist()
    diff = set_point - az
    out_msg.angular.z = diff if abs(diff) >= angle_tolerance else 0
    publisher.publish(out_msg)


def start_node():
    rospy.init_node('custom_local_planner', anonymous=True)
    # input_topic = rospy.get_param(
    #     f'{rospy.get_name()}/input_topic', INPUT_TOPIC)
    # output_topic = rospy.get_param(
    #     f'{rospy.get_name()}/output_topic', OUTPUT_TOPIC)
    # theta_z_tolerance = rospy.get_param(
    #     f'{rospy.get_name()}/theta_z_tolerance', DEFAULT_THETA_Z_TOLERANCE)
    input_topic = INPUT_TOPIC
    output_topic = OUTPUT_TOPIC
    theta_z_tolerance = DEFAULT_THETA_Z_TOLERANCE
    publisher = rospy.Publisher(output_topic, Twist, queue_size=1000)
    rospy.Subscriber(input_topic, Imu,
                     callback=create_and_run_plan,
                     callback_args=(publisher,
                                    set_point,
                                    theta_z_tolerance))
    rospy.spin()


if __name__ == '__main__':
    try:
        start_node()
    except rospy.ROSInterruptException:
        pass
