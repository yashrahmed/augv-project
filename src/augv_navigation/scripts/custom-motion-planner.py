import sys
import time
from math import pi

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from tf_conversions import transformations as txms

IMU_INPUT_TOPIC = '/sensor/imu'
OUTPUT_TOPIC = '/mobile_base_controller/cmd_vel'
DEFAULT_THETA_Z_TOLERANCE = 0.005

set_point = pi / 2

current_state = {
    'az': 0.0,
    'x': 0.0,
    'y': 0.0
}


def create_and_run_plan(publisher, angle_tgt, angle_tolerance):
    out_msg = Twist()
    diff = angle_tgt - current_state['az']
    out_msg.angular.z = diff if abs(diff) >= angle_tolerance else 0
    publisher.publish(out_msg)


def update_imu_state(imu_message):
    quat = [imu_message.orientation.x,
            imu_message.orientation.y,
            imu_message.orientation.z,
            imu_message.orientation.w]
    _, _, az = txms.euler_from_quaternion(quat)
    current_state['az'] = az


def start_node():
    rospy.init_node('custom_local_planner', anonymous=True)
    # input_topic = rospy.get_param(
    #     f'{rospy.get_name()}/input_topic', INPUT_TOPIC)
    # output_topic = rospy.get_param(
    #     f'{rospy.get_name()}/output_topic', OUTPUT_TOPIC)
    # theta_z_tolerance = rospy.get_param(
    #     f'{rospy.get_name()}/theta_z_tolerance', DEFAULT_THETA_Z_TOLERANCE)
    input_topic = IMU_INPUT_TOPIC
    output_topic = OUTPUT_TOPIC
    theta_z_tolerance = DEFAULT_THETA_Z_TOLERANCE
    publisher = rospy.Publisher(output_topic, Twist, queue_size=1000)
    rospy.Subscriber(input_topic, Imu,
                     callback=update_imu_state)
    rospy.Timer(rospy.Duration(0.05),
                callback=lambda _: create_and_run_plan(publisher,
                                                       set_point,
                                                       theta_z_tolerance))
    rospy.spin()


if __name__ == '__main__':
    try:
        start_node()
    except rospy.ROSInterruptException:
        pass
