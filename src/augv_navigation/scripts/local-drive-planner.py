import sys
import time
from math import pi, pow, sqrt

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from tf_conversions import transformations as txms

IMU_INPUT_TOPIC = '/sensor/imu'
ODOM_INPUT_TOPIC = '/sensor/odom'
OUTPUT_TOPIC = '/mobile_base_controller/cmd_vel'
DEFAULT_THETA_Z_TOLERANCE = 0.005
DEFAULT_POS_TOLERANCE = 0.01
DEFAULT_DRIVE_MODE_THETA_Z_THRESHOLD = 0.044

set_point = {
    # 'az': pi / 2,
    'az': 0.0,
    'dist': 0.5,
    'y': 0.0
}

current_state = {
    'az': 0.0,
    'x': 0.0,
    'y': 0.0,
    'dist' : 0.0
}


def create_and_run_plan(publisher, tgt_state, pos_tolerance, angle_tolerance):
    out_msg = Twist()
    az_diff = tgt_state['az'] - current_state['az']
    pos_diff = set_point['dist'] - current_state['dist']
    print(pos_diff)
    if abs(az_diff) >= DEFAULT_DRIVE_MODE_THETA_Z_THRESHOLD:
        out_msg.angular.z = az_diff
    else:
        out_msg.linear.x = pos_diff if abs(pos_diff) >= pos_tolerance else 0.0
        out_msg.angular.z = az_diff if abs(az_diff) >= angle_tolerance else 0.0
    publisher.publish(out_msg)


def update_imu_state(imu_message):
    quat = [imu_message.orientation.x,
            imu_message.orientation.y,
            imu_message.orientation.z,
            imu_message.orientation.w]
    _, _, az = txms.euler_from_quaternion(quat)
    current_state['az'] = az


def update_odom_state(odom_message):
    msg_position = odom_message.pose.pose.position
    current_state['dist'] += sqrt(pow(current_state['x'] - msg_position.x, 2)
                    + pow(current_state['y'] - msg_position.y, 2))
    current_state['x'] = msg_position.x
    current_state['y'] = msg_position.y


def start_node():
    rospy.init_node('custom_local_planner', anonymous=True)
    # imu_input_topic = rospy.get_param(
    #     f'{rospy.get_name()}/imu_input_topic', IMU_INPUT_TOPIC)
    # odom_input_topic = rospy.get_param(
    #     f'{rospy.get_name()}/odom_input_topic', ODOM_INPUT_TOPIC)
    # output_topic = rospy.get_param(
    #     f'{rospy.get_name()}/output_topic', OUTPUT_TOPIC)
    # theta_z_tolerance = rospy.get_param(
    #     f'{rospy.get_name()}/theta_z_tolerance', DEFAULT_THETA_Z_TOLERANCE)
    # xy_tolerance = rospy.get_param(
    #     f'{rospy.get_name()}/xy_tolerance', DEFAULT_POS_TOLERANCE)
    imu_input_topic = IMU_INPUT_TOPIC
    odom_input_topic = ODOM_INPUT_TOPIC
    output_topic = OUTPUT_TOPIC
    theta_z_tolerance = DEFAULT_THETA_Z_TOLERANCE
    xy_tolerance = DEFAULT_POS_TOLERANCE
    publisher = rospy.Publisher(output_topic, Twist, queue_size=1000)
    rospy.Subscriber(imu_input_topic, Imu,
                     callback=update_imu_state)
    rospy.Subscriber(odom_input_topic, Odometry,
                     callback=update_odom_state)
    rospy.Timer(rospy.Duration(0.05),
                callback=lambda _: create_and_run_plan(publisher,
                                                       set_point,
                                                       xy_tolerance,
                                                       theta_z_tolerance))
    rospy.spin()


if __name__ == '__main__':
    try:
        start_node()
    except rospy.ROSInterruptException:
        pass
