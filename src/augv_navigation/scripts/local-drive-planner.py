import json
import sys
import time
from math import cos, pi, pow, sin, sqrt

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from tf_conversions import transformations as txms

CMD_VEL_TOPIC = '/mobile_base_controller/cmd_vel'
ANGLE_AND_DIST_INPUT_TOPIC = '/sensor/angle_and_dist'
STATUS_OUTPUT_TOPIC = '/drive_status'
COMMAND_TOPIC = '/set_point'

GOAL_STATUS_SUCCESS_MESSAGE = "AT_GOAL"

DEFAULT_THETA_Z_TOLERANCE = 0.005
DEFAULT_POS_TOLERANCE = 0.01
DEFAULT_DRIVE_MODE_THETA_Z_THRESHOLD = 0.044

set_point = {
    'az': 0.0,
    'dist': 0.0,
}

current_state = {
    'reached_goal': True,
    'az': 0.0,
    'dist': 0.0,
    'total_dist': 0.0
}


def execute_drive_command(cmd_publisher, status_publisher,
                          tgt_state, pos_tolerance, angle_tolerance):
    out_msg = Twist()
    az_diff = compute_turn(tgt_state['az'], current_state['az'])
    pos_diff = set_point['dist'] - current_state['dist']
    if abs(az_diff) >= DEFAULT_DRIVE_MODE_THETA_Z_THRESHOLD:
        out_msg.angular.z = az_diff
    else:
        out_msg.linear.x = pos_diff if abs(pos_diff) >= pos_tolerance else 0.0
        out_msg.angular.z = az_diff if abs(az_diff) >= angle_tolerance else 0.0

    # Check if goal reached and then reset dist
    # Required as distance is relative.
    if abs(pos_diff) <= pos_tolerance and abs(az_diff) <= angle_tolerance:
        current_state['dist'] = 0.0
        set_point['dist'] = 0.0
        if not current_state['reached_goal']:
            status_publisher.publish(GOAL_STATUS_SUCCESS_MESSAGE)
            current_state['reached_goal'] = True

    cmd_publisher.publish(out_msg)


def compute_turn(a1, a2):
    diff = a1 - a2
    return denormalize_angle(diff)


def denormalize_angle(a):
    return a if a <= pi else a - 2*pi


def init_timestamps():
    current_timestamp = rospy.Time.now()
    current_state['last_angle_update_time'] = current_timestamp
    current_state['last_odom_update_time'] = current_timestamp


def normalize_angle(a):
    return a if a >= 0 else a + 2*pi


def update_state(message):
    try:
        theta, traveled_dist = [float(v) for v in message.data.split(",")]
        current_state['az'] = theta
        current_state['dist'] += traveled_dist - current_state['total_dist']
        current_state['total_dist'] = traveled_dist
    except Exception as ex:
        rospy.logerr(f"angle and distance updated failed. exception={ex}")


def update_set_point(command):
    try:
        cmd_values = [float(v) for v in command.data.split(",")]
        set_point['az'], set_point['dist'] = cmd_values
        set_point['az'] = denormalize_angle(set_point['az'])
        current_state['reached_goal'] = False
        rospy.loginfo(f"Updated set point to {set_point}")
    except Exception as ex:
        rospy.logerr(f"Received illegal command {command} exception={ex}")


def start_node():
    rospy.init_node('custom_local_planner', anonymous=True)
    init_timestamps()
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
    cmd_vel_topic = CMD_VEL_TOPIC
    command_topic = COMMAND_TOPIC
    status_output_topic = STATUS_OUTPUT_TOPIC
    angle_and_dist_state_topic = ANGLE_AND_DIST_INPUT_TOPIC

    theta_z_tolerance = DEFAULT_THETA_Z_TOLERANCE
    xy_tolerance = DEFAULT_POS_TOLERANCE

    cmd_vel_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size=1000)
    status_publisher = rospy.Publisher(
        status_output_topic, String, queue_size=10)
    rospy.Subscriber(command_topic, String,
                     callback=update_set_point)
    rospy.Subscriber(angle_and_dist_state_topic, String,
                     callback=update_state)
    rospy.Timer(rospy.Duration(0.05),
                callback=lambda _: execute_drive_command(cmd_vel_publisher,
                                                         status_publisher,
                                                         set_point,
                                                         xy_tolerance,
                                                         theta_z_tolerance))

    rospy.spin()


if __name__ == '__main__':
    try:
        start_node()
    except rospy.ROSInterruptException:
        pass
