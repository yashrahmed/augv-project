import json
import sys
import time
from math import cos, pi, pow, sin, sqrt

import rospy
from geometry_msgs.msg import Pose, PoseWithCovariance, Twist, TwistWithCovariance
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from tf_conversions import transformations as txms

POSE_COV_MATRIX = [[0.05, 0, 0, 0, 0, 0],
                   [0, 0.05, 0, 0, 0, 0],
                   [0, 0, 1000, 0, 0, 0],
                   [0, 0, 0, 1000, 0, 0],
                   [0, 0, 0, 0, 1000, 0],
                   [0, 0, 0, 0, 0, 0.005]]
TWIST_COV_MATRIX = [[0.05, 0, 0, 0, 0, 0],
                    [0, 0.05, 0, 0, 0, 0],
                    [0, 0, 1000, 0, 0, 0],
                    [0, 0, 0, 1000, 0, 0],
                    [0, 0, 0, 0, 1000, 0],
                    [0, 0, 0, 0, 0, 0.005]]
POSE_COV_MATRIX_RMAJ = [y for x in POSE_COV_MATRIX for y in x]
TWIST_COV_MATRIX_RMAJ = [y for x in TWIST_COV_MATRIX for y in x]


IMU_INPUT_TOPIC = '/sensor/imu'
ODOM_INPUT_TOPIC = '/sensor/odom'
ODOM_OUTPUT_TOPIC = '/odometry/raw'
CMD_VEL_TOPIC = '/mobile_base_controller/cmd_vel'
STATUS_OUTPUT_TOPIC = '/drive_status'
COMMAND_TOPIC = '/set_point'
ODOM_FRAME_ID = 'odom'
BASE_FRAME_ID = 'base_link'

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
    'x': 0.0,
    'y': 0.0,
    'dist': 0.0,
    'odom': {
        'x': 0.0,
        'y': 0.0,
        'tz': 0.0,
        'vx': 0.0,
        'vy': 0.0,
        'wz': 0.0
    },
    'last_angle_update_time': 0,
    'last_odom_update_time': 0
}


def execute_drive_command(cmd_publisher, status_publisher, odom_output_publisher,
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
    odom_output_publisher.publish(compute_odometry_from_current_state())


def compute_turn(a1, a2):
    diff = a1 - a2
    return denormalize_angle(diff)


def compute_odometry_from_current_state():
    message = Odometry()
    message.header.stamp = rospy.Time.now()
    message.header.frame_id = ODOM_FRAME_ID
    message.child_frame_id = BASE_FRAME_ID

    pose_with_cov = PoseWithCovariance()

    pose = Pose()
    orient_val = pose.orientation
    pose.position.x = current_state['odom']['x']
    pose.position.y = current_state['odom']['y']
    orient_val.x, orient_val.y, orient_val.z, orient_val.w = txms.quaternion_from_euler(
        0, 0, current_state['odom']['tz'])
    pose_with_cov.pose = pose
    pose_with_cov.covariance = POSE_COV_MATRIX_RMAJ

    twist_with_covariance = TwistWithCovariance()

    twist = Twist()
    twist.linear.x = current_state['odom']['vx']
    twist.linear.y = current_state['odom']['vy']
    twist.angular.z = current_state['odom']['wz']

    twist_with_covariance.twist = twist
    twist_with_covariance.covariance = TWIST_COV_MATRIX_RMAJ

    message.twist = twist_with_covariance
    message.pose = pose_with_cov
    return message


def denormalize_angle(a):
    return a if a <= pi else a - 2*pi


def init_timestamps():
    current_timestamp = rospy.Time.now()
    current_state['last_angle_update_time'] = current_timestamp
    current_state['last_odom_update_time'] = current_timestamp


def normalize_angle(a):
    return a if a >= 0 else a + 2*pi


def update_imu_state(imu_message):
    current_timestamp = rospy.Time.now()
    delta_t = (current_timestamp - current_state['last_angle_update_time']).to_sec()
    quat = [imu_message.orientation.x,
            imu_message.orientation.y,
            imu_message.orientation.z,
            imu_message.orientation.w]
    _, _, az = txms.euler_from_quaternion(quat)
    current_state['az'] = az
    current_state['odom']['tz'] = az
    current_state['odom']['wz'] = az / delta_t
    current_state['last_angle_update_time'] = current_timestamp


def update_odom_state(odom_message):
    current_timestamp = rospy.Time.now()
    delta_t = (current_timestamp - current_state['last_odom_update_time']).to_sec()
    msg_position = odom_message.pose.pose.position
    dist_traveled = sqrt(pow(current_state['x'] - msg_position.x, 2)
                         + pow(current_state['y'] - msg_position.y, 2))
    x_dist = dist_traveled * cos(current_state['az'])
    y_dist = dist_traveled * sin(current_state['az'])
    current_state['odom']['vx'] = x_dist / delta_t
    current_state['odom']['vy'] = y_dist / delta_t
    current_state['odom']['x'] += x_dist
    current_state['odom']['y'] += y_dist
    current_state['dist'] += dist_traveled
    current_state['x'] = msg_position.x
    current_state['y'] = msg_position.y
    current_state['last_odom_update_time'] = current_timestamp


def update_set_point(command):
    print(command)
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
    imu_input_topic = IMU_INPUT_TOPIC
    odom_input_topic = ODOM_INPUT_TOPIC
    odom_output_topic = ODOM_OUTPUT_TOPIC
    cmd_vel_topic = CMD_VEL_TOPIC
    command_topic = COMMAND_TOPIC
    status_output_topic = STATUS_OUTPUT_TOPIC

    theta_z_tolerance = DEFAULT_THETA_Z_TOLERANCE
    xy_tolerance = DEFAULT_POS_TOLERANCE

    cmd_vel_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size=1000)
    odom_output_publisher = rospy.Publisher(
        odom_output_topic, Odometry, queue_size=1000)
    status_publisher = rospy.Publisher(
        status_output_topic, String, queue_size=10)
    rospy.Subscriber(command_topic, String,
                     callback=update_set_point)
    rospy.Subscriber(imu_input_topic, Imu,
                     callback=update_imu_state)
    rospy.Subscriber(odom_input_topic, Odometry,
                     callback=update_odom_state)
    rospy.Timer(rospy.Duration(0.05),
                callback=lambda _: execute_drive_command(cmd_vel_publisher,
                                                         status_publisher,
                                                         odom_output_publisher,
                                                         set_point,
                                                         xy_tolerance,
                                                         theta_z_tolerance))

    rospy.spin()


if __name__ == '__main__':
    try:
        start_node()
    except rospy.ROSInterruptException:
        pass
