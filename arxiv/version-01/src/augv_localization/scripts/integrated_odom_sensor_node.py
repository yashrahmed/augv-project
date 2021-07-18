import json
import sys
import time
from math import cos, pi, pow, sin, sqrt

import rospy
from geometry_msgs.msg import (Pose, PoseWithCovariance, Twist,
                               TwistWithCovariance)
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


IMU_INPUT_TOPIC = '/my/sensor/imu'
ODOM_INPUT_TOPIC = '/my/sensor/odom'
ODOM_OUTPUT_TOPIC = '/my/odometry/raw'
ANGLE_DIST_OUTPUT_TOPIC = '/my/sensor/angle_and_dist'
ODOM_FRAME_ID = 'my_odom'
BASE_FRAME_ID = 'my_base_link'
DEFAULT_RATE_HZ = 20


current_state = {
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


def compute_odometry_from_current_state(odom_publisher, angle_dist_publisher, base_frame_id, odom_frame_id):
    message = Odometry()
    message.header.stamp = rospy.Time.now()
    message.header.frame_id = odom_frame_id
    message.child_frame_id = base_frame_id

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
    odom_publisher.publish(message)
    angle_dist_publisher.publish(
        f"{current_state['az']},{current_state['dist']}")


def denormalize_angle(a):
    return a if a <= pi else a - 2*pi


def init_timestamps():
    current_timestamp = rospy.Time.now()
    current_state['last_angle_update_time'] = current_timestamp
    current_state['last_odom_update_time'] = current_timestamp


def update_imu_state(imu_message):
    current_timestamp = rospy.Time.now()
    delta_t = (current_timestamp -
               current_state['last_angle_update_time']).to_sec()
    quat = [imu_message.orientation.x,
            imu_message.orientation.y,
            imu_message.orientation.z,
            imu_message.orientation.w]
    _, _, az = txms.euler_from_quaternion(quat)
    delta_z = denormalize_angle(az - current_state['az'])
    current_state['az'] = current_state['odom']['tz'] = az
    current_state['odom']['wz'] = delta_z / delta_t
    current_state['last_angle_update_time'] = current_timestamp

# ODOM is computed incorrectly if the vehicle is in reverse!!!!
def update_odom_state(odom_message):
    current_timestamp = rospy.Time.now()
    delta_t = (current_timestamp -
               current_state['last_odom_update_time']).to_sec()
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


def start_node():
    rospy.init_node('custom_local_planner', anonymous=True)
    init_timestamps()
    imu_input_topic = rospy.get_param(
        f'{rospy.get_name()}/imu_input_topic', IMU_INPUT_TOPIC)
    odom_input_topic = rospy.get_param(
        f'{rospy.get_name()}/odom_input_topic', ODOM_INPUT_TOPIC)
    odom_output_topic = rospy.get_param(
        f'{rospy.get_name()}/odom_output_topic', ODOM_OUTPUT_TOPIC)
    angle_dist_output_topic = rospy.get_param(
        f'{rospy.get_name()}/angle_dist_output_topic', ANGLE_DIST_OUTPUT_TOPIC)
    base_frame_id = rospy.get_param(
        f'{rospy.get_name()}/base_frame_id', BASE_FRAME_ID)
    odom_frame_id = rospy.get_param(
        f'{rospy.get_name()}/odom_frame_id', ODOM_FRAME_ID)
    frequency = rospy.get_param(
        f'{rospy.get_name()}/frequency', DEFAULT_RATE_HZ)
    delay = 1/frequency

    odom_output_publisher = rospy.Publisher(
        odom_output_topic, Odometry, queue_size=1000)
    angle_dist_publisher = rospy.Publisher(
        angle_dist_output_topic, String, queue_size=1000)

    rospy.Subscriber(imu_input_topic, Imu,
                     callback=update_imu_state)
    rospy.Subscriber(odom_input_topic, Odometry,
                     callback=update_odom_state)
    rospy.Timer(rospy.Duration(delay),
                callback=lambda _: compute_odometry_from_current_state(odom_output_publisher,
                                                                       angle_dist_publisher,
                                                                       base_frame_id,
                                                                       odom_frame_id))

    rospy.spin()


if __name__ == '__main__':
    try:
        start_node()
    except rospy.ROSInterruptException:
        pass
