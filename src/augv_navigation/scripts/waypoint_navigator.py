from std_msgs.msg import String
from nav_msgs.msg import Odometry
from math import atan2, pow, sqrt
import rospy

DEFAULT_ODOM_INPUT_TOPIC = '/odometry/filtered_map'
DEFAULT_DRIVE_STATUS_INPUT_TOPIC = '/drive_status'
DEFAULT_MOVE_CMD_OUTPUT_TOPIC = '/set_point'
TARGET_WP_IDX_KEY = 'waypoint_idx'

waypoints = [(0.5, 0), (0.5, 0.5), (0, 0.5), (0, 0)]
current_state = {
    'goal_set': False,
    TARGET_WP_IDX_KEY: 0,
    'x': 0.0,
    'y': 0.0
}


def send_movement_cmd(cmd_publisher):
    if current_state[TARGET_WP_IDX_KEY] < len(waypoints) and not current_state['goal_set']:
        x_tgt, y_tgt = waypoints[current_state[TARGET_WP_IDX_KEY]]
        x_curr = current_state['x']
        y_curr = current_state['y']
        dist = sqrt(pow(x_curr - x_tgt, 2) + pow(y_curr - y_tgt, 2))
        angle = atan2(y_tgt - y_curr, x_tgt - x_curr)
        current_state['goal_set'] = True
        cmd_publisher.publish(f'{angle},{dist}')


def update_goal_state(msg):
    # any non Null message is considered success
    if msg.data:
        current_state[TARGET_WP_IDX_KEY] += 1
        current_state['goal_set'] = False


def update_odom_state(odom_message):
    position = odom_message.pose.pose.position
    current_state['x'] = position.x
    current_state['y'] = position.y


def start_node():
    rospy.init_node('waypoint_nav_node')

    odom_input_topic = DEFAULT_ODOM_INPUT_TOPIC
    drive_status_input_topic = DEFAULT_DRIVE_STATUS_INPUT_TOPIC
    move_cmd_output_topic = DEFAULT_MOVE_CMD_OUTPUT_TOPIC

    cmd_publisher = rospy.Publisher(
        move_cmd_output_topic, String, queue_size=10)
    rospy.Subscriber(odom_input_topic, Odometry, callback=update_odom_state)
    rospy.Subscriber(drive_status_input_topic, String,
                     callback=update_goal_state)
    rospy.Timer(rospy.Duration(1), callback=lambda _ : send_movement_cmd(cmd_publisher))

    rospy.spin()


if __name__ == '__main__':
    start_node()
