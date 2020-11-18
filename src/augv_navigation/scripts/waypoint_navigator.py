from std_msgs.msg import String
from nav_msgs.msg import Odometry
from math import atan2, pow, sqrt
import rospy
import json

DEFAULT_ODOM_INPUT_TOPIC = '/my/odometry/filtered_map'
DEFAULT_DRIVE_STATUS_INPUT_TOPIC = '/my/drive_status'
DEFAULT_WAYPOINTS_UPLOAD_INPUT_TOPIC = '/my/way_points_upload'
DEFAULT_MOVE_CMD_OUTPUT_TOPIC = '/my/set_point'

TARGET_WP_IDX_KEY = 'waypoint_idx'

waypoints = []
current_state = {
    'goal_set': False,
    TARGET_WP_IDX_KEY: 0,
    'x': 0.0,
    'y': 0.0
}


def send_movement_cmd(cmd_publisher):
    try:
        if current_state[TARGET_WP_IDX_KEY] < len(waypoints) and not current_state['goal_set']:
            x_tgt, y_tgt = waypoints[current_state[TARGET_WP_IDX_KEY]]
            x_curr = current_state['x']
            y_curr = current_state['y']
            dist = sqrt(pow(x_curr - x_tgt, 2) + pow(y_curr - y_tgt, 2))
            angle = atan2(y_tgt - y_curr, x_tgt - x_curr)
            current_state['goal_set'] = True
            cmd_publisher.publish(f'{angle},{dist}')
    except Exception as e:
        rospy.logerr(f'Failed to send movement command ex={e}')


def update_goal_state(msg):
    # any non Null message is considered success
    if msg.data:
        current_state[TARGET_WP_IDX_KEY] += 1
        current_state['goal_set'] = False


def update_waypoints(msg):
    try:
        wp_path = json.loads(msg.data)
        waypoints.clear()
        waypoints.extend(wp_path)
        current_state[TARGET_WP_IDX_KEY] = 0
        current_state['goal_set'] = False
        rospy.loginfo(f"Waypoint mission => {wp_path}")
    except Exception as e:
        rospy.logerr(f"waypoint mission upload failed message={msg} exception={e}")


def update_odom_state(odom_message):
    position = odom_message.pose.pose.position
    current_state['x'] = position.x
    current_state['y'] = position.y


def start_node():
    rospy.init_node('waypoint_nav_node')

    odom_input_topic = rospy.get_param(
        f'{rospy.get_name()}/odom_input_topic', DEFAULT_ODOM_INPUT_TOPIC)
    drive_status_input_topic = rospy.get_param(
        f'{rospy.get_name()}/drive_status_input_topic', DEFAULT_DRIVE_STATUS_INPUT_TOPIC)
    move_cmd_output_topic = rospy.get_param(
        f'{rospy.get_name()}/move_cmd_output_topic', DEFAULT_MOVE_CMD_OUTPUT_TOPIC)
    waypoints_upload_topic = rospy.get_param(
        f'{rospy.get_name()}/waypoints_upload_topic', DEFAULT_WAYPOINTS_UPLOAD_INPUT_TOPIC)

    cmd_publisher = rospy.Publisher(
        move_cmd_output_topic, String, queue_size=10)
    rospy.Subscriber(odom_input_topic, Odometry, callback=update_odom_state)
    rospy.Subscriber(waypoints_upload_topic, String, callback=update_waypoints)
    rospy.Subscriber(drive_status_input_topic, String,
                     callback=update_goal_state)
    rospy.Timer(rospy.Duration(
        1), callback=lambda _: send_movement_cmd(cmd_publisher))

    rospy.spin()


if __name__ == '__main__':
    start_node()
