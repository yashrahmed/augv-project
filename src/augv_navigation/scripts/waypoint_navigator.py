import json
from math import atan2, cos, pi, pow, radians, sin, sqrt

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String

DEFAULT_ODOM_FRAME = 'my_odom'
DEFAULT_UTM_FRAME = 'my_utm'
DEFAULT_GPS_INPUT_TOPIC = '/my/sensor/gps'
DEFAULT_ODOM_INPUT_TOPIC = '/my/odometry/filtered_map'
DEFAULT_DRIVE_STATUS_INPUT_TOPIC = '/my/drive_status'
DEFAULT_WAYPOINTS_UPLOAD_INPUT_TOPIC = '/my/way_points_upload'
DEFAULT_MOVE_CMD_OUTPUT_TOPIC = '/my/set_point'
DEFAULT_GPS_MODE_ENABLED = 'false'
DEFAULT_MAG_DECL_RADIANS = 0.0

TARGET_WP_IDX_KEY = 'waypoint_idx'

waypoints = []
current_state = {
    'gps': {
        'lat': -1,
        'lon': -1
    },
    'goal_set': False,
    TARGET_WP_IDX_KEY: 0,
    'x': 0.0,
    'y': 0.0
}


def calc_angle_dist():
    x_tgt, y_tgt = waypoints[current_state[TARGET_WP_IDX_KEY]]
    x_curr = current_state['x']
    y_curr = current_state['y']
    dist = sqrt(pow(x_curr - x_tgt, 2) + pow(y_curr - y_tgt, 2))
    angle = atan2(y_tgt - y_curr, x_tgt - x_curr)
    return angle, dist


def calc_angle_dist_gps(decl_angle):
    x_tgt, y_tgt = waypoints[current_state[TARGET_WP_IDX_KEY]]
    x_curr = current_state['gps']['lat']
    y_curr = current_state['gps']['lon']
    if x_curr == -1:
        raise Exception("GPS not available")
    bearing, dist = calc_geo_disp((x_curr, y_curr), (x_tgt, y_tgt), decl_angle)
    return bearing, min(dist, 1)  # @TODO - remove min later


def calc_geo_disp(pointA, pointB, decl_angle):
    """
    Calculates the bearing between two points.
    The formulae used is the following:
        θ = atan2(sin(Δlong).cos(lat2),
                  cos(lat1).sin(lat2) − sin(lat1).cos(lat2).cos(Δlong))
    :Parameters:
      - `pointA: The tuple representing the latitude/longitude for the
        first point. Latitude and longitude must be in decimal degrees
      - `pointB: The tuple representing the latitude/longitude for the
        second point. Latitude and longitude must be in decimal degrees
    :Returns:
      The bearing in degrees
    :Returns Type:
      float
    """
    if (type(pointA) != tuple) or (type(pointB) != tuple):
        raise TypeError("Only tuples are supported as arguments")

    lat1, lon1 = pointA
    lat2, lon2 = pointB
    # convert decimal degrees to radians
    lon1, lat1, lon2, lat2 = map(radians, [lon1, lat1, lon2, lat2])

    # haversine formula
    dlon = lon2 - lon1
    dlat = lat2 - lat1
    a = sin(dlat/2)**2 + cos(lat1) * cos(lat2) * sin(dlon/2)**2
    c = 2 * atan2(sqrt(a), sqrt(1-a))
    r = 6371 * 1000  # Radius of earth in kilometers. Use 3956 for miles
    dist = c * r

    x = sin(dlon) * cos(lat2)
    y = cos(lat1) * sin(lat2) - (sin(lat1) * cos(lat2) * cos(dlon))

    bearing = atan2(x, y)
    bearing -= decl_angle
    """
    Now we have the initial bearing butatan2 return values
    from -180° to + 180° which is not what we want for a compass bearing
    The solution is to normalize the initial bearing as shown below
    """
    bearing = (bearing + 2*pi) % (2*pi)

    """
    This bearing is w.r.t True north but our IMU measures angle in ENU coordinates.
    Also the bearing increases in the clockwise direction
    Conversion below
    """
    x_wrt_n = dist * cos(bearing)
    y_wrt_n = dist * sin(bearing)
    # Note that x ,y vals are flipped before being given as an input to atan
    return atan2(x_wrt_n, y_wrt_n), dist


def send_movement_cmd(cmd_publisher, gps_mode_enabled, decl_angle):
    try:
        if current_state[TARGET_WP_IDX_KEY] < len(waypoints) and not current_state['goal_set']:
            angle, dist = calc_angle_dist_gps(
                decl_angle) if gps_mode_enabled else calc_angle_dist()
            current_state['goal_set'] = True
            cmd_publisher.publish(f'{angle},{dist}')
    except Exception as e:
        rospy.logerr(f'Failed to send movement command ex={e}')


def update_odom_state(odom_message):
    position = odom_message.pose.pose.position
    current_state['x'] = position.x
    current_state['y'] = position.y


def update_goal_state(msg):
    # any non Null message is considered success
    if msg.data:
        current_state[TARGET_WP_IDX_KEY] += 1
        current_state['goal_set'] = False


def update_gps_position(msg):
    current_state['gps']['lat'] = msg.latitude
    current_state['gps']['lon'] = msg.longitude


def update_waypoints(msg, args):
    gps_mode_enabled = args
    try:
        wp_path = json.loads(msg.data)
        waypoints.clear()
        if gps_mode_enabled:
            # @ToDo enable GPS transformation here of waypoints here......
            pass

        waypoints.extend(wp_path)
        current_state[TARGET_WP_IDX_KEY] = 0
        current_state['goal_set'] = False
        rospy.loginfo(f"Waypoint mission => {wp_path}")
    except Exception as e:
        rospy.logerr(
            f"waypoint mission upload failed message={msg} exception={e}")


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
    gps_mode_enabled = str(rospy.get_param(
        f'{rospy.get_name()}/gps_mode_enabled', DEFAULT_GPS_MODE_ENABLED)) == 'True'
    gps_input_topic = rospy.get_param(
        f'{rospy.get_name()}/gps_input_topic', DEFAULT_GPS_INPUT_TOPIC)
    magnetic_declination_radians = rospy.get_param(
        f'{rospy.get_name()}/magnetic_declination_radians', DEFAULT_MAG_DECL_RADIANS)

    rospy.loginfo(f"GPS Mode is set to {gps_mode_enabled}")

    cmd_publisher = rospy.Publisher(
        move_cmd_output_topic, String, queue_size=10)
    rospy.Subscriber(odom_input_topic, Odometry, callback=update_odom_state)
    rospy.Subscriber(waypoints_upload_topic, String,
                     callback=update_waypoints, callback_args=(gps_mode_enabled))
    rospy.Subscriber(drive_status_input_topic, String,
                     callback=update_goal_state)
    rospy.Subscriber(gps_input_topic, NavSatFix, callback=update_gps_position)
    rospy.Timer(rospy.Duration(
        1), callback=lambda _: send_movement_cmd(cmd_publisher, gps_mode_enabled, magnetic_declination_radians))

    rospy.spin()


if __name__ == '__main__':
    start_node()
