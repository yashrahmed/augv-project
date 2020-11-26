import sys
import time

import rospy
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import String

INPUT_TOPIC = '/input'
OUTPUT_TOPIC = '/data'
FRAME_ID = 'map'
# Characteristics of the UBLOX NEO-6m
# Datasheet -> 2.5m for GPS (No Augmentation); Rounding to 3m
GPS_XY_ERROR_IN_MS2 = 9


def extract_gps_data(message):
    try:
        # First part of the message is "GPS "
        # // message format is [GPS lat lon alt hdof course]
        vals = message.split(',')
        if len(vals) == 5:
            return [float(v) for v in vals]
        else:
            return None
    except (UnicodeDecodeError, IndexError, ValueError):
        return None


def create_gps_message(gps_data, frame_id) -> NavSatFix:
    # gps data columns - lat lon alt hdop course
    lat, lon, alt, hdop, _ = gps_data
    gps_msg = NavSatFix()
    gps_msg.header.frame_id = frame_id
    gps_msg.header.stamp = rospy.get_rostime()

    gps_status = NavSatStatus()
    # Characteristics of the UBLOX NEO-6m
    gps_status.service = NavSatStatus.SERVICE_GPS
    gps_status.status = NavSatStatus.STATUS_FIX
    gps_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED

    gps_msg.status = gps_status
    gps_msg.latitude = lat
    gps_msg.longitude = lon
    gps_msg.altitude = alt

    gps_msg.position_covariance = [
        GPS_XY_ERROR_IN_MS2 * hdop * hdop, 0.0, 0.0,
        0.0,  GPS_XY_ERROR_IN_MS2 * hdop * hdop, 0.0,
        0.0, 0.0, 100.0]

    return gps_msg


def publish_gps_data(message, args):
    publisher, frame_id = args
    gps_data = extract_gps_data(message.data)
    if gps_data:
        publisher.publish(create_gps_message(gps_data, frame_id))


def start_node():
    rospy.init_node('gps_sensor_node', anonymous=True)
    input_topic = rospy.get_param(
        f'{rospy.get_name()}/input_topic', INPUT_TOPIC)
    output_topic = rospy.get_param(
        f'{rospy.get_name()}/output_topic', OUTPUT_TOPIC)
    frame_id = rospy.get_param(
        f'{rospy.get_name()}/frame_id', FRAME_ID)
    publisher = rospy.Publisher(output_topic, NavSatFix, queue_size=1000)
    rospy.Subscriber(input_topic, String,
                     callback=publish_gps_data, callback_args=(publisher, frame_id))
    rospy.spin()


if __name__ == '__main__':
    try:
        start_node()
    except rospy.ROSInterruptException:
        pass
