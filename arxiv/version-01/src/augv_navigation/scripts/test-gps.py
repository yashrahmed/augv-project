from math import atan2, cos, pi, pow, radians, sin, sqrt, degrees

def calc_geo_disp(pointA, pointB):
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
    return degrees(atan2(x_wrt_n, y_wrt_n)), dist


pa = (41.78379440,-88.19348907)
pb = (41.783922, -88.193433)
pb = (41.783730, -88.193044)
pb = (41.783584, -88.193621)
pb = (41.783789, -88.194203)
pb = (41.784362, -88.193866)
print(calc_geo_disp(pa, pb))

"""
rostopic pub --once /way_points_upload std_msgs/String '{data: "[[41.783922, -88.193433]]"}'

"""