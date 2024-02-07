#!/usr/bin/env python3

import rospy
import math
from pyproj import CRS, Transformer, Proj
from tf.transformations import quaternion_from_euler
from tf2_ros import TransformBroadcaster
from novatel_oem7_msgs.msg import INSPVA
from geometry_msgs.msg import PoseStamped, TwistStamped, Quaternion, TransformStamped

class Localizer:
    def __init__(self):

        # Parameters
        self.undulation = rospy.get_param('/undulation')
        utm_origin_lat = rospy.get_param('/utm_origin_lat')
        utm_origin_lon = rospy.get_param('/utm_origin_lon')

        # Internal variables
        self.crs_wgs84 = CRS.from_epsg(4326)
        self.crs_utm = CRS.from_epsg(25835)
        self.utm_projection = Proj(self.crs_utm)
        
        # create coordinate transformer
        self.transformer = Transformer.from_crs(self.crs_wgs84, self.crs_utm)
        self.origin_x, self.origin_y = self.transformer.transform(utm_origin_lat, utm_origin_lon)

        # Subscribers
        rospy.Subscriber('/novatel/oem7/inspva', INSPVA, self.transform_coordinates)

        # Publishers
        self.current_pose_pub = rospy.Publisher('current_pose', PoseStamped, queue_size=10)
        self.current_velocity_pub = rospy.Publisher('current_velocity', TwistStamped, queue_size=10)
        self.br = TransformBroadcaster()

    def transform_coordinates(self, msg):
        # transform coordinates
        utm_x, utm_y = self.transformer.transform(msg.latitude, msg.longitude)
        elevation = msg.height - self.undulation

        azimuth_wgs84 = msg.azimuth
        azimuth_correction = (self.utm_projection.get_factors(msg.longitude, msg.latitude).meridian_convergence)
        azimuth_utm = azimuth_wgs84 - azimuth_correction

        yaw = convert_azimuth_to_yaw(math.radians(azimuth_utm))

        x, y, z, w = quaternion_from_euler(0, 0, yaw)
        orientation = Quaternion(x, y, z, w)

        velocity = math.sqrt(msg.north_velocity**2 + msg.east_velocity**2)

        # publish current pose
        current_pose_msg = PoseStamped()
        current_pose_msg.header.stamp = msg.header.stamp
        current_pose_msg.header.frame_id = 'map'
        current_pose_msg.pose.position.x = utm_x - self.origin_x
        current_pose_msg.pose.position.y = utm_y - self.origin_y
        current_pose_msg.pose.position.z = elevation
        current_pose_msg.pose.orientation = orientation
        self.current_pose_pub.publish(current_pose_msg)

        # publish current velocity
        current_velocity_msg = TwistStamped()
        current_velocity_msg.header.stamp = msg.header.stamp
        current_velocity_msg.header.frame_id = 'base_link'
        current_velocity_msg.twist.linear.x = velocity
        self.current_velocity_pub.publish(current_velocity_msg)

        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = 'map'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = utm_x - self.origin_x
        t.transform.translation.y = utm_y - self.origin_y
        t.transform.translation.z = elevation
        t.transform.rotation = orientation
        self.br.sendTransform(t)

    def run(self):
        rospy.spin()

def convert_azimuth_to_yaw(azimuth):
    """
    Converts azimuth to yaw. Azimuth is CW angle from North. Yaw is CCW angle from East.
    :param azimuth: azimuth in radians
    :return: yaw in radians
    """
    yaw = -azimuth + math.pi/2
    # Clamp within 0 to 2 pi
    if yaw > 2 * math.pi:
        yaw = yaw - 2 * math.pi
    elif yaw < 0:
        yaw += 2 * math.pi

    return yaw

if __name__ == '__main__':
    rospy.init_node('localizer')
    node = Localizer()
    node.run()