#!/usr/bin/env python

import argparse

import roslib; roslib.load_manifest('hrl_ft')
import rospy

from geometry_msgs.msg import WrenchStamped, PoseStamped, Point, PointStamped
from visualization_msgs.msg import  Marker

class WrenchToMarker(object):
    def __init__(self, scale, frame):
        self.scale = scale
        self.frame = frame
        self.ws_sub = rospy.Subscriber('wrench_in', WrenchStamped,
                self.wrench_cb)
        self.marker_pub = rospy.Publisher('wrench_marker', Marker)

    def wrench_cb(self, ws):
        marker = self.form_marker(ws)
        self.marker_pub.publish(marker)

    def form_marker(self, ws):
        force_vec = Marker()
        force_vec.header.stamp = rospy.Time.now()
        force_vec.header.frame_id = self.frame
        force_vec.ns = "ft_sensor"
        force_vec.action = 0
        force_vec.type = 0
        force_vec.scale.x = 0.1
        force_vec.scale.y = 0.2
        force_vec.scale.z = 0.1
        force_vec.color.a = 0.75
        force_vec.color.r = 0.0
        force_vec.color.g = 1.0
        force_vec.color.b = 0.1
        force_vec.lifetime = rospy.Duration(1)

        origin = Point()
        force_point = Point()
        force_point.x = self.scale*ws.wrench.force.x
        force_point.y = self.scale*ws.wrench.force.y
        force_point.z = self.scale*ws.wrench.force.z
        force_vec.points.append(origin)
        force_vec.points.append(force_point)
        return force_vec

if __name__=='__main__':
    parser = argparse.ArgumentParser(description="Apply a transform")
    parser.add_argument('-s','--scale', default=0.1, type=float,
                        help='The scaling applied to the vector magnitude.'+
                             'Length in meters per Newton of force. (m/N)')
    parser.add_argument('-f','--frame', default='/l_force_torque_link',
                        help='The TF frame of the incoming wrench.')
    args = parser.parse_known_args()

    rospy.init_node('wrench_viz_marker')
    wrench_to_marker = WrenchToMarker(scale=args[0].scale, frame=args[0].frame)
    rospy.spin()
