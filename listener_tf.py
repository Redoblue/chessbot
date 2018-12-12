#!/usr/bin/env python

import roslib
import rospy
import math
import tf
import geometry_msgs.msg


def main():
    rospy.init_node("listener_tf")

    listener = tf.TransformListener()

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            trans, rot = listener.lookupTransform("/world", "/kinect2_rgb_optical_frame", rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        ps_src = geometry_msgs.msg.PointStamped()
        ps_src.header.frame_id = "/kinect2_rgb_optical_frame"
        ps_src.point.x = -0.048
        ps_src.point.y = -0.052
        ps_src.point.z = 0.992

        ps_tar = listener.transformPoint("/world", ps_src)

        import ipdb
        ipdb.set_trace()

        rate.sleep()


if __name__ == "__main__":
    main()
