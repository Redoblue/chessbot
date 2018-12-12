import numpy as np

import tf
import rospy
import geometry_msgs


class CameraCoordTransformer:

    def __init__(self):
        self.tf_listener = tf.TransformListener()

    @staticmethod
    def get_coordinate(pcl, u, v):
        offset = pcl.row_step * v + pcl.point_step * u
        data = pcl.data[offset:offset+12]
        xyz = np.frombuffer(data, dtype=np.float32, count=3)

        # print(u, v, '->', xyz[0], xyz[1])
        # deal with nan
        judges = [not -np.inf < x < np.inf for x in xyz]
        if any(judges):
            return None

        return xyz.tolist()

    def transform(self, pclorxyz, x=None, y=None):
        try:
            trans, rot = self.tf_listener.lookupTransform(
                "/world", "/kinect2_rgb_optical_frame", rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print("something wrong happened.")

        # get point coord in camera frame
        if isinstance(pclorxyz, list) and x is None and y is None:
            xyz = pclorxyz
        else:
            xyz = self.get_coordinate(pclorxyz, x, y)
            if xyz is None:
                return None

        ps_src = geometry_msgs.msg.PointStamped()
        ps_src.header.frame_id = "/kinect2_rgb_optical_frame"
        ps_src.point.x = xyz[0]
        ps_src.point.y = xyz[1]
        ps_src.point.z = xyz[2]
        ps_tar = self.tf_listener.transformPoint("/world", ps_src)
        return ps_tar
