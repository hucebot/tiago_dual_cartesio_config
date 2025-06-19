#!/usr/bin/env python3

from cartesian_interface.pyci_all import *
import rospy
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation as R
import numpy as np

ci = pyci.CartesianInterfaceRos()
pose_initial = Affine3()
initial_ref = PoseStamped()
initialized = False
controlled_frame = str()

def callback(data: PoseStamped):
    global initial_ref, initialized, controlled_frame, pose_initial, ci, pose_initial

    if not initialized:
        initial_ref = data
        tmp, _, _ = ci.getPoseReference(controlled_frame)
        pose_initial.translation = tmp.translation
        initialized = True
        rospy.loginfo("correclty initialized")
    else:
        r_init = R.from_quat(
            [
                initial_ref.pose.orientation.x,
                initial_ref.pose.orientation.y,
                initial_ref.pose.orientation.z,
                initial_ref.pose.orientation.w,
            ]
        )
        data_m_init = Affine3()
        data_m_init.translation = np.array(
            [
                initial_ref.pose.position.x,
                initial_ref.pose.position.y,
                initial_ref.pose.position.z,
            ]
        )
        data_m_init.linear = r_init.as_matrix()

        r = R.from_quat(
            [
                data.pose.orientation.x,
                data.pose.orientation.y,
                data.pose.orientation.z,
                data.pose.orientation.w,
            ]
        )
        data_m = Affine3()
        data_m.translation = np.array(
            [data.pose.position.x, data.pose.position.y, data.pose.position.z - 0.08]
        )
        data_m.linear = r.as_matrix()

        ref_m = pose_initial * data_m_init.inverse() * data_m

        pose_ref = Affine3()
        pose_ref.translation = data_m.translation
        pose_ref.linear = data_m.linear

        ci.setPoseReference(controlled_frame, pose_ref)


if __name__ == "__main__":
    rospy.init_node("teleop_bridge", anonymous=False)

    if rospy.has_param("~controlled_frame"):
        controlled_frame = rospy.get_param("~controlled_frame")
    else:
        rospy.logerr("controlled_frame private param is mandatory! Exiting.")
        exit()

    rospy.loginfo(f"controlled_frame: {controlled_frame}")

    rospy.Subscriber("teleop_pose", PoseStamped, callback)

    rospy.spin()
