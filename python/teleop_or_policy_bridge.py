#!/usr/bin/env python3

from cartesian_interface.pyci_all import *
import rospy
from geometry_msgs.msg import PoseStamped, PointStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import numpy as np
import tf2_ros
from threading import Lock
import time

# CartesI/O ROS client interface
ci = pyci.CartesianInterfaceRos()

# Global variables
controlled_frame = str()
GRIPPER_MAX = 0.044
GRIPPER_MIN = 0.010
TELEOP_MAX = 0.0
TELEOP_MIN = 1.0
hz = 100.0
gripper_time_duration = 0.2
pose_initial = Affine3()
initial_ref = PoseStamped()
initialized = False
gripper_cmd_pub = None
gripper_cmd_msg = JointTrajectory()
goal_pose = None
gripper_cmd = None

# Mutex
mutex_pose = Lock()
mutex_gripper = Lock()


def teleop_gripper_cb(data: PointStamped):
    """Store the gripper command from teleoperation"""
    global gripper_cmd
    a = (GRIPPER_MAX - GRIPPER_MIN) / (TELEOP_MAX - TELEOP_MIN)
    d = data.point.x
    with mutex_gripper:
        gripper_cmd = a * d * (d - TELEOP_MAX) + GRIPPER_MAX


def teleop_pose_cb(data: PoseStamped):
    """Store the goal pose from teleoperation"""
    global goal_pose
    with mutex_pose:
        goal_pose = data


def init_gripper_msg(cmd_msg: JointTrajectory, joint_names):
    """Init the gripper command message"""
    cmd_msg.joint_names = joint_names
    cmd_msg.points.append(JointTrajectoryPoint())
    cmd_msg.points[0].positions = [0.0] * len(cmd_msg.joint_names)
    cmd_msg.points[0].velocities = [0.0] * len(cmd_msg.joint_names)
    return cmd_msg


def set_gripper_cmd(cmd: float):
    """Send the gripper command to the robot"""
    for joint_name in gripper_cmd_msg.joint_names:
        gripper_cmd_msg.points[0].positions[
            gripper_cmd_msg.joint_names.index(joint_name)
        ] = cmd
    gripper_cmd_msg.points[0].time_from_start = rospy.Duration(gripper_time_duration)
    gripper_cmd_pub.publish(gripper_cmd_msg)


def set_goal_pose(data: PoseStamped):
    """Send the goal pose in CartesI/O"""
    global initial_ref, initialized, pose_initial

    if not initialized:
        initial_ref = data
        tmp, _, _ = ci.getPoseReference(controlled_frame)
        pose_initial.translation = tmp.translation
        initialized = True
        rospy.loginfo("Teleop rotation offset correctly saved")
    else:
        data_m_init = Affine3()
        data_m_init.translation = np.array(
            [
                initial_ref.pose.position.x,
                initial_ref.pose.position.y,
                initial_ref.pose.position.z,
            ]
        )
        data_m_init.quaternion = np.array(
            [
                initial_ref.pose.orientation.x,
                initial_ref.pose.orientation.y,
                initial_ref.pose.orientation.z,
                initial_ref.pose.orientation.w,
            ]
        )

        data_m = Affine3()
        data_m.translation = np.array(
            [data.pose.position.x, data.pose.position.y, data.pose.position.z]
        )
        data_m.quaternion = np.array(
            [
                data.pose.orientation.x,
                data.pose.orientation.y,
                data.pose.orientation.z,
                data.pose.orientation.w,
            ]
        )

        ref_m = pose_initial * data_m_init.inverse() * data_m

        pose_ref = Affine3()
        pose_ref.translation = ref_m.translation
        pose_ref.linear = data_m.linear

        ci.setPoseReference(controlled_frame, pose_ref)


if __name__ == "__main__":
    rospy.init_node("teleop_or_policy_bridge", anonymous=False)
    # Set up tf2 listener ------------------------------------------------------------------------
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    time.sleep(1)

    # Get parameters -----------------------------------------------------------------------------
    if rospy.has_param("~controlled_frame"):
        controlled_frame = rospy.get_param("~controlled_frame")
    else:
        rospy.logerr("controlled_frame private param is mandatory! Exiting.")
        exit()
    rospy.loginfo(f"controlled_frame: {controlled_frame}")

    if rospy.has_param("~controlled_gripper"):
        controlled_gripper = rospy.get_param("~controlled_gripper")
    else:
        rospy.logerr("controlled_gripper private param is mandatory! Exiting.")
        exit()
    rospy.loginfo(f"controlled_gripper: {controlled_gripper}")

    controlled_gripper_joints = []
    if rospy.has_param("~controlled_gripper_joints"):
        controlled_gripper_joints = rospy.get_param("~controlled_gripper_joints")
    else:
        rospy.logerr("controlled_gripper_joints private param is mandatory! Exiting.")
        exit()
    rospy.loginfo(f"controlled_gripper_joints: {controlled_gripper_joints}")

    if rospy.has_param("~GRIPPER_MAX"):
        GRIPPER_MAX = rospy.get_param("~GRIPPER_MAX")
    if rospy.has_param("~GRIPPER_MIN"):
        GRIPPER_MIN = rospy.get_param("~GRIPPER_MIN")
    if rospy.has_param("~TELEOP_MAX"):
        TELEOP_MAX = rospy.get_param("~TELEOP_MAX")
    if rospy.has_param("~TELEOP_MIN"):
        TELEOP_MIN = rospy.get_param("~TELEOP_MIN")

    cameras = []
    if rospy.has_param("~cameras"):
        cameras = rospy.get_param("~cameras")
    rospy.loginfo(f"cameras: {cameras}")

    # Init gripper command publisher -------------------------------------------------------------
    gripper_cmd_pub = rospy.Publisher(
        controlled_gripper + "/command", JointTrajectory, queue_size=1
    )
    gripper_cmd_msg = init_gripper_msg(gripper_cmd_msg, controlled_gripper_joints)

    # Init proprioception publihsers -------------------------------------------------------------
    read_gripper_pose_pub = rospy.Publisher(
        controlled_frame + "/read", PoseStamped, queue_size=1
    )
    read_gripper_pose_msg = PoseStamped()
    read_gripper_pose_seq = 0

    goal_gripper_pose_pub = rospy.Publisher(
        controlled_frame + "/goal", PoseStamped, queue_size=1
    )
    goal_gripper_pose_msg = PoseStamped()
    goal_gripper_pose_seq = 0

    camera_frame_pub = {}
    camera_pose_msg = {}
    camera_frame_seq = {}
    # for camera in cameras:
    #     camera_frame_pub[camera] = rospy.Publisher(
    #         f"{camera}_color_optical_frame/pose", PoseStamped, queue_size=1
    #     )
    #     camera_pose_msg[camera] = PoseStamped()
    #     camera_frame_seq[camera] = 0

    # Subscribe to teleop topics -----------------------------------------------------------------
    rospy.Subscriber("/dxl_input/gripper_right", PointStamped, teleop_gripper_cb)
    rospy.Subscriber("/dxl_input/pos_right", PoseStamped, teleop_pose_cb)

    rospy.loginfo("Teleop/policy bridge initialized")

    # Start loop ---------------------------------------------------------------------------------
    rate = rospy.Rate(hz)
    while not rospy.is_shutdown():
        # Publish prooprioception topics ---------------------------------------------------------
        try:
            t = tf_buffer.lookup_transform(
                "base_link",
                controlled_frame,
                rospy.Time(),
            )
            read_gripper_pose_msg.pose.position.x = t.transform.translation.x
            read_gripper_pose_msg.pose.position.y = t.transform.translation.y
            read_gripper_pose_msg.pose.position.z = t.transform.translation.z
            read_gripper_pose_msg.pose.orientation.x = t.transform.rotation.x
            read_gripper_pose_msg.pose.orientation.y = t.transform.rotation.y
            read_gripper_pose_msg.pose.orientation.z = t.transform.rotation.z
            read_gripper_pose_msg.pose.orientation.w = t.transform.rotation.w
            read_gripper_pose_msg.header.seq = read_gripper_pose_seq
            read_gripper_pose_msg.header.stamp = t.header.stamp
            read_gripper_pose_msg.header.frame_id = "base_link"
            read_gripper_pose_pub.publish(read_gripper_pose_msg)
            read_gripper_pose_seq += 1
        except Exception as err:
            print(err)
        try:
            t = tf_buffer.lookup_transform(
                "ci/base_link",
                f"ci/{controlled_frame}",
                rospy.Time(),
            )
            goal_gripper_pose_msg.pose.position.x = t.transform.translation.x
            goal_gripper_pose_msg.pose.position.y = t.transform.translation.y
            goal_gripper_pose_msg.pose.position.z = t.transform.translation.z
            goal_gripper_pose_msg.pose.orientation.x = t.transform.rotation.x
            goal_gripper_pose_msg.pose.orientation.y = t.transform.rotation.y
            goal_gripper_pose_msg.pose.orientation.z = t.transform.rotation.z
            goal_gripper_pose_msg.pose.orientation.w = t.transform.rotation.w
            goal_gripper_pose_msg.header.seq = read_gripper_pose_seq
            read_gripper_pose_msg.header.stamp = t.header.stamp
            goal_gripper_pose_msg.header.frame_id = f"ci/{controlled_frame}"
            goal_gripper_pose_pub.publish(goal_gripper_pose_msg)
            goal_gripper_pose_seq += 1
        except Exception as err:
            print(err)
        # for camera in cameras:
        #     try:
        #         t = tf_buffer.lookup_transform(
        #             "base_link",
        #             f"{camera}_color_optical_frame",
        #             rospy.Time(),
        #         )
        #         camera_pose_msg[camera].pose.position.x = t.transform.translation.x
        #         camera_pose_msg[camera].pose.position.y = t.transform.translation.y
        #         camera_pose_msg[camera].pose.position.z = t.transform.translation.z
        #         camera_pose_msg[camera].pose.orientation.x = t.transform.rotation.x
        #         camera_pose_msg[camera].pose.orientation.y = t.transform.rotation.y
        #         camera_pose_msg[camera].pose.orientation.z = t.transform.rotation.z
        #         camera_pose_msg[camera].pose.orientation.w = t.transform.rotation.w
        #         camera_pose_msg[camera].header.seq = camera_frame_seq[camera]
        #         read_gripper_pose_msg.header.stamp = t.header.stamp
        #         camera_pose_msg[camera].header.frame_id = "base_link"
        #         camera_frame_pub[camera].publish(camera_pose_msg[camera])
        #         camera_frame_seq[camera] += 1
        #     except Exception as err:
        #         print(err)

        # Send to the robot the last received commands -------------------------------------------
        with mutex_gripper:
            if gripper_cmd is not None:
                set_gripper_cmd(gripper_cmd)
                gripper_cmd = None

        with mutex_pose:
            if goal_pose is not None:
                set_goal_pose(goal_pose)
                goal_pose = None

        rate.sleep()

    rospy.spin()
