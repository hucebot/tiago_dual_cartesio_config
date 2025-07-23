#!/usr/bin/env python3

import rospy
import os
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Twist, WrenchStamped
from std_msgs.msg import Float32, Bool
from std_srvs.srv import SetBool, SetBoolResponse
from pal_control_msgs.msg import ActuatorCurrentLimit
from controller_manager_msgs.srv import SwitchController

head_cmd_pub_ = None
head_cmd_msg_ = JointTrajectory()

left_arm_cmd_pub_ = None
left_arm_cmd_msg_ = JointTrajectory()

right_arm_cmd_pub_ = None
right_arm_cmd_msg_ = JointTrajectory()

torso_cmd_pub_ = None
torso_cmd_msg_ = JointTrajectory()

mobile_base_cmd_pub_ = None
mobile_base_cmd_msg_ = Twist()

last_joint_state_ = None

time_from_start_ = 0.2

# max value for each axis: x = -42, y = -58, z = -69
wrist_force_limit = 71.0
# max value for each axis: x = 8, y = -11, z = 3.0
wrist_torque_limit = 12.0

exit_ = False

send_commands_ = True

SEND_VELOCITY = False


def set_initial_configuration(data: JointState):
    home = dict()
    home["reference@v0"] = 0.0
    home["reference@v1"] = 0.0
    home["reference@v2"] = 0.0#!/usr/bin/env python3

import rospy
import os
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Twist, WrenchStamped
from std_msgs.msg import Float32, Bool
from std_srvs.srv import SetBool, SetBoolResponse
from pal_control_msgs.msg import ActuatorCurrentLimit

head_cmd_pub_ = None
head_cmd_msg_ = JointTrajectory()

left_arm_cmd_pub_ = None
left_arm_cmd_msg_ = JointTrajectory()

right_arm_cmd_pub_ = None
right_arm_cmd_msg_ = JointTrajectory()

torso_cmd_pub_ = None
torso_cmd_msg_ = JointTrajectory()

mobile_base_cmd_pub_ = None
mobile_base_cmd_msg_ = Twist()

last_joint_state_ = None

time_from_start_ = 0.2

# max value for each axis: x = -42, y = -58, z = -69
wrist_force_limit = 71.0
# max value for each axis: x = 8, y = -11, z = 3.0
wrist_torque_limit = 12.0

exit_ = False

send_commands_ = True

SEND_VELOCITY = False


def set_initial_configuration(data: JointState):
    home = dict()
    home["reference@v0"] = 0.0
    home["reference@v1"] = 0.0
    home["reference@v2"] = 0.0
    home["reference@v3"] = 0.0
    home["reference@v4"] = 0.0
    home["reference@v5"] = 0.0

    for i in range(len(data.name)):
        home[data.name[i]] = data.position[i]

    rospy.loginfo("Initial configuration:")
    for key in home:
        rospy.loginfo(f"{key}: {home[key]}")

    rospy.set_param("cartesian/home", home)

def fill_cms_msg(data: JointState, cmd_msg: JointTrajectory, send_velocity=True):
    global time_from_start_
    for joint_name in cmd_msg.joint_names:
        cmd_msg.points[0].positions[cmd_msg.joint_names.index(joint_name)] = (
            data.position[data.name.index(joint_name)]
        )
        if send_velocity:
            cmd_msg.points[0].velocities[cmd_msg.joint_names.index(joint_name)] = (
                data.velocity[data.name.index(joint_name)]
            )
    cmd_msg.points[0].time_from_start = rospy.Duration(time_from_start_)
    return cmd_msg

def start_motion_controllers():
    rospy.wait_for_service("/controller_manager/switch_controller")
    try:
        switch_controller = rospy.ServiceProxy("/controller_manager/switch_controller", SwitchController)
        # List of controllers to start again
        start_list = [
            "arm_left_controller",
            "arm_right_controller",
            "head_controller",
            "torso_controller",
            "mobile_base_controller",
            "gripper_right_controller"
        ]
        req = SwitchController._request_class()
        req.stop_controllers = []
        req.start_controllers = start_list
        req.strictness = 2  # STRICT to ensure controllers start properly

        resp = switch_controller(req)
        if resp.ok:
            rospy.loginfo("Controllers successfully started after emergency.")
        else:
            rospy.logerr("Failed to start motion controllers.")
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed while starting controllers: {e}")

def publish_current_position_as_target():
    global head_cmd_msg_, left_arm_cmd_msg_, right_arm_cmd_msg_, torso_cmd_msg_
    global head_cmd_pub_, left_arm_cmd_pub_, right_arm_cmd_pub_, torso_cmd_pub_
    global mobile_base_cmd_pub_, mobile_base_cmd_msg_

    rospy.loginfo("Waiting for current joint state before publishing target...")
    try:
        joint_state = rospy.wait_for_message("joint_states", JointState, timeout=1.0)
    except rospy.ROSException:
        rospy.logwarn("Timeout while waiting for joint_states.")
        return

    rospy.loginfo("Publishing current joint positions as target.")

    # Fill command messages with the fresh joint state
    head_cmd_msg_ = fill_cms_msg(joint_state, head_cmd_msg_, send_velocity=False)
    left_arm_cmd_msg_ = fill_cms_msg(joint_state, left_arm_cmd_msg_, send_velocity=False)
    right_arm_cmd_msg_ = fill_cms_msg(joint_state, right_arm_cmd_msg_, send_velocity=False)
    torso_cmd_msg_ = fill_cms_msg(joint_state, torso_cmd_msg_, send_velocity=False)

    now = rospy.get_rostime()
    head_cmd_msg_.header.stamp = now
    left_arm_cmd_msg_.header.stamp = now
    right_arm_cmd_msg_.header.stamp = now
    torso_cmd_msg_.header.stamp = now

    # Publish the command messages
    head_cmd_pub_.publish(head_cmd_msg_)
    left_arm_cmd_pub_.publish(left_arm_cmd_msg_)
    right_arm_cmd_pub_.publish(right_arm_cmd_msg_)
    torso_cmd_pub_.publish(torso_cmd_msg_)

    # Optionally stop the mobile base
    mobile_base_cmd_msg_.linear.x = 0.0
    mobile_base_cmd_msg_.linear.y = 0.0
    mobile_base_cmd_msg_.linear.z = 0.0
    mobile_base_cmd_msg_.angular.x = 0.0
    mobile_base_cmd_msg_.angular.y = 0.0
    mobile_base_cmd_msg_.angular.z = 0.0
    mobile_base_cmd_pub_.publish(mobile_base_cmd_msg_)




def stop_motion_controllers():
    rospy.wait_for_service("/controller_manager/switch_controller")
    try:
        switch_controller = rospy.ServiceProxy("/controller_manager/switch_controller", SwitchController)
        # List of controllers to stop (excluding joint_state_controller and sensor controllers)
        stop_list = [
            "arm_left_controller",
            "arm_right_controller",
            "head_controller",
            "torso_controller",
            "mobile_base_controller",
            "gripper_right_controller"
        ]
        req = SwitchController._request_class()
        req.stop_controllers = stop_list
        req.start_controllers = []
        req.strictness = 2  # BEST_EFFORT = 1, STRICT = 2

        resp = switch_controller(req)
        if resp.ok:
            rospy.logwarn("Emergency: Motion controllers successfully stopped.")
        else:
            rospy.logerr("Emergency: Failed to stop motion controllers.")
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")


def left_ft_cb(data: WrenchStamped):
    global wrist_force_limit
    global wrist_torque_limit
    global send_commands_
    if (
        abs(data.wrench.force.x) > wrist_force_limit
        or abs(data.wrench.force.y) > wrist_force_limit
        or abs(data.wrench.force.z) > wrist_force_limit
    ):
        rospy.logwarn(f"Exceeded left wrist force limit. Motion disabled")
        send_commands_ = False

    if (
        abs(data.wrench.torque.x) > wrist_torque_limit
        or abs(data.wrench.torque.y) > wrist_torque_limit
        or abs(data.wrench.torque.z) > wrist_torque_limit
    ):
        rospy.logwarn(f"Exceeded left wrist torque limit. Motion disabled")
        send_commands_ = False


def right_ft_cb(data: WrenchStamped):
    global wrist_force_limit
    global wrist_torque_limit
    global send_commands_
    if (
        abs(data.wrench.force.x) > wrist_force_limit
        or abs(data.wrench.force.y) > wrist_force_limit
        or abs(data.wrench.force.z) > wrist_force_limit
    ):
        rospy.logwarn(f"Exceeded right wrist force limit. Motion disabled")
        send_commands_ = False

    if (
        abs(data.wrench.torque.x) > wrist_torque_limit
        or abs(data.wrench.torque.y) > wrist_torque_limit
        or abs(data.wrench.torque.z) > wrist_torque_limit
    # Fill all command messages with current position and zero velocity
    ):
        rospy.logwarn(f"Exceeded right wrist torque limit. Motion disabled")
        send_commands_ = False


def io_callback(data: JointState):
    global head_cmd_pub_, head_cmd_msg_
    global left_arm_cmd_pub_, left_arm_cmd_msg_
    global right_arm_cmd_pub_, right_arm_cmd_msg_
    global torso_cmd_pub_, torso_cmd_msg_
    global mobile_base_cmd_pub_, mobile_base_cmd_msg_
    global time_from_start_
    global send_commands_
    global last_joint_state_
    last_joint_state_ = data

    head_cmd_msg_ = fill_cms_msg(data, head_cmd_msg_, send_velocity=SEND_VELOCITY)
    left_arm_cmd_msg_ = fill_cms_msg(
        data, left_arm_cmd_msg_, send_velocity=SEND_VELOCITY
    )
    right_arm_cmd_msg_ = fill_cms_msg(
        data, right_arm_cmd_msg_, send_velocity=SEND_VELOCITY
    )
    torso_cmd_msg_ = fill_cms_msg(data, torso_cmd_msg_, send_velocity=SEND_VELOCITY)

    mobile_base_cmd_msg_.linear.x = data.velocity[0]
    mobile_base_cmd_msg_.linear.y = data.velocity[1]
    mobile_base_cmd_msg_.linear.z = 0.0
    mobile_base_cmd_msg_.angular.x = 0.0
    mobile_base_cmd_msg_.angular.y = 0.0
    mobile_base_cmd_msg_.angular.z = data.velocity[5]

    time = rospy.get_rostime()

    head_cmd_msg_.header.stamp = time
    left_arm_cmd_msg_.header.stamp = time
    right_arm_cmd_msg_.header.stamp = time
    torso_cmd_msg_.header.stamp = time

    if send_commands_:
        head_cmd_pub_.publish(head_cmd_msg_)
        left_arm_cmd_pub_.publish(left_arm_cmd_msg_)
        right_arm_cmd_pub_.publish(right_arm_cmd_msg_)
        torso_cmd_pub_.publish(torso_cmd_msg_)
        mobile_base_cmd_pub_.publish(mobile_base_cmd_msg_)


def init_cmd_msg(cmd_msg: JointTrajectory, joint_names):
    cmd_msg.joint_names = joint_names
    cmd_msg.points.append(JointTrajectoryPoint())
    cmd_msg.points[0].positions = [0.0] * len(cmd_msg.joint_names)
    cmd_msg.points[0].velocities = [0.0] * len(cmd_msg.joint_names)
    return cmd_msg


def set_send_commands_cb(req: SetBool):
    global send_commands_
    send_commands_ = req.data
    response = SetBoolResponse()
    response.success = True
    response.message = f"send_commands: {req.data}"
    rospy.loginfo(f"Setting 'send_commands: {req.data}'")
    return response


def set_time_from_start_cb(msg: Float32):
    global time_from_start_
    if msg.data != time_from_start_:
        if msg.data >= 0.1:
            time_from_start_ = msg.data
            rospy.loginfo(f"Setting 'time_from_start: {time_from_start_}'")
        else:
            rospy.logwarn(
                "Trying to set a too low 'time_from_start'. It must be >= 0.1secs"
            )


def em_cb(msg):
    global send_commands_
    if msg.data:  # Emergency triggered
        rospy.logwarn("Emergency button pressed. Stopping controllers.")
        send_commands_ = False
        stop_motion_controllers()

def restart_cb(msg):
    global send_commands_
    if msg.data:
        rospy.logwarn("Restarting controllers")
        send_commands_ = True
        start_motion_controllers()
        rospy.sleep(2.0)
        publish_current_position_as_target()


def control_initiator_cb(msg):
    global exit_
    if not msg.data:
        exit_ = True


if __name__ == "__main__":
    rospy.init_node("ros_control_bridge_node", anonymous=False)
    rospy.loginfo(f"SEND_VELOCITY: {SEND_VELOCITY}")

    # Wait to receive the first msg from /joint_states
    data = rospy.wait_for_message("joint_states", JointState, timeout=5)
    set_initial_configuration(data)

    # Lower maximum currents for the grippers
    # lg_curr_lim_pub = rospy.Publisher(
    #     "/gripper_left_current_limit_controller/command",
    #     ActuatorCurrentLimit,
    #     queue_size=1,
    # )
    # lg_curr_lim_msg = ActuatorCurrentLimit()
    # lg_curr_lim_msg.actuator_names = [
    #     "gripper_left_left_finger_motor",
    #     "gripper_left_right_finger_motor",
    # ]
    # lg_curr_lim_msg.current_limits = [0.75, 0.75]
    # while lg_curr_lim_pub.get_num_connections() < 1:
    #     # wait for a connection to publisher
    #     pass
    # lg_curr_lim_pub.publish(lg_curr_lim_msg)

    # rg_curr_lim_pub = rospy.Publisher(
    #     "/gripper_right_current_limit_controller/command",
    #     ActuatorCurrentLimit,
    #     queue_size=1,
    # )
    # rg_curr_lim_msg = ActuatorCurrentLimit()
    # rg_curr_lim_msg.actuator_names = [
    #     "gripper_right_left_finger_motor",
    #     "gripper_right_right_finger_motor",
    # ]
    # rg_curr_lim_msg.current_limits = [0.75, 0.75]
    # while rg_curr_lim_pub.get_num_connections() < 1:
    #     # wait for a connection to publisher
    #     pass
    # rg_curr_lim_pub.publish(rg_curr_lim_msg)

    # Set up command publishers and msgs
    head_cmd_pub_ = rospy.Publisher(
        "head_controller/command", JointTrajectory, queue_size=1
    )
    head_cmd_msg_ = init_cmd_msg(head_cmd_msg_, ["head_1_joint", "head_2_joint"])

    left_arm_cmd_pub_ = rospy.Publisher(
        "/arm_left_controller/command", JointTrajectory, queue_size=1
    )
    left_arm_cmd_msg_ = init_cmd_msg(
        left_arm_cmd_msg_,
        [
            "arm_left_1_joint",
            "arm_left_2_joint",
            "arm_left_3_joint",
            "arm_left_4_joint",
            "arm_left_5_joint",
            "arm_left_6_joint",
            "arm_left_7_joint",
        ],
    )

    right_arm_cmd_pub_ = rospy.Publisher(
        "/arm_right_controller/command", JointTrajectory, queue_size=1
    )
    right_arm_cmd_msg_ = init_cmd_msg(
        right_arm_cmd_msg_,
        [
            "arm_right_1_joint",
            "arm_right_2_joint",
            "arm_right_3_joint",
            "arm_right_4_joint",
            "arm_right_5_joint",
            "arm_right_6_joint",
            "arm_right_7_joint",
        ],
    )

    torso_cmd_pub_ = rospy.Publisher(
        "/torso_controller/command", JointTrajectory, queue_size=1
    )
    torso_cmd_msg_ = init_cmd_msg(torso_cmd_msg_, ["torso_lift_joint"])

    mobile_base_cmd_pub_ = rospy.Publisher(
        "/mobile_base_controller/cmd_vel", Twist, queue_size=1
    )

    # Get (possible) enable/disable parameter and set up service to change it
    if rospy.has_param("~send_commands"):
        send_commands_ = rospy.get_param("~send_commands")
    rospy.Service("ros_control_bridge/send_commands", SetBool, set_send_commands_cb)
    rospy.loginfo(f"send_commands_: {send_commands_}")

    # Get (possible) 'time_from_start' parameter and set up subscriber to change it
    if rospy.has_param("~time_from_start"):
        time_from_start_ = rospy.get_param("~time_from_start")
    rospy.loginfo(f"time_from_start: {time_from_start_}")
    rospy.Subscriber(
        "ros_control_bridge/time_from_start", Float32, set_time_from_start_cb
    )

    # Set up subscriber to CartesI/O solution topic
    rospy.Subscriber("cartesian/solution", JointState, io_callback)

    # Set up a subscriber to kill the node
    rospy.Subscriber(
        "/streamdeck/ros_control_bridge_initiator", Bool, control_initiator_cb
    )

    # Get (possible) 'wrist_force_limit' parameter and set up ft subscribers for emergency
    if rospy.has_param("~wrist_force_limit"):
        wrist_force_limit = rospy.get_param("~wrist_force_limit")
    if rospy.has_param("~wrist_torque_limit"):
        wrist_torque_limit = rospy.get_param("~wrist_torque_limit")
    rospy.loginfo(f"wrist_force_limit: {wrist_force_limit} N")
    rospy.loginfo(f"wrist_torque_limit: {wrist_torque_limit} Nm")
    rospy.Subscriber("wrist_left_ft/corrected", WrenchStamped, left_ft_cb)
    rospy.Subscriber("wrist_right_ft/corrected", WrenchStamped, right_ft_cb)
    rospy.Subscriber("ros_control_bridge/emergency", Bool, em_cb)
    rospy.Subscriber("ros_control_bridge/restart", Bool, restart_cb)
    while not rospy.is_shutdown():
        if exit_:
            break
        rospy.spin()

    rospy.loginfo("Exiting ros_control_bridge")
    os._exit(0)
