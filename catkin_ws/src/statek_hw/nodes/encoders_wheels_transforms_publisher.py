#!/usr/bin/env python
import rospy
import tf_conversions
import tf2_ros
import geometry_msgs.msg

from statek_hw.msg import Encoder

def send_transform(parent_link, child_link, x, y, z, rx, ry, rz):
    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()

    t.header.stamp = rospy.Time.now()
    t.header.frame_id = parent_link
    t.child_frame_id = child_link
    t.transform.translation.x = x
    t.transform.translation.y = y
    t.transform.translation.z = z
    q = tf_conversions.transformations.quaternion_from_euler(rx, ry, rz)
    t.transform.rotation.x = q[0] 
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]

    br.sendTransform(t)

def encoder_callback(msg, args):
    statek_name = args[0]
    tf_dict = args[1]

    parent_link = statek_name + "/base_link"

    send_transform(parent_link, tf_dict["motor_link"], tf_dict["motor"][0], tf_dict["motor"][1], tf_dict["motor"][2], msg.position, tf_dict["motor"][4], tf_dict["motor"][5])
    send_transform(parent_link, tf_dict["front_wheel_link"], tf_dict["front_wheel"][0], tf_dict["front_wheel"][1], tf_dict["front_wheel"][2], msg.position, tf_dict["front_wheel"][4], tf_dict["front_wheel"][5])
    send_transform(parent_link, tf_dict["back_wheel_link"], tf_dict["back_wheel"][0], tf_dict["back_wheel"][1], tf_dict["back_wheel"][2], msg.position, tf_dict["back_wheel"][4], tf_dict["back_wheel"][5])

def string_to_list(str):
    return [float(item) for item in str.split()]

rospy.init_node("dynamic_transform_publisher", anonymous=True)

statek_name = rospy.get_param("~statek_name", "statek")

left_encoder_topic = rospy.get_param("~left_encoder_topic", "/real_time/motors/left/encoder")
right_encoder_topic = rospy.get_param("~right_encoder_topic", "/real_time/motors/right/encoder")

left_motor_tf = string_to_list(rospy.get_param("~base_to_left_motor"))
right_motor_tf = string_to_list(rospy.get_param("~base_to_right_motor"))

left_back_wheel_tf = string_to_list(rospy.get_param("~base_to_left_back_wheel"))
left_front_wheel_tf = string_to_list(rospy.get_param("~base_to_left_front_wheel"))
right_back_wheel_tf = string_to_list(rospy.get_param("~base_to_right_back_wheel"))
right_front_wheel_tf = string_to_list(rospy.get_param("~base_to_right_front_wheel"))

left_side_tf = {
    "motor": left_motor_tf,
    "back_wheel": left_back_wheel_tf,
    "front_wheel": left_front_wheel_tf,
    "motor_link": statek_name + "/motors/left_link",
    "back_wheel_link": statek_name + "/wheels/left_back_link",
    "front_wheel_link": statek_name + "/wheels/left_front_link"
}

right_side_tf = {
    "motor": right_motor_tf,
    "back_wheel": right_back_wheel_tf,
    "front_wheel": right_front_wheel_tf,
    "motor_link": statek_name + "/motors/right_link",
    "back_wheel_link": statek_name + "/wheels/right_back_link",
    "front_wheel_link": statek_name + "/wheels/right_front_link"
}

rospy.Subscriber("/" + statek_name + left_encoder_topic, Encoder, encoder_callback, (statek_name, left_side_tf), queue_size=1)
rospy.Subscriber("/" + statek_name + right_encoder_topic, Encoder, encoder_callback, (statek_name, right_side_tf), queue_size=1)

rospy.spin()
