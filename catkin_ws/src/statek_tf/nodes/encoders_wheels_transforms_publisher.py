#!/usr/bin/env python
import rospy
import tf_conversions
import tf2_ros
import geometry_msgs.msg

from statek_msgs.msg import Encoder

def send_transform(link, x, y, z, rx):
    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()

    t.header.stamp = rospy.Time.now()
    t.header.frame_id = statek_name + "/base_link"
    t.child_frame_id = link
    t.transform.translation.x = x
    t.transform.translation.y = y
    t.transform.translation.z = z
    q = tf_conversions.transformations.quaternion_from_euler(rx, 0, 1.5708)
    t.transform.rotation.x = q[0] 
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]

    br.sendTransform(t)

def encoder_callback(msg, args):
    side = args[0]
    statek_name = args[1]

    motor_link = statek_name + "/motors/" + side + "_link"
    front_wheel_link = statek_name + "/wheels/" + side + "_front_link"
    back_wheel_link = statek_name + "/wheels/" + side + "_back_link"

    y_offset_motor = -0.262682 if side == "right" else 0.262682
    y_offset_wheel = -0.317682 if side == "right" else 0.317682

    send_transform(motor_link, -0.057, y_offset_motor, 0.0, msg.position)
    send_transform(front_wheel_link, 0.167807, y_offset_wheel, -0.03126, msg.position)
    send_transform(back_wheel_link, -0.252682, y_offset_wheel, -0.03126, msg.position)

rospy.init_node("dynamic_transform_publisher", anonymous=True)

statek_name = rospy.get_param("~statek_name", "statek")

rospy.Subscriber("/" + statek_name + "/motors/left/encoder", Encoder, encoder_callback, ("left", statek_name))
rospy.Subscriber("/" + statek_name + "/motors/right/encoder", Encoder, encoder_callback, ("right", statek_name))

rospy.spin()
