#!/usr/bin/python3

import rospy
import tf
from nav_msgs.msg import Odometry

br = tf.TransformBroadcaster()

def callback(odom_msg):
    x = odom_msg.pose.pose.orientation.x
    y = odom_msg.pose.pose.orientation.y
    z = odom_msg.pose.pose.orientation.z
    w = odom_msg.pose.pose.orientation.w

    i = odom_msg.pose.pose.position.x
    j = odom_msg.pose.pose.position.y
    k = odom_msg.pose.pose.position.z

    br.sendTransform(
        (i, j, k),
        (x,y,z,w),
        rospy.Time.now(),
        odom_msg.child_frame_id,
        odom_msg.header.frame_id,
    )

if __name__ == "__main__":
    rospy.init_node("odom_broadcaster")
    sub = rospy.Subscriber("/odom", Odometry, callback)
    rospy.spin()