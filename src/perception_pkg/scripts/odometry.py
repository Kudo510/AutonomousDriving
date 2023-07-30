#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, TwistStamped, Vector3, Quaternion

import numpy as np
from tf.transformations import quaternion_matrix

def pose_callback(pose_msg_stamped):
    global odom_msg
    # Process the received PoseStamped message and update the Odometry message
    pose_msg = pose_msg_stamped.pose
    odom_msg.pose.pose.position = pose_msg.position
    odom_msg.pose.pose.orientation = pose_msg.orientation

def twist_callback(twist_msg_stamped):
    global odom_msg
    twist_msg = twist_msg_stamped.twist
    v = np.array([twist_msg.linear.x, twist_msg.linear.y, twist_msg.linear.z])
    omega = np.array([twist_msg.angular.x, twist_msg.angular.y, twist_msg.angular.z])

    q = Quaternion(odom_msg.pose.pose.orientation.x,
                   odom_msg.pose.pose.orientation.y,
                   odom_msg.pose.pose.orientation.z,
                   odom_msg.pose.pose.orientation.w)
    R = quaternion_matrix([q.x, q.y, q.z, q.w])[:3, :3]

    omega = np.dot(R.transpose(), omega)  # Rotate omega
    v = np.dot(R.transpose(), v)          # Rotate v

    # Still not working together with controller_node.cpp (line 110)
    # velocity_control = -linear_vel - v(0);
    # Therefore, setting it to zero until it works out
    odom_msg.twist.twist.linear.x = 5 #v[0]
    odom_msg.twist.twist.linear.y = 0 #v[1]
    odom_msg.twist.twist.linear.z = 0 #v[2]

    # Still not working together with controller_node.cpp (line 128)
    # omega_control  = (-angular_vel) - omega[2];
    # Therefore, setting it to zero until it works out
    odom_msg.twist.twist.angular.x = 0 #omega[0]
    odom_msg.twist.twist.angular.y = 0 #omega[1]
    odom_msg.twist.twist.angular.z = 0 #omega[2]


if __name__ == '__main__':
    rospy.init_node('odometry_publisher')

    # Create a publisher for the odometry topic
    odom_pub = rospy.Publisher('/odom', Odometry, queue_size=1)

    # Create an Odometry message object
    odom_msg = Odometry()

    # The position, orientation and velocity are given in the world frame
    odom_msg.header.frame_id = 'world'

    # The reference frame that is attached to the car is the 'body'frame
    odom_msg.child_frame_id = 'body'

    # The odometry message says: the 'body' frame has the pose in the 'world' frame
    # and the velocity in the body frame

    # Create subscribers for the PoseStamped and TwistStamped topics
    rospy.Subscriber('/true_pose', PoseStamped, pose_callback)
    rospy.Subscriber('/true_twist', TwistStamped, twist_callback)

    rate = rospy.Rate(50)  # Publish at 10 Hz

    while not rospy.is_shutdown():
        # Publish the Odometry message
        odom_pub.publish(odom_msg)
        rate.sleep()


