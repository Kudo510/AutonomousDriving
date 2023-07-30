#!/usr/bin/env python3
import rospy
import math
import time
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import quaternion_from_euler

class MoveBaseSeq():

    def __init__(self):

        rospy.init_node('move_base_sequence')
        points_seq = rospy.get_param('move_base_seq/p_seq')
        yaweulerangles_seq = rospy.get_param('move_base_seq/yea_seq')
        quat_seq = list()
        self.pose_seq = list()
        self.goal_cnt = 0
        for yawangle in yaweulerangles_seq:
            quat_seq.append(Quaternion(*(quaternion_from_euler(0, 0, yawangle*math.pi/180, axes='sxyz'))))
        n = 3
        points = [points_seq[i:i+n] for i in range(0, len(points_seq), n)]
        rospy.loginfo(str(points))
        for point in points:
            self.pose_seq.append(Pose(Point(*point),quat_seq[n-3]))
            n += 1
        self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        wait = self.client.wait_for_server()
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
            return
        rospy.loginfo("Connected to move base server")
        rospy.loginfo("Starting goals achievements ...")
        self.movebase_client()

    def movebase_client(self):   
        for i in range(len(self.pose_seq)): 
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = "world"
            goal.target_pose.header.stamp = rospy.Time.now() 
            goal.target_pose.pose = self.pose_seq[i]
            rospy.loginfo("Sending goal pose "+str(i+1)+" to Action Server")
            rospy.loginfo(str(self.pose_seq[i]))
            
            # Use send_goal_and_wait instead of send_goal
            self.client.send_goal(goal)
            result = self.client.wait_for_result()
            
            if result:
                state = self.client.get_state()
                if state == GoalStatus.SUCCEEDED:
                    rospy.loginfo("Goal pose "+str(i+1)+" reached")
                elif state == GoalStatus.PREEMPTED:
                    rospy.loginfo("Goal pose "+str(i+1)+" was preempted")
                else:
                    rospy.loginfo("Goal pose "+str(i+1)+" failed")
            else:
                rospy.logerr("Action server not available!")
                rospy.signal_shutdown("Action server not available!")
                break

if __name__ == '__main__':
    try:
        MoveBaseSeq()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation finished.")
