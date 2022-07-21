#!/usr/bin/env python
import rospy
from turtlesim.msg import Pose

class PoseTurtlesim:
    def __init__(self):
        rospy.init_node('pose_turtlesim')
        self.sub_pose = rospy.Subscriber('/turtle1/pose', Pose, self.poseCallback)

    def poseCallback(self, topic_data):
        rospy.loginfo("(x, y): " + '(' + str(topic_data.x) + ', ' + str(topic_data.y) + ')')



if __name__ == '__main__':
    pose = PoseTurtlesim()
    rospy.spin()