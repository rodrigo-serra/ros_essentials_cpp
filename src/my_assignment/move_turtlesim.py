#!/usr/bin/env python
# license removed for brevity
import rospy
from geometry_msgs.msg import Twist

class MoveTurlesim:
    def __init__(self):
        rospy.init_node('move_turlesim', anonymous=True)
        self.pub_vel = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)

    def control(self, linear_vel, angular_vel):
        msg = Twist()
        msg.linear.x = linear_vel
        msg.angular.z = angular_vel
        self.pub_vel.publish(msg)
        

if __name__ == '__main__':
    controller = MoveTurlesim()
    vel = 2
    angVel = 1
    rate = rospy.Rate(1)

    try:
        while not rospy.is_shutdown():
            controller.control(vel, angVel)
            rate.sleep()
    except rospy.ROSInterruptException:
        rospy.loginfo('Got interrupt request')

    rospy.loginfo('Closing remote controller')