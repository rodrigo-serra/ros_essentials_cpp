#!/usr/bin/env python
# license removed for brevity
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from std_srvs.srv import Empty
import time, math

class MoveDistTurlesim:
    def __init__(self):
        rospy.init_node('move_dist_turlesim', anonymous=True)
        self.pub_vel = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        self.sub_pose = rospy.Subscriber('/turtle1/pose', Pose, self.poseCallback)
        self.x = 0
        self.y = 0
        self.yaw = 0
        self.xf = 3
        self.yf = 3
        self.k_linear = 0.5
        self.k_angular = 4
        time.sleep(2)

    def poseCallback(self, topic_data):
        self.x = topic_data.x
        self.y = topic_data.y
        self.yaw = topic_data.theta
        # rospy.loginfo("(x, y): " + '(' + str(topic_data.x) + ', ' + str(topic_data.y) + ')')

    def control(self, linear_vel, angular_vel):
        msg = Twist()
        msg.linear.x = linear_vel
        msg.angular.z = angular_vel
        self.pub_vel.publish(msg)
        

if __name__ == '__main__':
    controller = MoveDistTurlesim()
    # Publishing 10 times a second
    rate = rospy.Rate(10)
    
    try:
        while True:
            distance = abs(math.sqrt(((controller.xf - controller.x)** 2) + ((controller.yf - controller.y)**2)))
            vel = distance * controller.k_linear
            rospy.loginfo(str(distance))

            desired_angle_goal = math.atan2((controller.yf - controller.y), (controller.xf - controller.x))
            angVel = (desired_angle_goal - controller.yaw) * controller.k_angular
            # rospy.loginfo(str(desired_angle_goal))

            controller.control(vel, angVel)
            rate.sleep()
            
            # IF DISTANCE TO THE GOAL
            if distance < 0.01:
                rospy.loginfo('REACHED')
                break
        
        #Force the robot to stop
        controller.control(0, 0)

        time.sleep(2)
        rospy.wait_for_service('reset')
        reset_turtle = rospy.ServiceProxy('reset', Empty)
        reset_turtle()
        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.loginfo('Got interrupt request')

    rospy.loginfo('Closing remote controller')