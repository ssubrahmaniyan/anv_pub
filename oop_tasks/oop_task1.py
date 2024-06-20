#!/usr/bin/env python3

import rospy
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from math import pi

class TurtleController:
    def __init__(self):
        rospy.init_node("controller_node")
        rospy.loginfo("node initialized")
        self.err = 0.001
        self.pub = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=10)
        self.sub = rospy.Subscriber("/turtle1/pose", Pose, callback= self.pose_callback)

    def pose_callback(self, msg: Pose):
        cmd = Twist()
        if abs(msg.y - 7.544) >= self.err and msg.theta >= -0.01:
            cmd.linear.x = 1
            cmd.angular.z = 1
        else:
            if abs(msg.theta + pi / 2) >= 10 * self.err:
                cmd.linear.x = 0
                cmd.angular.z = 1
            elif abs(msg.y - 5.544) >= self.err:
                cmd.linear.x = 1
                cmd.angular.z = 0
            else:
                cmd.linear.x = 0
                cmd.angular.z = 0
        self.pub.publish(cmd)

    def run(self):
        rospy.spin()

if __name__ == "__main__":
    controller = TurtleController()
    controller.run()

