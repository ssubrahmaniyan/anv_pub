#!/usr/bin/python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

class joystick:
    def __init__(self):
        rospy.init_node("controller")
        self.sub = rospy.Subscriber("/joy", Joy, callback=self.control_callback)
        self.pub = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size = 1)


    def control_callback(self,ctl: Joy):
        cmd = Twist()
        cmd.linear.x = ctl.axes[1]
        cmd.angular.z = ctl.axes[0]
        self.pub.publish(cmd)

    def run(self):
        rospy.spin()

if __name__ == "__main__":
    node = joystick()
    node.run()
