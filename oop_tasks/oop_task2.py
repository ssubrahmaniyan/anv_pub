#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import Kill, Spawn
from math import atan2, sqrt

class Turtle:
    def __init__(self):
        rospy.init_node("chasernode")
        rospy.loginfo("chaser node initiated")
        
        self.x_init = 5.5445
        self.y_init = 5.5445
        self.x_target = 2
        self.y_target = 5
        self.lion_pose = None
        
        self.cmd_pub = rospy.Publisher("/Lion/cmd_vel", Twist, queue_size=10)
        rospy.Subscriber("/Lion/pose", Pose, self.lion_pose_callback)
        
        rospy.wait_for_service("/kill")
        self.call_kill_service("turtle1")
        
        rospy.wait_for_service("/spawn")
        self.call_spawn_service(self.x_init, self.y_init, 0, "Lion")
        
        rospy.wait_for_service("/spawn")
        self.call_spawn_service(self.x_target, self.y_target, 0, "Deer")

    def call_kill_service(self, name):
        try:
            killer = rospy.ServiceProxy("/kill", Kill)
            _ = killer(name)
        except rospy.ServiceException as e:
            rospy.logerr(e)

    def call_spawn_service(self, x, y, theta, name):
        try:
            spawner = rospy.ServiceProxy('/spawn', Spawn)
            _ = spawner(x, y, theta, name)
        except rospy.ServiceException as e:
            rospy.logerr(e)

    def lion_pose_callback(self, msg):
        self.lion_pose = msg
        if self.lion_pose:
            cmd = Twist()
            distance = sqrt((self.x_target - self.lion_pose.x)**2 + (self.y_target - self.lion_pose.y)**2)
            
            if distance < 0.1:
                cmd.linear.x = 0
                cmd.angular.z = 0
                self.cmd_pub.publish(cmd)
                rospy.loginfo("Deer caught by Lion hahaha")
                rospy.signal_shutdown("Deer caught by lion")
                return
            
            angle_to_target = atan2(self.y_target - self.lion_pose.y, self.x_target - self.lion_pose.x)
            
            angle_to_go = angle_to_target - self.lion_pose.theta
            
            if abs(angle_to_go) > 0.01: 
                cmd.linear.x = 0
                cmd.angular.z = angle_to_go
            else:
                cmd.linear.x = distance
                cmd.angular.z = 0
            self.cmd_pub.publish(cmd)
        
    def run(self):
        rospy.spin()

if __name__ == "__main__":
    node = Turtle()
    node.run()
