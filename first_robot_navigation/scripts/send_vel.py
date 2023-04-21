#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist

def cmd_cb(msg):
      global x, theta
      x = msg.linear.x
      y = msg.linear.y
      z = msg.linear.z
      theta = msg.angular.z 

if __name__ == '__main__' :
    rospy.init_node("cmd_vel_sub")
    rospy.loginfo("Node has been started.")

    goal = rospy.Subscriber("/sim_p3at/cmd_vel", Twist, cmd_cb)
    
    rate = rospy.Rate(2)

    while not rospy.is_shutdown():
        # Publish cmd vel
        goal = Twist()

        rate.sleep()
