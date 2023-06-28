#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist

if __name__ == '__main__' :
    rospy.init_node("Draw_circle")
    rospy.loginfo("Node has been started.")

    pub = rospy.Publisher("/sim_p3at/cmd_vel", Twist, queue_size=10)
    
    rate = rospy.Rate(2)

    while not rospy.is_shutdown():
        # Publish cmd vel
        msg = Twist()
        msg.linear.x = 2.0
        msg.angular.z = 1.0
        pub.publish(msg)

        rate.sleep()
