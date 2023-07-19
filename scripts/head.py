#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int64
import time
class Head_Saver:
    def __init__(self):
        rospy.Subscriber('/leo_bot/head/absolute_position', Int64,callback=self.head_pose, queue_size=1) 
        self.pub = rospy.Publisher('/leo_bot/head/goal_position', Int64, queue_size = 10)
        self.counter = 0 
        self.goal = Int64()

    def head_pose(self, msg):
        time.sleep(3)
        if(self.counter % 2 == 0):
            self.goal = 270
            
        else:
            self.goal = 90
        self.counter = self.counter + 1
        self.pub.publish(self.goal)

        
if __name__ == '__main__':
    rospy.init_node('Head_control')

    server = Head_Saver()
    
    rospy.spin()