#!/usr/bin/env python
import rospy
from nav_msgs.msg import Path
import numpy as np
from std_msgs.msg import Int64
from move_base_msgs.msg import MoveBaseActionGoal

class Path_distance:
    def __init__(self):
        rospy.Subscriber("/leo_bot/move_base/NavfnROS/plan", Path, self.path_cb)
        rospy.Subscriber("/leo_bot/move_base/goal", MoveBaseActionGoal, self.goal_cb)    
        self.pub = rospy.Publisher("/leo_bot/head/goal_position",Int64, queue_size=10)
        self.count = 0
        self.head_goal_pos = Int64()
        self.head_rotated = False
        

        
    def path_cb(self, msg):
        # global path_length
        self.path_length = 0
        if len(msg.poses) <= 100:
            for i in range(len(msg.poses)-1):
                position_a_x = msg.poses[i].pose.position.x
                position_b_x = msg.poses[i+1].pose.position.x
                position_a_y = msg.poses[i].pose.position.y
                position_b_y = msg.poses[i+1].pose.position.y

                self.path_length += np.sqrt(np.power((position_b_x - position_a_x), 2) + np.power((position_b_y- position_a_y), 2))
            
            rospy.loginfo(f"path_length: {round(self.path_length, 2)}  poses: {len(msg.poses)}")
            
            if(self.path_length < 2 and self.head_rotated == False):
                if(self.count % 2 == 0):
                    self.head_goal_pos.data = 90
                else:
                    self.head_goal_pos.data = 270
                self.pub.publish(self.head_goal_pos)
                self.head_rotated = True    
            

    def goal_cb(self, msg):
        self.count = self.count + 1
        self.head_rotated = False

if __name__ == "__main__":
    rospy.init_node("path_listener")
    r = rospy.Rate(20)
    path_distance = Path_distance()
    rospy.spin()            