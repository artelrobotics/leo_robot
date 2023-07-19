#!/usr/bin/env python
import rospy
from std_msgs.msg import Int64
from move_base_msgs.msg import MoveBaseActionGoal
from actionlib_msgs.msg import GoalStatusArray
from geometry_msgs.msg import PoseStamped
import Euler
class Acurate_Goal_Reach:
    def __init__(self):
        rospy.Subscriber("/leo_bot/move_base/goal", MoveBaseActionGoal, self.goal_cb)
        rospy.Subscriber("/leo_bot/move_base/status", GoalStatusArray, self.status_cb)
        rospy.Subscriber("/leo_bot/robot_pose", PoseStamped, self.pose_cb)    
        self.pub = rospy.Publisher("/leo_bot/head/goal_position",Int64, queue_size=10)
        self.pose.x = 0
        self.pose.x = 0
        self.rot.z = 0
        

        
    def status_cb(self, msg):
        msg.     
            
    def pose_cb(self, msg):
        self.pose.x = msg.pose.position.x
        self.pose.y = msg.pose.position.y
        self.rot.z = 

    def goal_cb(self, msg):
        self.count = self.count + 1
        self.head_rotated = False

if __name__ == "__main__":
    rospy.init_node("Acurate_Goal_Reacher")
    r = rospy.Rate(20)
    goal = Acurate_Goal_Reach()
    rospy.spin()            