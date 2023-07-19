#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
import time
from std_srvs.srv import SetBool
class Initial_Pose:
    def __init__(self):
        self.rate = rospy.Rate(10)
        self.frame_id = "leo_bot/map"
        self.pose_pub = rospy.Publisher("/leo_bot/initialpose", PoseWithCovarianceStamped, queue_size=1)
        self._as = rospy.Service('/leo_bot/set_initial_pose', SetBool, handler=self.initial_pose)
        self.pose_msg = PoseWithCovarianceStamped()
    
    def initial_pose(self, msg):
        self.pose_msg.header.stamp = rospy.Time.now()
        self.pose_msg.header.frame_id = self.frame_id
        self.pose_msg.pose.pose.position.x = 0.348
        self.pose_msg.pose.pose.position.y = -0.825
        self.pose_msg.pose.pose.position.z = 0
        self.pose_msg.pose.pose.orientation.x = 0.0
        self.pose_msg.pose.pose.orientation.y = -0.0
        self.pose_msg.pose.pose.orientation.z = 0.997
        self.pose_msg.pose.pose.orientation.w = -0.079
        self.pose_msg.pose.covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 
                                         0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
                                         0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                         0.0, 0.0, 0.06853892326654787]
        
        self.pose_pub.publish(self.pose_msg)
        self.rate.sleep()

            
if __name__ == "__main__":
    rospy.init_node("Init_pose")
    init_pose = Initial_Pose()
    rospy.spin()
