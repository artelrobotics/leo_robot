#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
import time
class Acceleration_Calculation:
    def __init__(self):
        rospy.Subscriber('/leo_bot/encoder/odometry', Odometry, callback=self.odom_callback, queue_size=1)
        self.last_time = rospy.Time.now()
        self.test_start = False
        rospy.loginfo("Accelerstion calculation starts")
        self.last_speed = 0
        self.linear_test = False
        self.angular_test = False
    def odom_callback(self, msg:Odometry):
        self.linear_x = msg.twist.twist.linear.x
        self.angular_z = msg.twist.twist.angular.z
        self.current_time = rospy.Time.now()
        
        if(self.linear_test == False):
            if(self.linear_x > 0 and self.test_start == False and self.last_speed_x == 0):    
                self.test_start = True
            
            if(self.linear_x >= 1.2 and self.test_start == True):
                elapsed = self.current_time.to_sec() - self.last_time.to_sec()
                acceleration_linear_x = self.linear_x / elapsed
                rospy.loginfo("Accelerstion linear X is: %f", acceleration_linear_x)
                self.test_start = False
                self.linear_test = True
            if(self.linear_x == 0 and self.test_start == False):
                self.last_time = self.current_time
        elif(self.angular_test == False):
            if(self.angular_z > 0 and self.test_start == False and self.last_speed_z == 0):    
                self.test_start = True
            
            if(self.angular_z >= 2 and self.test_start == True):
                elapsed = self.current_time.to_sec() - self.last_time.to_sec()
                acceleration_angular_z = self.angular_z / elapsed
                rospy.loginfo("Accelerstion angular Z is: %f", acceleration_angular_z)
                self.test_start = False
                self.angular_test = True
            if(self.angular_z == 0 and self.test_start == False):
                self.last_time = self.current_time

        self.last_speed_x = self.linear_x
        self.last_speed_z = self.angular_z
            

if __name__ == "__main__":
    rospy.init_node("Acceleration_Calculation")
    acc_cal = Acceleration_Calculation()
    rospy.spin()