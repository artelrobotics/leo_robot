#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from odrive_driver.msg import Status
from robotnik_msgs.msg import BatteryStatus
from std_msgs.msg import Int64
from leo_robot.msg import Robot_Status
class Robot:
    def __init__(self):
        rospy.Subscriber('driver/status', Status, callback=self.driver_callback, queue_size=1)
        rospy.Subscriber('daly_bms/data', BatteryStatus, callback=self.bms_callback, queue_size=1)
        rospy.Subscriber('head/absolute_position', Int64, callback=self.head_pose_callback, queue_size=1)
        self.robot_status_pub = rospy.Publisher("robot_status", Robot_Status, queue_size=5)
        self.robot_status = Robot_Status()
        self.robot_status.head_pose = "No data"
        self.robot_status.driver_status = "No data"
        self.robot_status.battery_percentage = "No data"
        self.robot_status.is_charging = "No data"
        
        
    def bms_callback(self, msg:BatteryStatus):
        self.robot_status.battery_percentage = round(msg.level, 2)
        self.robot_status.is_charging = str(msg.is_charging)

    def driver_callback(self, msg:Status):
        if(msg.system_error == 1):
            self.robot_status.driver_status = "Emergency is ON"

        elif(msg.left_encoder_error > 0 or msg.system_error > 0 or msg.right_axis_error > 0 or msg.right_motor_error > 0 
            or msg.right_sensorless_estimator_error > 0 or msg.right_encoder_error > 0 or msg.right_controller_error > 0 or msg.left_axis_error > 0
            or msg.left_motor_error > 0 or msg.left_sensorless_estimator_error > 0 or msg.left_controller_error > 0):
   
            
            self.robot_status.driver_status = "Driver has error"
        else:
            self.robot_status.driver_status = "Driver is properly working"

    def head_pose_callback(self, msg:Int64):
        self.robot_status.head_pose =  f"{msg.data} Degree"

    def status_publishing(self):
        self.robot_status_pub.publish(self.robot_status)

if __name__ == "__main__":
    rospy.init_node("Robot_status")
    r = rospy.Rate(20)
    status = Robot()
    while not rospy.is_shutdown():
        status.status_publishing()
        r.sleep()
    rospy.spin()