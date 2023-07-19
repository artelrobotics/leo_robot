#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Range, LaserScan
import time
class Sonar_to_Laser:
    def __init__(self):
        self.time_now = 0
        rospy.Subscriber('/leo_bot/ultrasound_1', Range, callback=self.range_callback_1)
        self.pub_1 = rospy.Publisher("/leo_bot/sonar_to_lidar_1", LaserScan, queue_size=10)
        rospy.Subscriber('/leo_bot/ultrasound_2', Range, callback=self.range_callback_2)
        self.pub_2 = rospy.Publisher("/leo_bot/sonar_to_lidar_2", LaserScan, queue_size=10)
        rospy.Subscriber('/leo_bot/ultrasound_3', Range, callback=self.range_callback_3)
        self.pub_3 = rospy.Publisher("/leo_bot/sonar_to_lidar_3", LaserScan, queue_size=10)
        rospy.Subscriber('/leo_bot/ultrasound_4', Range, callback=self.range_callback_4)
        self.pub_4 = rospy.Publisher("/leo_bot/sonar_to_lidar_4", LaserScan, queue_size=10)
        self.laser_1 = LaserScan()
        self.laser_2 = LaserScan()
        self.laser_3 = LaserScan()
        self.laser_4 = LaserScan()
        self.laser_1.angle_min = 0
        self.laser_1.angle_max = 0.1
        self.laser_1.angle_increment = 0.02
        self.laser_1.range_max = 0.5
        self.laser_1.scan_time = 0.041
        self.laser_2.angle_min = 0
        self.laser_2.angle_max = 0.1
        self.laser_2.angle_increment = 0.02
        self.laser_2.range_max = 0.5
        self.laser_2.scan_time = 0.041
        self.laser_3.angle_min = 0
        self.laser_3.angle_max = 0.1
        self.laser_3.angle_increment = 0.02
        self.laser_3.range_max = 0.5
        self.laser_3.scan_time = 0.041
        self.laser_4.angle_min = 0
        self.laser_4.angle_max = 0.1
        self.laser_4.angle_increment = 0.02
        self.laser_4.range_max = 0.5
        self.laser_4.scan_time = 0.041
        
    def range_callback_1(self, msg:Range):
        self.laser_1.header = msg.header
        self.laser_1.time_increment = msg.header.stamp.secs - self.time_now
        self.laser_1.range_max = msg.max_range
        self.laser_1.range_min = msg.min_range
        if(msg.range >= 0.5):
            self.laser_1.ranges = [float('inf'),float('inf'),float('inf'),float('inf'),float('inf')]
            self.laser_1.intensities = [0, 0, 0, 0, 0]
        else:
            self.laser_1.ranges = [msg.range,msg.range,msg.range,msg.range,msg.range]
            self.laser_1.intensities = [47.0, 47.0, 47.0, 47.0, 47.0]
        self.pub_1.publish(self.laser_1)
        self.time_now = msg.header.stamp.secs

    def range_callback_2(self, msg:Range):
        self.laser_2.header = msg.header
        self.laser_2.time_increment = msg.header.stamp.secs - self.time_now
        self.laser_2.range_max = msg.max_range
        self.laser_2.range_min = msg.min_range
        self.laser_2.ranges = [msg.range,msg.range,msg.range,msg.range,msg.range]
        self.pub_2.publish(self.laser_2)
        self.time_now = msg.header.stamp.secs
    def range_callback_3(self, msg:Range):
        self.laser_3.header = msg.header
        self.laser_3.time_increment = msg.header.stamp.secs - self.time_now
        self.laser_3.range_max = msg.max_range
        self.laser_3.range_min = msg.min_range
        self.laser_3.ranges = [msg.range,msg.range,msg.range,msg.range,msg.range]
        self.pub_3.publish(self.laser_3)
        self.time_now = msg.header.stamp.secs
    def range_callback_4(self, msg:Range):
        self.laser_4.header = msg.header
        self.laser_4.time_increment = msg.header.stamp.secs - self.time_now
        self.laser_4.range_max = msg.max_range
        self.laser_4.range_min = msg.min_range
        self.laser_4.ranges = [msg.range,msg.range,msg.range,msg.range,msg.range]
        self.pub_4.publish(self.laser_4)
        self.time_now = msg.header.stamp.secs
if __name__ == "__main__":
    rospy.init_node("Sonar_to_Laser")
    convert = Sonar_to_Laser()
    rospy.spin()