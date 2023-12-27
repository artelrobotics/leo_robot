#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion


class DepthScanMod:
    def __init__(self):
        rospy.Subscriber('depth/scan', LaserScan, callback=self.scan_callback, queue_size=5)
        self.scan_pub = rospy.Publisher("depth/scan_filtered", LaserScan, queue_size=5)
        self.scan_filtered = LaserScan()
    
    def scan_callback(self, msg:LaserScan):
        self.scan_filtered = msg
        self.scan_filtered.intensities = [2 for i in range(len(msg.ranges))]
        # for x in range(length):
        #     if (msg.ranges[x] == 'inf'):
        #         self.scan_filtered.intensities.append[0]
        #     else:
        #         intensity.append[msg.range_max]
        
        self.scan_pub.publish(self.scan_filtered)
   

if __name__ == "__main__":
    rospy.init_node("Depth_Scan_Modification")
    imu = DepthScanMod()
    rospy.spin()