#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
import numpy as np
from scipy.ndimage import maximum_filter
from geometry_msgs.msg import Point

class Line_detect:
    def __init__(self):
        rospy.Subscriber("/leo_bot/scan", LaserScan, self.scan_cb)
        # self.pub = rospy.Publisher("/leo_bot/lines",, queue_size=10)
    
    def scan_cb(self, msg):
        lines = self.detect_lines(msg.ranges, msg.angle_min, msg.angle_increment)
        print(lines)
    def detect_lines(self, ranges, angle_min, angle_increment):
        # Convert the polar coordinates to Cartesian coordinates
        x = np.multiply(ranges, np.cos(np.arange(angle_min, angle_min + len(ranges) * angle_increment, angle_increment)))
        y = np.multiply(ranges, np.sin(np.arange(angle_min, angle_min + len(ranges) * angle_increment, angle_increment)))

        # Apply the Hough Transform
        accumulator = np.zeros((180, 200))
        for i in range(len(x)):
            for theta in range(0, 180):
                rho = int(x[i] * np.cos(np.deg2rad(theta)) + y[i] * np.sin(np.deg2rad(theta)))
                accumulator[theta][rho + 100] += 1

        # Find local maxima in the accumulator
        local_maxima = maximum_filter(accumulator, size=5) == accumulator

        # Get the indices of the local maxima
        theta_indices, rho_indices = np.nonzero(local_maxima)

        # Convert indices to theta and rho values
        theta_values = np.deg2rad(theta_indices)
        rho_values = rho_indices - 100

        # Create a list of detected lines
        lines = []
        for theta, rho in zip(theta_values, rho_values):
            point1 = Point()
            point1.x = rho * np.cos(theta)
            point1.y = rho * np.sin(theta)
            lines.append(point1)

        return lines

   

if __name__ == "__main__":
    rospy.init_node("Line_detect")
    r = rospy.Rate(20)
    line = Line_detect()
    rospy.spin()            