#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion

class ImuStillZero:
    def __init__(self):
        rospy.Subscriber('d455_camera/camera/imu', Imu, callback=self.imu_callback, queue_size=1)
        rospy.Subscriber('encoder/odometry', Odometry, callback=self.odom_callback, queue_size=1)
        self._imu_pub = rospy.Publisher("imu/data_raw", Imu, queue_size=5)
        self.linear_x = 0.0
        self.angular_z = 0.0
        self.is_initialized = False
        self.counter = 0
        self.last_orientation = Quaternion()

    def imu_callback(self, msg:Imu):
        imu_to_pub = Imu()
        if self.is_initialized and self.counter > 200:
            if self.linear_x < 0.001 and self.angular_z < 0.001:
                rospy.loginfo(f'linear x {self.linear_x}  angular_z {self.angular_z}')
                imu_to_pub = msg
                # imu_to_pub.orientation = self.last_orientation
                imu_to_pub.angular_velocity.x = 0.0
                imu_to_pub.angular_velocity.y = 0.0
                imu_to_pub.angular_velocity.z = 0.0
                imu_to_pub.linear_acceleration.x = 0.0
                # imu_to_pub.linear_acceleration.y = 0.0
                imu_to_pub.linear_acceleration.z = 0.0
                self._imu_pub.publish(imu_to_pub)
            else:
                self._imu_pub.publish(msg)
                self.last_orientation = msg.orientation
        else:
            self._imu_pub.publish(msg)
            rospy.loginfo(f'counter {self.counter}')
            # self.last_orientation = msg.orientation
            self.is_initialized = True
            self.counter += 1
    def odom_callback(self, msg:Odometry):
        self.linear_x = msg.twist.twist.linear.x
        self.angular_z = msg.twist.twist.angular.z


if __name__ == "__main__":
    rospy.init_node("imu_zero_still")
    imu = ImuStillZero()
    rospy.spin()