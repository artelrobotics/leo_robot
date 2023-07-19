#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TransformStamped, PoseStamped, Quaternion, Point
from std_msgs.msg import Header
import tf2_ros
import sys

class Tf_to_Pose:
    def __init__(self):
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)
        
        self.pose_pub = rospy.Publisher("robot_pose", PoseStamped, queue_size=1)
        self.child_frame = rospy.get_param("child_frame", default="leo_bot/base_footprint")
        self.parrent_frame = rospy.get_param("parrent_frame", default="leo_bot/map")
        self.pose_msg = PoseStamped()
    
    def get_transform(self) -> TransformStamped:
        try:
            can_transform = self._tf_buffer.can_transform(self.parrent_frame, self.child_frame, rospy.Time(), rospy.Duration(1.0))
            if can_transform:
                transform: TransformStamped = self._tf_buffer.lookup_transform(self.parrent_frame, self.child_frame, rospy.Time())
                return transform
            else:
                return None
        except tf2_ros.TransformException as ex:
            rospy.logwarn(str(ex))
            rospy.sleep(1.0)
 
    def to_pose(self):
        transform = self.get_transform()
        if transform is not None:
            self.pose_msg.header.frame_id = self.parrent_frame
            self.pose_msg.header.stamp = rospy.Time.now()
            self.pose_msg.pose.position.x = transform.transform.translation.x
            self.pose_msg.pose.position.y = transform.transform.translation.y
            self.pose_msg.pose.position.z = transform.transform.translation.z
            self.pose_msg.pose.orientation = transform.transform.rotation
        else:
            self.pose_msg.header.frame_id = self.parrent_frame
            self.pose_msg.header.stamp = rospy.Time.now()
        self.pose_pub.publish(self.pose_msg)

if __name__ == "__main__":
    rospy.init_node("tf_to_pose")
    freq = rospy.get_param("rate", default=1)
    rate = rospy.Rate(freq)
    robot_pose = Tf_to_Pose()
    while not rospy.is_shutdown():
        try:
            robot_pose.to_pose()
            rate.sleep()
        except KeyboardInterrupt:
            break
    else:
        sys.exit(0)
    rospy.spin()
