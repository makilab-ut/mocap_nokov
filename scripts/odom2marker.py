#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker

class OdomArrowMarkerPublisher:
    def __init__(self):
        rospy.init_node('odom_arrow_marker_publisher')

        # Publisher for the marker
        self.marker_pub1 = rospy.Publisher('/odom_marker1', Marker, queue_size=10)
        self.marker_pub2 = rospy.Publisher('/odom_marker2', Marker, queue_size=10)

        # Subscriber to the odometry topic
        rospy.Subscriber('/mocap_node/Robot_1/Odom', Odometry, self.odom_callback1)
        rospy.Subscriber('/mocap_node/Robot_2/Odom', Odometry, self.odom_callback2)

        # Set up the basic marker structure
        self.marker1 = Marker()
        self.marker1.header.frame_id = "world"  # Adjust if using "map" or "odom"
        self.marker1.ns = "robot"
        self.marker1.id = 0
        self.marker1.type = Marker.ARROW
        self.marker1.action = Marker.ADD

        # Arrow dimensions (length = x, width = y, height = z)
        self.marker1.scale.x = 1.0  # Length of arrow shaft
        self.marker1.scale.y = 0.1  # Width
        self.marker1.scale.z = 0.1  # Height

        # Arrow color
        self.marker1.color.a = 1.0
        self.marker1.color.r = 0.0
        self.marker1.color.g = 1.0
        self.marker1.color.b = 0.0

        self.marker2 = Marker()
        self.marker2.header.frame_id = "world"  # Adjust if using "map" or "odom"
        self.marker2.ns = "robot"
        self.marker2.id = 0
        self.marker2.type = Marker.ARROW
        self.marker2.action = Marker.ADD

        # Arrow dimensions (length = x, width = y, height = z)
        self.marker2.scale.x = 1.0  # Length of arrow shaft
        self.marker2.scale.y = 0.1  # Width
        self.marker2.scale.z = 0.1  # Height
        self.marker2.color.a = 1.0
        self.marker2.color.r = 1.0
        self.marker2.color.g = 0.0
        self.marker2.color.b = 0.0

    def odom_callback1(self, msg):
        pos = msg.pose.pose.position
        self.marker1.header.stamp = rospy.Time.now()
        self.marker1.pose.position = pos
        self.marker1.pose.orientation = msg.pose.pose.orientation
        self.marker_pub1.publish(self.marker1)

    def odom_callback2(self, msg):
        pos = msg.pose.pose.position
        self.marker2.header.stamp = rospy.Time.now()
        self.marker2.pose.position = pos
        self.marker2.pose.orientation = msg.pose.pose.orientation
        self.marker_pub2.publish(self.marker2)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = OdomArrowMarkerPublisher()
        node.run()
    except rospy.ROSInterruptException:
        pass
