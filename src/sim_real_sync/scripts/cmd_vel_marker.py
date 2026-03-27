#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Point, Twist
from visualization_msgs.msg import Marker


class CmdVelMarkerPublisher:
    def __init__(self):
        self.cmd_vel_topic = rospy.get_param("~cmd_vel_topic", "/cmd_vel")
        self.marker_topic = rospy.get_param("~marker_topic", "/rviz/cmd_marker")
        self.frame_id = rospy.get_param("~frame_id", "base_link")
        self.linear_scale = float(rospy.get_param("~linear_scale", 2.0))

        self.publisher = rospy.Publisher(self.marker_topic, Marker, queue_size=10)
        self.subscriber = rospy.Subscriber(self.cmd_vel_topic, Twist, self.cb_cmd, queue_size=10)

        rospy.loginfo("cmd_vel_marker started: %s -> %s", self.cmd_vel_topic, self.marker_topic)

    def cb_cmd(self, msg):
        marker = Marker()
        marker.header.stamp = rospy.Time.now()
        marker.header.frame_id = self.frame_id
        marker.ns = "cmd_vel"
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.05
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.a = 0.9
        marker.color.r = 0.15
        marker.color.g = 0.8
        marker.color.b = 0.2

        start = Point(0.0, 0.0, 0.08)
        end = Point(msg.linear.x * self.linear_scale, msg.linear.y * self.linear_scale, 0.08)

        if abs(end.x) < 1e-4 and abs(end.y) < 1e-4:
            end.x = 0.001

        marker.points = [start, end]
        self.publisher.publish(marker)


if __name__ == "__main__":
    rospy.init_node("cmd_vel_marker")
    CmdVelMarkerPublisher()
    rospy.spin()
