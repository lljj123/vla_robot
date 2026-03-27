#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""vla_nav_node.py

MVP VLA navigation node (ROS1 Noetic):
- Accepts natural language commands (std_msgs/String) on /vla_nav/command
- Subscribes to camera image (sensor_msgs/Image) on ~image
- Publishes a nav goal (geometry_msgs/PoseStamped) to /move_base_simple/goal

This is a scaffold:
- 'VLA policy' is pluggable; for now we map a few phrases to demo goals.
- Later: replace with real VLA model using image+text.
"""

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped


class VlaNavNode:
    def __init__(self):
        self.frame_id = rospy.get_param("~frame_id", "map")
        self.last_image_stamp = None

        self.pub_goal = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)
        self.pub_status = rospy.Publisher("/vla_nav/status", String, queue_size=10)

        rospy.Subscriber("/vla_nav/command", String, self.on_command, queue_size=10)
        rospy.Subscriber("~image", Image, self.on_image, queue_size=1)

        self.pub_status.publish("ready")
        rospy.loginfo("vla_nav ready (frame_id=%s)", self.frame_id)

    def on_image(self, msg: Image):
        self.last_image_stamp = msg.header.stamp

    def on_command(self, msg: String):
        text = (msg.data or "").strip()
        if not text:
            return

        self.pub_status.publish(f"received:{text}")

        # MVP: simple phrase->goal mapping (meters in map frame)
        x, y = 1.0, 0.0
        if "forward" in text or "前" in text:
            x, y = 1.0, 0.0
        elif "left" in text or "左" in text:
            x, y = 0.0, 1.0
        elif "right" in text or "右" in text:
            x, y = 0.0, -1.0
        elif "back" in text or "后" in text:
            x, y = -1.0, 0.0

        if self.last_image_stamp is None:
            self.pub_status.publish("warn:no_image")
        else:
            age = (rospy.Time.now() - self.last_image_stamp).to_sec()
            if age > 2.0:
                self.pub_status.publish(f"warn:image_stale:{age:.2f}s")

        goal = PoseStamped()
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = self.frame_id
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.orientation.w = 1.0

        self.pub_goal.publish(goal)
        self.pub_status.publish(f"goal:{x:.2f},{y:.2f}")


def main():
    rospy.init_node("vla_nav")
    VlaNavNode()
    rospy.spin()


if __name__ == "__main__":
    main()
