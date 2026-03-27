#!/usr/bin/env python3
import math
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


def clamp(v, vmin, vmax):
    return max(vmin, min(vmax, v))


def yaw_from_quat(q):
    # yaw (z-axis rotation)
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def norm_angle(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


class SyncController:
    def __init__(self):
        self.cmd_timeout = rospy.get_param("~cmd_timeout", rospy.get_param("/cmd_timeout", 0.5))
        self.control_rate = rospy.get_param("~control_rate", rospy.get_param("/control_rate", 30.0))
        self.urdf_dir = rospy.get_param("~urdf_dir", "")
        self.urdf_file = rospy.get_param("~urdf_file", "")
        self.robot_description_param = rospy.get_param("~robot_description_param", "/robot_description")

        topics = rospy.get_param("~topics", rospy.get_param("/topics", {}))
        gains = rospy.get_param("~gains", rospy.get_param("/gains", {}))
        limits = rospy.get_param("~limits", rospy.get_param("/limits", {}))

        self.t_cmd_in = topics.get("cmd_vel_in", "/cmd_vel")
        self.t_sim_odom = topics.get("sim_odom", "/sim/odom")
        self.t_real_odom = topics.get("real_odom", "/real/odom")
        self.t_sim_cmd_out = topics.get("sim_cmd_vel_out", "/sim/cmd_vel")
        self.t_real_cmd_out = topics.get("real_cmd_vel_out", "/real/cmd_vel")
        self.t_err_out = topics.get("sync_error_out", "/sync/error")

        self.kp_lin = float(gains.get("kp_lin", 0.8))
        self.kp_yaw = float(gains.get("kp_yaw", 1.2))
        self.max_corr_lin = float(limits.get("max_corr_lin", 0.25))
        self.max_corr_ang = float(limits.get("max_corr_ang", 0.6))

        self.last_cmd = Twist()
        self.last_cmd_time = rospy.Time(0)

        self.sim_odom = None
        self.real_odom = None

        rospy.Subscriber(self.t_cmd_in, Twist, self.cb_cmd, queue_size=10)
        rospy.Subscriber(self.t_sim_odom, Odometry, self.cb_sim_odom, queue_size=20)
        rospy.Subscriber(self.t_real_odom, Odometry, self.cb_real_odom, queue_size=20)

        self.pub_sim_cmd = rospy.Publisher(self.t_sim_cmd_out, Twist, queue_size=10)
        self.pub_real_cmd = rospy.Publisher(self.t_real_cmd_out, Twist, queue_size=10)
        self.pub_err = rospy.Publisher(self.t_err_out, Twist, queue_size=10)

        self.timer = rospy.Timer(rospy.Duration(1.0 / self.control_rate), self.update)

        rospy.loginfo("sim_real_sync_controller started.")
        rospy.loginfo("cmd_in=%s sim_odom=%s real_odom=%s", self.t_cmd_in, self.t_sim_odom, self.t_real_odom)
        rospy.loginfo(
            "urdf_dir=%s urdf_file=%s robot_description_param=%s",
            self.urdf_dir,
            self.urdf_file,
            self.robot_description_param,
        )

    def cb_cmd(self, msg):
        self.last_cmd = msg
        self.last_cmd_time = rospy.Time.now()

    def cb_sim_odom(self, msg):
        self.sim_odom = msg

    def cb_real_odom(self, msg):
        self.real_odom = msg

    def get_safe_cmd(self):
        if (rospy.Time.now() - self.last_cmd_time).to_sec() > self.cmd_timeout:
            return Twist()
        return self.last_cmd

    def update(self, _):
        base_cmd = self.get_safe_cmd()

        # 真实车直接跟随基础命令（避免引入额外风险）
        real_cmd = Twist()
        real_cmd.linear.x = base_cmd.linear.x
        real_cmd.angular.z = base_cmd.angular.z

        # Gazebo 在基础命令上加同步修正
        sim_cmd = Twist()
        sim_cmd.linear.x = base_cmd.linear.x
        sim_cmd.angular.z = base_cmd.angular.z

        err_msg = Twist()

        if self.sim_odom is not None and self.real_odom is not None:
            sx = self.sim_odom.pose.pose.position.x
            sy = self.sim_odom.pose.pose.position.y
            syaw = yaw_from_quat(self.sim_odom.pose.pose.orientation)

            rx = self.real_odom.pose.pose.position.x
            ry = self.real_odom.pose.pose.position.y
            ryaw = yaw_from_quat(self.real_odom.pose.pose.orientation)

            # 误差定义：real - sim
            ex = rx - sx
            ey = ry - sy
            eyaw = norm_angle(ryaw - syaw)

            # 只用前向误差 + 航向误差做简化闭环
            # 把平面误差投影到 sim 朝向
            ex_forward = ex * math.cos(syaw) + ey * math.sin(syaw)

            corr_lin = clamp(self.kp_lin * ex_forward, -self.max_corr_lin, self.max_corr_lin)
            corr_ang = clamp(self.kp_yaw * eyaw, -self.max_corr_ang, self.max_corr_ang)

            sim_cmd.linear.x += corr_lin
            sim_cmd.angular.z += corr_ang

            err_msg.linear.x = ex
            err_msg.linear.y = ey
            err_msg.angular.z = eyaw

        self.pub_real_cmd.publish(real_cmd)
        self.pub_sim_cmd.publish(sim_cmd)
        self.pub_err.publish(err_msg)


if __name__ == "__main__":
    rospy.init_node("sim_real_sync_controller")
    SyncController()
    rospy.spin()
