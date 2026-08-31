#!/usr/bin/env python3

import rospy
import numpy as np

from trajectory_msgs.msg import (
    MultiDOFJointTrajectory,
    MultiDOFJointTrajectoryPoint,
)
from geometry_msgs.msg import Transform, Twist
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped


class CSVTrajectoryPublisher:

    def __init__(self):
        rospy.init_node("csv_trajectory_publisher")

        self.csv_path = rospy.get_param("~csv_path")

        self.load_csv()

        self.current_pos = None
        self.odom_received = False
        self.published = False

        self.pub = rospy.Publisher(
            "/cmd_trajectory", MultiDOFJointTrajectory, queue_size=1, latch=True
        )

        self.path_pub = rospy.Publisher(
            "/trajectory_path", Path, queue_size=1, latch=True
        )

        self.odom_sub = rospy.Subscriber("/odometry", Odometry, self.odom_callback)

        rospy.loginfo("CSV Trajectory Publisher ready.")

    # ----------------------------------------------------
    def odom_callback(self, msg):
        self.current_pos = np.array(
            [
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                msg.pose.pose.position.z,
            ]
        )

        if not self.odom_received:
            self.odom_received = True
            self.publish_trajectory()

    # ----------------------------------------------------
    def load_csv(self):
        rospy.loginfo(f"Loading CSV: {self.csv_path}")

        data = np.genfromtxt(self.csv_path, delimiter=",", names=True)

        self.t = data["t"]
        self.p = np.vstack((data["p_x"], data["p_y"], data["p_z"])).T
        self.v = np.vstack((data["v_x"], data["v_y"], data["v_z"])).T
        self.a = np.vstack((data["a_lin_x"], data["a_lin_y"], data["a_lin_z"])).T

        rospy.loginfo(f"Loaded {len(self.t)} trajectory points.")

    # ----------------------------------------------------
    def publish_trajectory(self):
        if self.published:
            return

        shift = self.current_pos - self.p[0, :]
        rospy.loginfo(f"Applying shift: {shift}")

        msg = MultiDOFJointTrajectory()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "odom"
        msg.joint_names = ["base_link"]

        path = Path()
        path.header.frame_id = "odom"
        path.header.stamp = rospy.Time.now()

        for i in range(len(self.t)):
            pt = MultiDOFJointTrajectoryPoint()

            tf = Transform()
            tf.translation.x = self.p[i, 0] + shift[0]
            tf.translation.y = self.p[i, 1] + shift[1]
            tf.translation.z = self.p[i, 2] + shift[2]
            tf.rotation.w = 1.0
            pt.transforms.append(tf)

            vel = Twist()
            vel.linear.x = self.v[i, 0]
            vel.linear.y = self.v[i, 1]
            vel.linear.z = self.v[i, 2]
            pt.velocities.append(vel)

            acc = Twist()
            acc.linear.x = self.a[i, 0]
            acc.linear.y = self.a[i, 1]
            acc.linear.z = self.a[i, 2]
            pt.accelerations.append(acc)

            pt.time_from_start = rospy.Duration(self.t[i])

            msg.points.append(pt)

            pose = PoseStamped()
            pose.header.frame_id = "odom"
            pose.header.stamp = path.header.stamp
            pose.pose.position.x = self.p[i, 0] + shift[0]
            pose.pose.position.y = self.p[i, 1] + shift[1]
            pose.pose.position.z = self.p[i, 2] + shift[2]
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)

        self.pub.publish(msg)
        self.path_pub.publish(path)
        self.published = True
        rospy.loginfo("Trajectory published.")


if __name__ == "__main__":
    try:
        CSVTrajectoryPublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
