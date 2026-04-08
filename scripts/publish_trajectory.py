#!/usr/bin/env python3

import rospy
import numpy as np

from trajectory_msgs.msg import JointTrajectoryPoint
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped


class CSVTrajectoryPublisher:

    def __init__(self):
        rospy.init_node("csv_trajectory_publisher")

        self.csv_path = rospy.get_param("~csv_path")
        self.dt = 0.001  # 1 kHz timer

        # Load data
        self.load_csv()

        self.current_pos = None
        self.odom_received = False

        self.index = 0
        self.shift = None
        self.start_time = None

        self.msg = JointTrajectoryPoint()

        self.pub = rospy.Publisher(
            "/cmd_full_state", JointTrajectoryPoint, queue_size=10
        )

        # LATCHED publisher → RViz gets it even if started later
        self.path_pub = rospy.Publisher(
            "/trajectory_path", Path, queue_size=1, latch=True
        )

        self.odom_sub = rospy.Subscriber("/odometry", Odometry, self.odom_callback)

        # Start high-rate timer
        self.timer = rospy.Timer(rospy.Duration(self.dt), self.timer_callback)

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
            self.publish_path()

    # ----------------------------------------------------
    def load_csv(self):
        rospy.loginfo(f"Loading CSV: {self.csv_path}")

        data = np.genfromtxt(self.csv_path, delimiter=",", names=True)

        self.t = data["t"]

        self.p = np.vstack((data["p_x"], data["p_y"], data["p_z"])).T

        self.v = np.vstack((data["v_x"], data["v_y"], data["v_z"])).T

        self.a = np.vstack((data["a_lin_x"], data["a_lin_y"], data["a_lin_z"])).T

        self.j = np.vstack((data["jerk_x"], data["jerk_y"], data["jerk_z"])).T

        rospy.loginfo(f"Loaded {len(self.t)} trajectory points.")

    # ----------------------------------------------------
    def publish_path(self):
        path = Path()
        path.header.frame_id = "odom"
        path.header.stamp = rospy.Time.now()

        self.shift = self.current_pos - self.p[0, :]
        rospy.loginfo(f"Applying shift: {self.shift}")

        for i in range(len(self.p)):
            pose = PoseStamped()
            pose.header.frame_id = "odom"
            pose.header.stamp = path.header.stamp
            pose.pose.position.x = self.p[i, 0] + self.shift[0]
            pose.pose.position.y = self.p[i, 1] + self.shift[1]
            pose.pose.position.z = self.p[i, 2] + self.shift[2]
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)

        self.path_pub.publish(path)
        rospy.loginfo("Trajectory path published for RViz.")

    # ----------------------------------------------------
    def timer_callback(self, event):

        if not self.odom_received:
            return

        if self.index >= len(self.t):
            return

        now = rospy.Time.now()

        if self.start_time is None:
            self.start_time = now

        elapsed = (now - self.start_time).to_sec()

        # Advance index based on elapsed time
        while self.index < len(self.t) - 1 and elapsed >= self.t[self.index] + 1:
            self.index += 1

        if self.shift is None:
            # self.shift = self.current_pos - self.p[0]
            return

        pos = self.p[self.index] + self.shift
        vel = self.v[self.index]
        acc = self.a[self.index]
        jerk = self.j[self.index]

        self.msg.positions = pos.tolist()
        self.msg.velocities = vel.tolist()
        self.msg.accelerations = acc.tolist()
        self.msg.effort = jerk.tolist()
        self.msg.time_from_start = rospy.Duration(self.t[self.index])

        self.pub.publish(self.msg)


if __name__ == "__main__":
    try:
        CSVTrajectoryPublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
