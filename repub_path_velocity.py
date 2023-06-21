import rospy
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion
import time
import csv
import math
import numpy as np


class OdomPublisher(object):
    def __init__(self):
        rospy.init_node('odom_publisher')
        self.subscription = rospy.Subscriber(
            '/vrpn_client_node/P1/pose',
            PoseStamped,
            self.odom_callback,
            queue_size=10
        )

        self.publisher = rospy.Publisher(
            'robot_pose',
            PoseStamped,
            queue_size=10
        )

        self.timer = rospy.Timer(rospy.Duration(1/30), self.loop)

        # Create a CSV file to store the data
        self.csv_file = open('path_data_low_pass.csv', 'w')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['Time', 'X', 'Y', 'W', 'Xd', 'Yd', 'Wd'])

        self.count = 0
        self.prev_pose = None
        self.prev_time = None
        self.start_time = None
        self.callback_time = None
        self.alpha = 0.3

    def calculate_euler_diff(self, current_orientation, previous_orientation):
        # Convert the quaternion objects to lists
        current_quaternion = [current_orientation.x, current_orientation.y, current_orientation.z, current_orientation.w]
        previous_quaternion = [previous_orientation.x, previous_orientation.y, previous_orientation.z, previous_orientation.w]

        # Calculate the Euler angle differences
        current_euler = euler_from_quaternion(current_quaternion)
        previous_euler = euler_from_quaternion(previous_quaternion)
        euler_diff = [
            self.normalize_angle(current_euler[0] - previous_euler[0]),
            self.normalize_angle(current_euler[1] - previous_euler[1]),
            self.normalize_angle(current_euler[2] - previous_euler[2])
        ]

        return euler_diff

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def loop(self, event):
        current_time = time.time()

        # If the time of the callback is essentially the same as the control loop time
        if self.callback_time is None:
            return

        if np.round((self.callback_time - current_time), 2) != 0.0:
            # print(np.round((self.callback_time - current_time), 2))
            # return
            pass

        if self.prev_pose is not None and self.prev_time is not None:
            if self.prev_pose == self.pose:
                return

            time_diff = current_time - self.prev_time

            # Calculate linear velocity
            xd = (self.x - self.prev_pose.position.x) / time_diff
            yd = (self.y - self.prev_pose.position.y) / time_diff

            # Low-pass filter the velocity data
            self.filtered_xd = self.alpha * xd + (1 - self.alpha) * self.filtered_xd
            self.filtered_yd = self.alpha * yd + (1 - self.alpha) * self.filtered_yd

            # Convert Euler angles (roll, pitch, yaw) to angular velocities
            euler_diff = self.calculate_euler_diff(self.orientation, self.prev_pose.orientation)

            # Calculate angular velocity
            wd = euler_diff[2] / time_diff

            # Low-pass filter the angular velocity data
            self.filtered_wd = self.alpha * wd + (1 - self.alpha) * self.filtered_wd

        else:
            self.filtered_xd = 0.0
            self.filtered_yd = 0.0
            self.filtered_wd = 0.0

        self.prev_pose = self.pose
        self.prev_time = current_time

        # Write the data to the CSV file
        self.csv_writer.writerow([self.elapsed_time, self.x, self.y, self.w, self.filtered_xd, self.filtered_yd, self.filtered_wd])

    def odom_callback(self, msg):
        # Get the current timestamp
        current_time = rospy.Time.now()

        self.callback_time = time.time()

        if self.start_time is None:
            self.start_time = current_time

        # Calculate elapsed time
        self.elapsed_time = (current_time - self.start_time).to_sec()

        # Process the pose data
        pose = msg.pose
        self.pose = pose

        # Process the pose data
        self.x = pose.position.x
        self.y = pose.position.y

        orientation = pose.orientation
        self.orientation = orientation
        _, _, self.w = euler_from_quaternion(
            [orientation.x, orientation.y, orientation.z, orientation.w]
        )
        self.publisher.publish(msg)


def main():
    odom_publisher = OdomPublisher()
    rospy.spin()
    odom_publisher.csv_file.close()


if __name__ == '__main__':
    main()
