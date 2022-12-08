#!/usr/bin/env python3

import os
import csv
import math
import numpy as np
from numpy import linalg as la

import rospy
from ackermann_msgs.msg import AckermannDrive
from geometry_msgs.msg import Twist, Vector3
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion, quaternion_from_euler

from gazebo_msgs.srv import GetModelState
from gazebo_msgs.msg import ModelState

from pid import PID


class WaypointPIDControl(object):
    def __init__(self):

        self.rate = rospy.Rate(20)

        self.look_ahead = 6

        self.ackermann_msg = AckermannDrive()
        self.ackermann_msg.steering_angle_velocity = 0.0
        self.ackermann_msg.acceleration = 0.0
        self.ackermann_msg.jerk = 0.0
        self.ackermann_msg.speed = 0.0
        self.ackermann_msg.steering_angle = 0.0

        self.ackermann_pub = rospy.Publisher(
            "ackermann_cmd", AckermannDrive, queue_size=1
        )

        self.read_waypoints()

        ################################################################### TODO: set your own weights for P, I, D terms
        self.pid = PID(
            Kp=0.8,
            Ki=0.4,
            Kd=0.04,
            set_point=0.0,
            sample_time=0.01,
            out_limits=(-0.61, 0.61),
        )

    def read_waypoints(self):
        dirname = os.path.dirname(__file__)
        filename = os.path.join(dirname, "../waypoints/wps.csv")

        with open(filename) as f:
            path_points = [tuple(line) for line in csv.reader(f)]

        # turn path_points into a list of floats to eliminate the need for casts
        self.path_points_x = [float(point[0]) for point in path_points]
        self.path_points_y = [float(point[1]) for point in path_points]
        self.dist_arr = np.zeros(len(self.path_points_x))

    def dist(self, p1, p2):
        return round(np.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2), 3)

    def get_gem_pose(self):

        rospy.wait_for_service("/gazebo/get_model_state")

        try:
            service_response = rospy.ServiceProxy(
                "/gazebo/get_model_state", GetModelState
            )
            model_state = service_response(model_name="gem")
        except rospy.ServiceException as exc:
            rospy.loginfo("Service did not process request: " + str(exc))

        x = model_state.pose.position.x
        y = model_state.pose.position.y

        orientation_q = model_state.pose.orientation
        orientation_list = [
            orientation_q.x,
            orientation_q.y,
            orientation_q.z,
            orientation_q.w,
        ]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)

        return round(x, 4), round(y, 4), round(yaw, 4)

    def find_angle(self, v1, v2):
        cosang = np.dot(v1, v2)
        sinang = la.norm(np.cross(v1, v2))
        # [-pi, pi]
        return np.arctan2(sinang, cosang)

    def start_drive(self):

        while not rospy.is_shutdown():

            # get current position and orientation in the world frame
            cur_x, cur_y, cur_yaw = self.get_gem_pose()

            self.path_points_x = np.array(self.path_points_x)
            self.path_points_y = np.array(self.path_points_y)

            # finding the distance of each way point from the current position
            for i in range(len(self.path_points_x)):
                self.dist_arr[i] = self.dist(
                    (self.path_points_x[i], self.path_points_y[i]), (cur_x, cur_y)
                )

            # finding those points which are less than the look ahead distance (will be behind and ahead of the vehicle)
            goal_arr = np.where(
                (self.dist_arr < self.look_ahead + 0.3)
                & (self.dist_arr > self.look_ahead - 0.3)
            )[0]

            # finding the goal point which is the last in the set of points less than the lookahead distance
            for idx in goal_arr:
                v1 = [
                    self.path_points_x[idx] - cur_x,
                    self.path_points_y[idx] - cur_y,
                ]
                v2 = [np.cos(cur_yaw), np.sin(cur_yaw)]
                temp_angle = self.find_angle(v1, v2)
                if abs(temp_angle) < np.pi / 2:
                    self.goal = idx
                    break
            
            # TODO: transforming the goal point into the vehicle coordinate frame
            vcfx = self.path_points_x[self.goal] - cur_x
            vcfy = self.path_points_y[self.goal] - cur_y
            coordx = vcfx*np.cos(cur_yaw) + vcfy*np.sin(cur_yaw)
            coordy = vcfy*np.cos(cur_yaw) - vcfx*np.sin(cur_yaw)
            
            # TODO: define your feedback value
            currentAngle=math.atan(coordy/ coordx)
            currentFeedback = currentAngle
            self.pid(currentFeedback)
            angle =-self.pid.output
            print("angle: %f", angle)
            self.ackermann_msg.speed = 2.8
            self.ackermann_msg.steering_angle = float(angle)
            self.ackermann_pub.publish(self.ackermann_msg)

            self.rate.sleep()


def main():

    rospy.init_node("pure_pursuit_sim_node", anonymous=True)
    control = WaypointPIDControl()

    try:
        control.start_drive()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
