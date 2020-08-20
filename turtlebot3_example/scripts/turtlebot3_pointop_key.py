#!/usr/bin/env python
#################################################################################
# Copyright 2018 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#################################################################################

# Authors: Gilbert #

import rospy
from geometry_msgs.msg import Twist, Point, Quaternion
import tf
from math import radians, copysign, sqrt, pow, pi, atan2
from tf.transformations import euler_from_quaternion
import numpy as np

msg = """
control your Turtlebot3!
-----------------------
Insert xyz - coordinate.
x : position x (m)
y : position y (m)
z : orientation z (degree: -180 ~ 180)
q : quit
-----------------------
"""
        
def shutdown():
    rospy.Publisher('cmd_vel', Twist, queue_size=1).publish(Twist())
    rospy.sleep(1)


def getkey():
    return_value = ""
    in_string = input("| x | y | z |\n")
    if in_string[0] == 'q':
        shutdown()
    else:
        try:
            x, y, z = in_string.split()
            return_value = [float(x), float(y), float(z)]
        except ValueError:
            shutdown()
    return return_value


def get_odom(tf_listener, odom_frame, base_frame):
    try:
        (trans, rot) = tf_listener.lookupTransform(odom_frame, base_frame, rospy.Time(0))
        rotation = euler_from_quaternion(rot)

    except (tf.Exception, tf.ConnectivityException, tf.LookupException):
        rospy.loginfo("TF Exception")
        return

    return (Point(*trans), rotation[2])


def main():
    rospy.init_node('turtlebot3_pointop_key', anonymous=False)
    rospy.on_shutdown(shutdown)
    cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)
    position = Point()
    move_cmd = Twist()
    r = rospy.Rate(10)
    tf_listener = tf.TransformListener()
    odom_frame = 'odom'

    try:
        tf_listener.waitForTransform(odom_frame, 'base_footprint', rospy.Time(), rospy.Duration(1.0))
        base_frame = 'base_footprint'
    except (tf.Exception, tf.ConnectivityException, tf.LookupException):
        try:
            tf_listener.waitForTransform(odom_frame, 'base_link', rospy.Time(), rospy.Duration(1.0))
            base_frame = 'base_link'
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("Cannot find transform between odom and base_link or base_footprint")
            rospy.signal_shutdown("tf Exception")

    while not rospy.is_shutdown():
        print(msg)
        (position, rotation) = get_odom(tf_listener, odom_frame, base_frame)
        last_rotation = 0
        linear_speed = 1
        angular_speed = 1
        (goal_x, goal_y, goal_z) = getkey()
        if goal_z > 180 or goal_z < -180:
            print("you input wrong z range.")
            shutdown()
        goal_z = np.deg2rad(goal_z)
        goal_distance = sqrt(pow(goal_x - position.x, 2) + pow(goal_y - position.y, 2))
        distance = goal_distance

        while distance > 0.05:
            (position, rotation) = get_odom(tf_listener, odom_frame, base_frame)
            x_start = position.x
            y_start = position.y
            path_angle = atan2(goal_y - y_start, goal_x- x_start)

            if path_angle < -pi/4 or path_angle > pi/4:
                if goal_y < 0 and y_start < goal_y:
                    path_angle = -2*pi + path_angle
                elif goal_y >= 0 and y_start > goal_y:
                    path_angle = 2*pi + path_angle
            if last_rotation > pi-0.1 and rotation <= 0:
                rotation = 2*pi + rotation
            elif last_rotation < -pi+0.1 and rotation > 0:
                rotation = -2*pi + rotation
            move_cmd.angular.z = angular_speed * path_angle-rotation

            distance = sqrt(pow((goal_x - x_start), 2) + pow((goal_y - y_start), 2))
            move_cmd.linear.x = min(linear_speed * distance, 0.1)

            if move_cmd.angular.z > 0:
                move_cmd.angular.z = min(move_cmd.angular.z, 1.5)
            else:
                move_cmd.angular.z = max(move_cmd.angular.z, -1.5)

            last_rotation = rotation
            cmd_vel_pub.publish(move_cmd)
            r.sleep()
        (position, rotation) = get_odom(tf_listener, odom_frame, base_frame)

        while abs(rotation - goal_z) > 0.05:
            (position, rotation) = get_odom(tf_listener, odom_frame, base_frame)
            if goal_z >= 0:
                if rotation <= goal_z and rotation >= goal_z - pi:
                    move_cmd.linear.x = 0.00
                    move_cmd.angular.z = 0.5
                else:
                    move_cmd.linear.x = 0.00
                    move_cmd.angular.z = -0.5
            else:
                if rotation <= goal_z + pi and rotation > goal_z:
                    move_cmd.linear.x = 0.00
                    move_cmd.angular.z = -0.5
                else:
                    move_cmd.linear.x = 0.00
                    move_cmd.angular.z = 0.5
            cmd_vel_pub.publish(move_cmd)
            r.sleep()


        rospy.loginfo("Stopping the robot...")
        cmd_vel_pub.publish(Twist())


if __name__ == '__main__':
    try:
        main()
    except:
        rospy.loginfo("shutdown program.")

