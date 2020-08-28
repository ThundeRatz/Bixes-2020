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
# Modified by Lucas #

import rospy
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from math import radians, sqrt, pow, pi, atan2
from tf.transformations import euler_from_quaternion


#################################################
### Public Variables
#################################################


MAX_LINEAR_VELOCITY = 0.22
MAX_ANGULAR_VELOCITY = 2.84

control_msg = """
control your Turtlebot3!
-----------------------
Insert xyz - coordinate.
x : position x (m)
y : position y (m)
z : orientation z (degree: -180 ~ 180)
q : quit
-----------------------
"""

#################################################
### Auxiliary functions
#################################################

        
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


def distance_between_points(point_a, point_b):
    distance = sqrt(pow(point_a.x - point_b.x, 2) + pow(point_a.y - point_b.y, 2))

    return distance


def odom_msg_callback(msg, tb3_pose):
    quaternion = msg.pose.pose.orientation
    position = msg.pose.pose.position

    quatenion_list = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
    tb3_orientation = euler_from_quaternion(quatenion_list)

    tb3_pose["point"] = position
    tb3_pose["rotation"] = tb3_orientation[2]


#################################################
### Main Function
#################################################


def main():
    rospy.init_node('turtlebot3_pointop_key', anonymous=False)
    rospy.on_shutdown(shutdown)

    tb3_pose = {"point": Point(0, 0, 0), "rotation": .0}

    cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)
    rospy.Subscriber("odom", Odometry, odom_msg_callback, tb3_pose, 10)

    move_cmd = Twist()

    loop_rate = rospy.Rate(10)

    print("Wainting for first message...")
    rospy.wait_for_message('odom', Odometry)

    while not rospy.is_shutdown():
        print(control_msg)

        last_rotation = 0
        linear_speed = 1
        angular_speed = 1

        (goal_x, goal_y, goal_z) = getkey()

        if goal_z > 180 or goal_z < -180:
            print("you input wrong z range.")
            shutdown()

        goal_z = radians(goal_z)
        goal_distance = distance_between_points(Point(goal_x, goal_y, 0), tb3_pose["point"])

        while goal_distance > 0.05:
            rotation = tb3_pose["rotation"]

            x_start = tb3_pose["point"].x
            y_start = tb3_pose["point"].y
            path_angle = atan2(goal_y - y_start, goal_x - x_start)

            if path_angle < -pi / 4 or path_angle > pi / 4:
                if goal_y < 0 and y_start < goal_y:
                    path_angle = -2 * pi + path_angle
                elif goal_y >= 0 and y_start > goal_y:
                    path_angle = 2 * pi + path_angle

            if last_rotation > pi - 0.1 and rotation <= 0:
                rotation = 2 * pi + rotation
            elif last_rotation < -pi + 0.1 and rotation > 0:
                rotation = -2 * pi + rotation
            
            move_cmd.angular.z = angular_speed * path_angle-rotation

            if move_cmd.angular.z > 0:
                move_cmd.angular.z = min(move_cmd.angular.z, MAX_ANGULAR_VELOCITY)
            else:
                move_cmd.angular.z = max(move_cmd.angular.z, -MAX_ANGULAR_VELOCITY)

            goal_distance = distance_between_points(Point(goal_x, goal_y, 0), Point(x_start, y_start, 0))
            linear_cmd = min(linear_speed * goal_distance, MAX_LINEAR_VELOCITY) # don't go to fast in the wrong direction
            move_cmd.linear.x = max(linear_cmd - abs(move_cmd.angular.z) / 5, 0)

            last_rotation = rotation
            cmd_vel_pub.publish(move_cmd)
            loop_rate.sleep()
        
        rotation = tb3_pose["rotation"]

        while abs(rotation - goal_z) > 0.05:
            rotation = tb3_pose["rotation"]
        
            move_cmd.linear.x = 0.00

            angular_velocity = MAX_ANGULAR_VELOCITY / 3

            if goal_z >= 0:
                if rotation <= goal_z and rotation >= goal_z - pi:
                    move_cmd.angular.z = angular_velocity
                else:
                    move_cmd.angular.z = -angular_velocity
            else:
                if rotation <= goal_z + pi and rotation > goal_z:
                    move_cmd.angular.z = -angular_velocity
                else:
                    move_cmd.angular.z = angular_velocity
            
            cmd_vel_pub.publish(move_cmd)
            loop_rate.sleep()

        rospy.loginfo("Stopping the robot...")
        cmd_vel_pub.publish(Twist())


if __name__ == '__main__':
    try:
        main()
    except:
        rospy.loginfo("shutdown program.")

