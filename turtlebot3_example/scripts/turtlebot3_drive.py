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

# Adapted from cpp code from turtlebot3_gazebo

import rospy
import math
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion


#################################################
### Public Variables
#################################################


CENTER = 0
LEFT = 1
RIGHT = 2

LINEAR_VELOCITY = 0.3
ANGULAR_VELOCITY = 1.5

GET_TB3_DIRECTION = 0
TB3_DRIVE_FORWARD = 1
TB3_RIGHT_TURN = 2
TB3_LEFT_TURN = 3


#################################################
### Auxiliary functions
#################################################


def odom_msg_callback(msg, tb3_orientation):
    quaternion = msg.pose.pose.orientation
    quatenion_list = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
    
    for i in range(len(tb3_orientation)):
        tb3_orientation[i] = euler_from_quaternion(quatenion_list)[i]



def laser_scan_msg_callback(msg, scan_data):
    scan_angle = [0, 30, 330]

    for i in range(len(scan_angle)):
        if (math.isinf(msg.ranges[scan_angle[i]])):
            scan_data[i] = msg.range_max
        else:
            scan_data[i] = msg.ranges[scan_angle[i]]



def update_command_velocity(linear, angular, cmd_vel_pub):
    cmd_vel = Twist()

    cmd_vel.linear.x  = linear
    cmd_vel.angular.z = angular

    cmd_vel_pub.publish(cmd_vel)



def control_loop(scan_data, tb3_orientation, cmd_vel_pub):
    escape_range = math.radians(30.0)
    check_forward_dist = 0.7
    check_side_dist = 0.6
    prev_tb3_orientation_z = 0.0

    turtlebot3_state = GET_TB3_DIRECTION

    while not rospy.is_shutdown():
        if (turtlebot3_state == GET_TB3_DIRECTION):
            if (scan_data[CENTER] > check_forward_dist):
                if (scan_data[LEFT] < check_side_dist):
                    prev_tb3_orientation_z = tb3_orientation[2]
                    turtlebot3_state = TB3_RIGHT_TURN

                elif (scan_data[RIGHT] < check_side_dist):
                    prev_tb3_orientation_z = tb3_orientation[2]
                    turtlebot3_state = TB3_LEFT_TURN

                else:
                    turtlebot3_state = TB3_DRIVE_FORWARD

            else:
                prev_tb3_orientation_z = tb3_orientation[2]
                turtlebot3_state = TB3_LEFT_TURN

        elif (turtlebot3_state == TB3_DRIVE_FORWARD):
            update_command_velocity(LINEAR_VELOCITY, 0.0, cmd_vel_pub)
            turtlebot3_state = GET_TB3_DIRECTION

        elif (turtlebot3_state == TB3_RIGHT_TURN):    
            if (math.fabs(prev_tb3_orientation_z - tb3_orientation[2]) >= escape_range):
                turtlebot3_state = GET_TB3_DIRECTION
            else:
                update_command_velocity(0.0, -1 * ANGULAR_VELOCITY, cmd_vel_pub)

        elif (turtlebot3_state == TB3_LEFT_TURN):
            if (math.fabs(prev_tb3_orientation_z - tb3_orientation[2]) >= escape_range):
                turtlebot3_state = GET_TB3_DIRECTION
            else:
                update_command_velocity(0.0, ANGULAR_VELOCITY, cmd_vel_pub)
        
        else:
            turtlebot3_state = GET_TB3_DIRECTION


#################################################
### Main Function
#################################################


def main():
    rospy.init_node('turtlebot3_drive')
    print("TurtleBot3 Simulation Node Init")

    scan_data = [.0, .0, .0]
    tb3_orientation = [.0, .0, .0]

    cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
    
    rospy.Subscriber("scan", LaserScan, laser_scan_msg_callback, scan_data, 10) 
    rospy.Subscriber("odom", Odometry, odom_msg_callback, tb3_orientation, 10)

    control_loop(scan_data, tb3_orientation, cmd_vel_pub)


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("shutdown program.")
