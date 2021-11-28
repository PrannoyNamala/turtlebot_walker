/**
 * MIT License
 * Copyright (c) 2021 Prannoy Namala
 *
 *  Permission is hereby granted, free of charge, to any person obtaining a
 *  copy of this software and associated documentation files (the "Software"),
 *  to deal in the Software without restriction, including without
 *  limitation the rights to use, copy, modify, merge, publish, distribute,
 *  sublicense, and/or sell copies of the Software, and to permit persons to
 *  whom the Software is furnished to do so, subject to the following
 *  conditions:
 *
 *  The above copyright notice and this permission notice shall be included
 *  in all copies or substantial portions of the Software.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 *  THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 *  FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 *  DEALINGS IN THE SOFTWARE.N CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 SOFTWARE.
 */

/**
* @file walker.cpp
* @brief a simple walker class for turtlebot
* @details Implementation of walker class to generate walker behaviour in turtlebot
* @author Prannoy Namala
* @version 1.0
* @copyright MIT License (c) 2021 Prannoy Namala
*/

// cppcheck-suppress missingInclude
#include <array>
// cppcheck-suppress missingInclude
#include "ros/ros.h"
// cppcheck-suppress missingInclude
#include "sensor_msgs/LaserScan.h"
// cppcheck-suppress missingInclude
#include "geometry_msgs/Twist.h"
// cppcheck-suppress missingInclude
#include "walker.hpp"



Walker::Walker() {
    // Publish velocity
    velPub = n.advertise <geometry_msgs::Twist> ("/cmd_vel",
    1000);

    // Subscibe to laser scan message
    laserSub = n.subscribe <sensor_msgs::LaserScan> ("/scan", 500,
    &Walker::checkObstacle, this);

    // Set collision flag to false
    collisionFlag = false;
}

Walker::~Walker() {
    // Reset the velocity

    velocity.linear.x = 0;
    velocity.linear.z = 0;

    // Publish the velocity

    velPub.publish(velocity);
}

void Walker::checkObstacle(const sensor_msgs::LaserScan::ConstPtr& msg) {
    // Iterate over range values
    collisionFlag = false;
    std::array<int, 15> indices = {0, 1, 2, 3, 4, 5, 6, 7, 17,
        18, 19, 20, 21, 22, 23};
    for (auto i : indices) {
     if ( msg->ranges[i] < 0.8 ) {
        collisionFlag = true;
      }
    }
}

void  Walker::walk() {
    ros::Rate loop_rate(10);
    while (ros::ok()) {
        // Check if collison flag is true
        if (collisionFlag) {
            ROS_INFO("Obstacle Detected");
        // Set velocity if collision flag is true
            velocity.linear.x = 0.0;
            velocity.angular.z = 0.4;
        } else {
      // Set velocity if collision flag is false.
            ROS_INFO("Moving Forward");
            velocity.angular.z = 0.0;
            velocity.linear.x = 0.4;
        }
        velPub.publish(velocity);
        ros::spinOnce();
        loop_rate.sleep();
    }
}


