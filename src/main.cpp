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
* @file main.cpp
* @brief Creating an exectutible for ROS
* @details Definition of walker class to generate walker behaviour in turtlebot
* @author Prannoy Namala
* @version 1.0
* @copyright MIT License (c) 2021 Prannoy Namala
*/

// cppcheck-suppress missingInclude
#include "ros/ros.h"
// cppcheck-suppress missingInclude
#include "sensor_msgs/LaserScan.h"
// cppcheck-suppress missingInclude
#include "geometry_msgs/Twist.h"
// cppcheck-suppress missingInclude
#include <walker.hpp>

int main(int argc, char* argv[]) {
  // Node Initialization

  ros::init(argc, argv, "Walker");
  Walker turtlebot_walker;
  turtlebot_walker.walk();
  return 0;
}
