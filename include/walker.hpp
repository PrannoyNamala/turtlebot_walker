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
* @file walker.hpp
* @brief a simple walker class for turtlebot
* @details Definition of walker class to generate walker behaviour in turtlebot
* @author Prannoy Namala
* @version 1.0
* @copyright MIT License (c) 2021 Prannoy Namala
*/

#ifndef INCLUDE_WALKER_HPP_
#define INCLUDE_WALKER_HPP_
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"


/**
* @brief Walker class
*/

class Walker {
  private:
    // Check for collision
    bool collisionFlag;
    // Velocity
    geometry_msgs::Twist velocity;
    // Nodehandle
    ros::NodeHandle n;
    // Velocity Publisher
    ros::Publisher velPub;
    // Laser Data Subscriber
    ros::Subscriber laserSub;

  public:
    /**
    * @brief Constructor for Walker
    */
    Walker();

    /**
    * @brief Destructor for Walker
    */
    ~Walker();

    /**
    * @brief Method to check the distace of turtlebot from obstacle
    * @param[in] msg laser scan message
    * @return None
    */
    void checkObstacle(const sensor_msgs::LaserScan::ConstPtr& msg);

    /**
    * @brief Method to run the turtlebot with walker behaviour
    * @param[in] None
    * @return None
    */
    void walk();
};

#endif  // INCLUDE_WALKER_HPP_