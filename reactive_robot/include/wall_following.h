#ifndef STDR_WALL_FOLLOWING
#define STDR_WALL_FOLLOWING

#include <iostream>
#include <cstdlib>
#include <cmath>

#include <ros/package.h>
#include "ros/ros.h"

#include <stdr_msgs/RobotIndexedVectorMsg.h>

#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Range.h>

/**
@namespace stdr_reactive
@brief The main namespace for STDR reactive robot
**/ 
namespace stdr_reactive
{
  /**
  @class WallFollowing
  @brief Performs obstacle avoidance to a single robot
  **/ 
  class WallFollowing
  {
    private:
      
      //!< The ros laser scan msg
      sensor_msgs::LaserScan scan_;
      
      //!< Subscriber for the ros laser msg
      ros::Subscriber subscriber_;
      
      //!< The ROS node handle
      ros::NodeHandle n_;
      
      //!< The laser topic
      std::string laser_topic_;
      
      //!< The speeds topic
      std::string speeds_topic_;
      
      //!< The twist publisher
      ros::Publisher cmd_vel_pub_;
      
    public:
    
      /**
      @brief Default contructor
      @param argc [int] Number of input arguments
      @param argv [char **] Input arguments
      @return void
      **/
      WallFollowing(int argc,char **argv);
      
      /**
      @brief Default destructor
      @return void
      **/
      ~WallFollowing(void);
      
      /**
      @brief Callback for the ros laser message
      @param msg [const sensor_msgs::LaserScan&] The new laser scan message
      @return void
      **/
      void callback(const sensor_msgs::LaserScan& msg);
      
  };
}

#endif
