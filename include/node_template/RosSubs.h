#ifndef ROS_SUBS_H
#define ROS_SUBS_H

#include "ros/ros.h"
#include <std_msgs/Float64.h>
#include "RosSyncObj.h"
#include <iostream>
#include <vector>
  
using namespace std;

class RosSubs<N> : public RosSyncObj<ros::Subscriber, N>
{
  private: 
    /* ---ATTRIBUTES--------------------------------------------------------------------- */

    /* ---METHODS------------------------------------------------------------------------ */

  public:
    /* ---ATTRIBUTES--------------------------------------------------------------------- */

    /* ---METHODS------------------------------------------------------------------------ */

};

#endif /* ROS_SUBS_H */
