#ifndef ROS_PUBS_H
#define ROS_PUBS_H

#include "ros/ros.h"
#include <std_msgs/Float64.h>
#include "RosSyncObj.h"
#include <iostream>
#include <vector>
  
using namespace std;

class RosSubs<N> : public RosSyncObj<ros::Publisher, N>
{
  private: 
    /* ---ATTRIBUTES--------------------------------------------------------------------- */

    /* ---METHODS------------------------------------------------------------------------ */

  public:
    /* ---ATTRIBUTES--------------------------------------------------------------------- */

    /* ---METHODS------------------------------------------------------------------------ */

};

#endif /* ROS_PUBS_H */