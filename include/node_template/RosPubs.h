/* developed by mirko usuelli
 */
#ifndef ROS_PUBS_H
#define ROS_PUBS_H

#include "node_template/RosSyncObj.h"
  
using namespace std;

class RosPubs : public RosSyncObj<ros::Publisher> {};

#endif /* ROS_PUBS_H */