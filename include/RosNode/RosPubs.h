/* developed by mirko usuelli
 */
#ifndef ROS_PUBS_H
#define ROS_PUBS_H

#include "RosNode/RosSyncObj.h"

/* extenting the class Ros Synchronous Objects 
 * in order to maganege a list of publishers 
 */
class RosPubs : public RosSyncObj<ros::Publisher> {};

#endif /* ROS_PUBS_H */