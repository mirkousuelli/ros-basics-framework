/* developed by mirko usuelli
 */
#ifndef ROS_SUBS_H
#define ROS_SUBS_H

#include "RosNode/RosSyncObj.h"

/* extenting the class Ros Synchronous Objects 
 * in order to maganege a list of subscribers 
 */
class RosSubs : public RosSyncObj<ros::Subscriber> {};

#endif /* ROS_SUBS_H */
