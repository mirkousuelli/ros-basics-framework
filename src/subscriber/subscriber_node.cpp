/* developed by mirko usuelli
 */
#include "ros-basics-framework/subscriber_class.h"
#include "ros/ros.h"
#include "std_msgs/Int32.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "subscriber");
  
  subscriber_class node;
   
  node.Prepare();
  
  node.RunPeriodically(node.run_period);
  
  node.Shutdown();
  
  return 0;
}

