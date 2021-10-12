/* developed by mirko usuelli
 */
#include "ros-basics-framework/RosNode.h"
#include "ros/ros.h"
#include "std_msgs/Int32.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pulisher");
  
  RosNode node;
   
  node.Prepare();
  
  node.RunPeriodically(node.run_period);
  
  node.Shutdown();
  
  return 0;
}

