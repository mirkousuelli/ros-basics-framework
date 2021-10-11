/* developed by mirko usuelli
 */
#include "ros-basics-framework/RosNode.h"
#include <std_msgs/Int32.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "subscriber");
  
  RosNode<std_msgs::Int32> node;
   
  node.Prepare();
  
  node.RunPeriodically(node.run_period);
  
  node.Shutdown();
  
  return 0;
}

