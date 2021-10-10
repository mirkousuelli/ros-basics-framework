/* developed by mirko usuelli
 */
#include "RosNode/RosNode.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, NAME_OF_THIS_NODE);
  
  RosNode node;
   
  node.Prepare();
  
  node.RunPeriodically(node.run_period);
  
  node.Shutdown();
  
  return 0;
}

