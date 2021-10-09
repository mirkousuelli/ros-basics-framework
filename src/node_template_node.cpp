#include "node_template/RosNode.h"

/* ROS topic callbacks */
void inTopic_MessageCallback(const std_msgs::Float64::ConstPtr& msg);

int main(int argc, char **argv)
{
  ros::init(argc, argv, NAME_OF_THIS_NODE);
  
  node_template node_template_node;
   
  node_template_node.Prepare();
  
  node_template_node.RunPeriodically(node_template_node.RunPeriod);
  
  node_template_node.Shutdown();
  
  return (0);
}

