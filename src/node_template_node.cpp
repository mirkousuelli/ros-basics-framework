#include "node_template/RosNode.h"

/* ROS topic callbacks */
void inTopic_MessageCallback(const std_msgs::Float64::ConstPtr& msg){
    /*  Read message and store information */
    in_data = msg->data;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, NAME_OF_THIS_NODE);
  
  RosNode node;
   
  node.Prepare();

  /* ROS topics */
  inTopic_subscriber = Handle.subscribe("/in_topic", 1, &node_template::inTopic_MessageCallback, this);
  outTopic_publisher = Handle.advertise<std_msgs::String>("/out_topic", 1);
  
  node.RunPeriodically(node.run_period);
  
  node.Shutdown();
  
  return 0;
}

