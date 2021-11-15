#include "unicycle_dynamics_simulator/unicycle_dynamics_tester.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, NAME_OF_THIS_NODE);
  
  unicycle_dynamics_tester unicycle_dynamics_tester_node;
   
  unicycle_dynamics_tester_node.Prepare();
  
  unicycle_dynamics_tester_node.RunPeriodically();
  
  unicycle_dynamics_tester_node.Shutdown();
  
  return (0);
}

