#include "unicycle_dynamics_simulator/unicycle_dynamics_simulator.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, NAME_OF_THIS_NODE);
  
  unicycle_dynamics_simulator unicycle_dynamics_simulator_node;
   
  unicycle_dynamics_simulator_node.Prepare();
  
  unicycle_dynamics_simulator_node.RunPeriodically();
  
  unicycle_dynamics_simulator_node.Shutdown();
  
  return (0);
}

