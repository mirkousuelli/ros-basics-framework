#include "unicycle_kinematics_simulator/unicycle_kinematics_simulator.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, NAME_OF_THIS_NODE);
  
  unicycle_kinematics_simulator unicycle_kinematics_simulator_node;
   
  unicycle_kinematics_simulator_node.Prepare();
  
  unicycle_kinematics_simulator_node.RunPeriodically();
  
  unicycle_kinematics_simulator_node.Shutdown();
  
  return (0);
}

