#include "RosNode/RosNode.h"
#include <std_msgs/String.h>

/* (0) defining the internal periodic phase */
void RosNode::_PeriodicTask(void)
{
    // Do all the periodic computation here 

    //  Publish results 
    std_msgs::String stringMsg;
    std::stringstream ss("I'm alive!");
    stringMsg.data = ss.str();
    outTopic_publisher.publish(stringMsg);
}

/* (1) beginning phase */
void RosNode::Prepare(void)
{
    // Retrieve parameters from ROS parameter server
    std::string full_param_name;

    // run_period : from ROS server
    full_param_name = ros::this_node::getName() + "/run_period";
    if (false == Handle.getParam(FullParamName, run_period))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

    // example_parameter : from ROS server
    full_param_name = ros::this_node::getName() + "/example_parameter";
    if (false == Handle.getParam(full_param_name, _param))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

    ROS_INFO("Node %s ready to run.", ros::this_node::getName().c_str());
}

/* (2) running phase */
void RosNode::RunPeriodically(float run_period)
{
    ros::Rate LoopRate(1.0/Period);

    ROS_INFO("Node %s running periodically (T=%.2fs, f=%.2fHz).", ros::this_node::getName().c_str(), Period, 1.0/Period);

    while (ros::ok())
    {
        PeriodicTask();

        ros::spinOnce();

        LoopRate.sleep();
    }
}

/* (3) ending phase */
void RosNode::Shutdown(void)
{
    // Nothing to be done
    ROS_INFO("Node %s shutting down.", ros::this_node::getName().c_str());
}