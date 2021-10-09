#include "node_template/RosNode.h"
#include <std_msgs/String.h>

/* Retrieve parameters from ROS parameter server */
void node_template::Prepare(void)
{
    std::string full_param_name;

    // run_period : from ROS server
    full_param_name = ros::this_node::getName() + "/run_period";
    if (false == Handle.getParam(FullParamName, run_period))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

    // example_parameter : from ROS server
    full_param_name = ros::this_node::getName() + "/example_parameter";
    if (false == Handle.getParam(full_param_name, _param))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

    /* ROS topics */
    inTopic_subscriber = Handle.subscribe("/in_topic", 1, &node_template::inTopic_MessageCallback, this);
    outTopic_publisher = Handle.advertise<std_msgs::String>("/out_topic", 1);

    /* Initialize node state */
    in_data = 0.0;

    ROS_INFO("Node %s ready to run.", ros::this_node::getName().c_str());
}

void node_template::RunPeriodically(float Period)
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

void node_template::Shutdown(void)
{
    // Nothing to be done
    ROS_INFO("Node %s shutting down.", ros::this_node::getName().c_str());
}

void node_template::inTopic_MessageCallback(const std_msgs::Float64::ConstPtr& msg)
{
    /*  Read message and store information */
    in_data = msg->data;
}

void node_template::PeriodicTask(void)
{
    /* Do all the periodic computation here */

    /*  Publish results */
    std_msgs::String stringMsg;
    std::stringstream ss("I'm alive!");
    stringMsg.data = ss.str();
    outTopic_publisher.publish(stringMsg);
}
